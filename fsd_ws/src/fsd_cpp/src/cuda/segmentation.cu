// CUDA perception kernels.
//
// 1. segment_kernel: BGR8 -> HSV -> three color masks + grayscale, one pass
//    over the image (the per-pixel heavy lifting).
// 2. gray_kernel: BGR8 -> grayscale (right image of the stereo pair).
// 3. sad_match_kernel: batched stereo SAD matching — one block per
//    detection, threads parallel over disparities, shared-memory reduction
//    keeping the two best candidates for the ratio test.
//
// Thresholds and grayscale weights mirror the CPU fallback exactly.

#include "fsd_cpp/segmentation.hpp"

#include <cuda_runtime.h>

namespace
{

constexpr int kMatchThreads = 128;

__device__ inline void bgr_to_hsv(uint8_t b, uint8_t g, uint8_t r,
                                  int & h, int & s, int & v)
{
  const int mx = max(b, max(g, r));
  const int mn = min(b, min(g, r));
  v = mx;
  s = (mx == 0) ? 0 : (255 * (mx - mn)) / mx;
  if (mx == mn) {
    h = 0;
    return;
  }
  const int d = mx - mn;
  int hh;
  if (mx == r) {
    hh = (60 * (static_cast<int>(g) - static_cast<int>(b))) / d;
  } else if (mx == g) {
    hh = 120 + (60 * (static_cast<int>(b) - static_cast<int>(r))) / d;
  } else {
    hh = 240 + (60 * (static_cast<int>(r) - static_cast<int>(g))) / d;
  }
  if (hh < 0) {
    hh += 360;
  }
  h = hh / 2;  // 0..179
}

// OpenCV BGR2GRAY fixed-point weights (must match the CPU path).
__device__ inline uint8_t bgr_to_gray(uint8_t b, uint8_t g, uint8_t r)
{
  return static_cast<uint8_t>((1868 * b + 9617 * g + 4899 * r + 8192) >> 14);
}

__global__ void segment_kernel(const uint8_t * __restrict__ bgr,
                               int n_pixels,
                               uint8_t * __restrict__ mb,
                               uint8_t * __restrict__ my,
                               uint8_t * __restrict__ mo,
                               uint8_t * __restrict__ gray)
{
  const int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= n_pixels) {
    return;
  }
  const uint8_t b = bgr[3 * i + 0];
  const uint8_t g = bgr[3 * i + 1];
  const uint8_t r = bgr[3 * i + 2];
  int h, s, v;
  bgr_to_hsv(b, g, r, h, s, v);
  // Thresholds tuned on live FSDS frames (overexposed, desaturated cones):
  // blue S>=105 rejects the ~S90 blue sky; yellow S>=50 catches the
  // desaturated yellow cone (measured S~63). See MEMORY.md.
  mb[i] = (h >= 100 && h <= 130 && s >= 105 && v >= 60) ? 255 : 0;
  my[i] = (h >= 20 && h <= 38 && s >= 50 && v >= 90) ? 255 : 0;
  mo[i] = (h >= 5 && h <= 18 && s >= 90 && v >= 90) ? 255 : 0;
  gray[i] = bgr_to_gray(b, g, r);
}

__global__ void gray_kernel(const uint8_t * __restrict__ bgr, int n_pixels,
                            uint8_t * __restrict__ gray)
{
  const int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= n_pixels) {
    return;
  }
  gray[i] = bgr_to_gray(bgr[3 * i], bgr[3 * i + 1], bgr[3 * i + 2]);
}

// Packed candidate: SAD in the high 32 bits, disparity in the low 32 —
// integer comparison then orders by SAD first, smaller disparity on ties.
__global__ void sad_match_kernel(const uint8_t * __restrict__ L,
                                 const uint8_t * __restrict__ R,
                                 int w,
                                 const int * __restrict__ cxs,
                                 const int * __restrict__ cys,
                                 int n, int ph, int max_disp,
                                 int * __restrict__ best_disp,
                                 float * __restrict__ ratio)
{
  const int det = blockIdx.x;
  if (det >= n) {
    return;
  }
  const int u = cxs[det];
  const int v = cys[det];
  const unsigned long long kInf = ~0ULL;

  unsigned long long b1 = kInf, b2 = kInf;   // per-thread two best
  const int limit = min(max_disp, u - ph);
  for (int d = threadIdx.x + 1; d <= limit; d += blockDim.x) {
    long sad = 0;
    for (int dy = -ph; dy <= ph; ++dy) {
      const uint8_t * lrow = L + (v + dy) * w;
      const uint8_t * rrow = R + (v + dy) * w;
      for (int dx = -ph; dx <= ph; ++dx) {
        sad += abs(static_cast<int>(lrow[u + dx]) -
                   static_cast<int>(rrow[u - d + dx]));
      }
    }
    const unsigned long long p =
      (static_cast<unsigned long long>(sad) << 32) | static_cast<unsigned>(d);
    if (p < b1) {
      b2 = b1;
      b1 = p;
    } else if (p < b2) {
      b2 = p;
    }
  }

  __shared__ unsigned long long s1[kMatchThreads];
  __shared__ unsigned long long s2[kMatchThreads];
  s1[threadIdx.x] = b1;
  s2[threadIdx.x] = b2;
  __syncthreads();
  for (int s = blockDim.x / 2; s > 0; s >>= 1) {
    if (threadIdx.x < static_cast<unsigned>(s)) {
      const unsigned long long a1 = s1[threadIdx.x], a2 = s2[threadIdx.x];
      const unsigned long long c1 = s1[threadIdx.x + s], c2 = s2[threadIdx.x + s];
      // two smallest of {a1, a2, c1, c2} (a1<=a2, c1<=c2 by construction)
      if (a1 < c1) {
        s1[threadIdx.x] = a1;
        s2[threadIdx.x] = min(a2, c1);
      } else {
        s1[threadIdx.x] = c1;
        s2[threadIdx.x] = min(c2, a1);
      }
    }
    __syncthreads();
  }
  if (threadIdx.x == 0) {
    if (s1[0] == kInf) {
      best_disp[det] = -1;
      ratio[det] = 1.0f;
    } else {
      best_disp[det] = static_cast<int>(s1[0] & 0xffffffffULL);
      const float b = static_cast<float>(s1[0] >> 32);
      if (s2[0] == kInf) {
        ratio[det] = 1.0f;   // single candidate: ambiguous, let caller reject
      } else {
        const float sec = static_cast<float>(s2[0] >> 32);
        ratio[det] = sec > 0.0f ? b / sec : 1.0f;
      }
    }
  }
}

// ------------------------------------------------------- persistent state
uint8_t * d_bgr = nullptr;      // frame upload staging (left or right)
uint8_t * d_masks = nullptr;    // 3 masks back-to-back
uint8_t * d_left_gray = nullptr;
uint8_t * d_right_gray = nullptr;
size_t cap_pixels = 0;
int left_w = 0, left_h = 0;
int right_w = 0, right_h = 0;

int * d_cx = nullptr;
int * d_cy = nullptr;
int * d_disp = nullptr;
float * d_ratio = nullptr;
int cap_dets = 0;

bool ensure_capacity(size_t n_pixels)
{
  if (n_pixels <= cap_pixels) {
    return true;
  }
  for (uint8_t ** p : {&d_bgr, &d_masks, &d_left_gray, &d_right_gray}) {
    if (*p) {
      cudaFree(*p);
      *p = nullptr;
    }
  }
  cap_pixels = 0;
  if (cudaMalloc(&d_bgr, n_pixels * 3) != cudaSuccess ||
      cudaMalloc(&d_masks, n_pixels * 3) != cudaSuccess ||
      cudaMalloc(&d_left_gray, n_pixels) != cudaSuccess ||
      cudaMalloc(&d_right_gray, n_pixels) != cudaSuccess)
  {
    return false;
  }
  cap_pixels = n_pixels;
  return true;
}

bool ensure_det_capacity(int n)
{
  if (n <= cap_dets) {
    return true;
  }
  for (void ** p : {reinterpret_cast<void **>(&d_cx),
                    reinterpret_cast<void **>(&d_cy),
                    reinterpret_cast<void **>(&d_disp),
                    reinterpret_cast<void **>(&d_ratio)})
  {
    if (*p) {
      cudaFree(*p);
      *p = nullptr;
    }
  }
  cap_dets = 0;
  if (cudaMalloc(&d_cx, n * sizeof(int)) != cudaSuccess ||
      cudaMalloc(&d_cy, n * sizeof(int)) != cudaSuccess ||
      cudaMalloc(&d_disp, n * sizeof(int)) != cudaSuccess ||
      cudaMalloc(&d_ratio, n * sizeof(float)) != cudaSuccess)
  {
    return false;
  }
  cap_dets = n;
  return true;
}

}  // namespace

namespace fsd_cuda
{

bool segment(const uint8_t * bgr, int width, int height,
             uint8_t * mask_blue, uint8_t * mask_yellow, uint8_t * mask_orange)
{
  const size_t n = static_cast<size_t>(width) * height;
  if (n == 0 || !ensure_capacity(n)) {
    return false;
  }
  if (cudaMemcpy(d_bgr, bgr, n * 3, cudaMemcpyHostToDevice) != cudaSuccess) {
    return false;
  }
  const int threads = 256;
  const int blocks = static_cast<int>((n + threads - 1) / threads);
  segment_kernel<<<blocks, threads>>>(d_bgr, static_cast<int>(n),
                                      d_masks, d_masks + n, d_masks + 2 * n,
                                      d_left_gray);
  if (cudaGetLastError() != cudaSuccess) {
    return false;
  }
  if (cudaMemcpy(mask_blue, d_masks, n, cudaMemcpyDeviceToHost) != cudaSuccess ||
      cudaMemcpy(mask_yellow, d_masks + n, n, cudaMemcpyDeviceToHost) != cudaSuccess ||
      cudaMemcpy(mask_orange, d_masks + 2 * n, n, cudaMemcpyDeviceToHost) != cudaSuccess)
  {
    return false;
  }
  left_w = width;
  left_h = height;
  return true;
}

bool upload_right(const uint8_t * bgr, int width, int height)
{
  const size_t n = static_cast<size_t>(width) * height;
  if (n == 0 || !ensure_capacity(n)) {
    return false;
  }
  if (cudaMemcpy(d_bgr, bgr, n * 3, cudaMemcpyHostToDevice) != cudaSuccess) {
    return false;
  }
  const int threads = 256;
  const int blocks = static_cast<int>((n + threads - 1) / threads);
  gray_kernel<<<blocks, threads>>>(d_bgr, static_cast<int>(n), d_right_gray);
  if (cudaGetLastError() != cudaSuccess) {
    return false;
  }
  right_w = width;
  right_h = height;
  return true;
}

bool match_disparities(const int * cx, const int * cy, int n,
                       int patch_half, int max_disp,
                       int * best_disp, float * ratio)
{
  if (n <= 0 || left_w == 0 || left_w != right_w || left_h != right_h) {
    return false;
  }
  if (!ensure_det_capacity(n)) {
    return false;
  }
  if (cudaMemcpy(d_cx, cx, n * sizeof(int), cudaMemcpyHostToDevice) != cudaSuccess ||
      cudaMemcpy(d_cy, cy, n * sizeof(int), cudaMemcpyHostToDevice) != cudaSuccess)
  {
    return false;
  }
  sad_match_kernel<<<n, kMatchThreads>>>(d_left_gray, d_right_gray, left_w,
                                         d_cx, d_cy, n, patch_half, max_disp,
                                         d_disp, d_ratio);
  if (cudaGetLastError() != cudaSuccess) {
    return false;
  }
  if (cudaMemcpy(best_disp, d_disp, n * sizeof(int), cudaMemcpyDeviceToHost) != cudaSuccess ||
      cudaMemcpy(ratio, d_ratio, n * sizeof(float), cudaMemcpyDeviceToHost) != cudaSuccess)
  {
    return false;
  }
  return true;
}

}  // namespace fsd_cuda
