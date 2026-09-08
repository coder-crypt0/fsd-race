// GPU perception primitives. Implemented in src/cuda/segmentation.cu when
// built with CUDA; the stereo node holds equivalent CPU fallbacks. Every
// function returns false on any CUDA error so callers can fall back.
#pragma once

#include <cstdint>

namespace fsd_cuda
{

// One-pass BGR -> HSV -> three binary masks (blue / yellow / orange) on the
// GPU. Also converts and CACHES the left grayscale image on the device for
// a subsequent match_disparities() call. Buffers are managed internally.
bool segment(const uint8_t * bgr, int width, int height,
             uint8_t * mask_blue, uint8_t * mask_yellow, uint8_t * mask_orange);

// Upload + grayscale-convert the right image of the rectified pair onto the
// device (call from the right-image callback).
bool upload_right(const uint8_t * bgr, int width, int height);

// Batched stereo SAD matching, entirely on the GPU: for each detection
// center (cx[i], cy[i]) search disparities 1..max_disp along the epipolar
// row. One CUDA block per detection, threads parallel over disparities,
// shared-memory reduction keeps the best AND second-best candidate for the
// ratio test. Outputs per detection: best disparity (-1 = no candidate)
// and ratio = best_sad / second_sad (1.0 when no second candidate; callers
// reject ambiguous matches with ratio > 0.8).
// Preconditions (caller enforces): patch fully inside both images, left
// gray cached by segment(), right uploaded with matching dimensions.
bool match_disparities(const int * cx, const int * cy, int n,
                       int patch_half, int max_disp,
                       int * best_disp, float * ratio);

}  // namespace fsd_cuda
