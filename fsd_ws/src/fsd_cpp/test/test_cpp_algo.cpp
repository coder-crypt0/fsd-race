// Correctness tests for the algorithm block extracted VERBATIM from
// fsd_cpp/src/path_planning_node.cpp (see algo_extract.hpp).
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <map>
#include <random>
#include <optional>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "algo_extract.hpp"

int main()
{
  std::mt19937 rng(7);
  std::uniform_real_distribution<double> jitter(-0.05, 0.05);

  // ---- 1. Delaunay on a jittered grid: verify the empty-circumcircle
  // property for EVERY triangle against EVERY point (the definition).
  std::vector<P2> pts;
  for (int i = 0; i < 8; ++i) {
    for (int j = 0; j < 6; ++j) {
      pts.push_back({i * 3.0 + jitter(rng), j * 3.0 + jitter(rng)});
    }
  }
  auto tris = delaunay(pts);
  if (tris.empty()) { std::printf("FAIL: delaunay returned no triangles\n"); return 1; }
  int violations = 0;
  for (const auto & t : tris) {
    for (int q = 0; q < static_cast<int>(pts.size()); ++q) {
      if (q == t.a || q == t.b || q == t.c) continue;
      if (in_circumcircle(pts[t.a], pts[t.b], pts[t.c], pts[q])) ++violations;
    }
  }
  std::printf("1. delaunay: %zu pts -> %zu tris, circumcircle violations: %d\n",
              pts.size(), tris.size(), violations);
  if (violations != 0) { std::printf("FAIL\n"); return 1; }

  // Structural check: boundary edges (appearing in exactly one triangle)
  // must form a single closed loop — every boundary vertex has exactly two
  // boundary edges — which rules out holes. Then Euler's identity
  // T = 2n - 2 - h must hold with h = boundary loop length.
  {
    std::map<std::pair<int, int>, int> edge_count;
    for (const auto & t : tris) {
      const std::pair<int, int> es[3] = {
        {std::min(t.a, t.b), std::max(t.a, t.b)},
        {std::min(t.b, t.c), std::max(t.b, t.c)},
        {std::min(t.c, t.a), std::max(t.c, t.a)}};
      for (const auto & e : es) edge_count[e] += 1;
    }
    std::map<int, int> boundary_deg;
    size_t h = 0;
    bool over_shared = false;
    for (const auto & [e, cnt] : edge_count) {
      if (cnt == 1) {
        ++h;
        boundary_deg[e.first] += 1;
        boundary_deg[e.second] += 1;
      } else if (cnt != 2) {
        over_shared = true;
      }
    }
    bool loop_ok = !over_shared;
    for (const auto & [v, deg] : boundary_deg) {
      (void)v;
      if (deg != 2) loop_ok = false;
    }
    const size_t expected = 2 * pts.size() - 2 - h;
    std::printf("   structure: boundary loop h=%zu closed=%s, "
                "euler expected T=%zu, got T=%zu\n",
                h, loop_ok ? "yes" : "NO", expected, tris.size());
    if (!loop_ok || tris.size() != expected) {
      std::printf("FAIL: triangulation structure broken\n");
      return 1;
    }
  }

  // ---- 2. Delaunay of track-like double ring (blue/yellow boundary).
  std::vector<P2> ring;
  for (int i = 0; i < 24; ++i) {
    const double a = 2.0 * M_PI * i / 24;
    // jitter breaks exact cocircularity (degenerate for strict incircle
    // checks; any triangulation of cocircular points is valid Delaunay)
    ring.push_back({12.0 * std::cos(a) + jitter(rng),
                    12.0 * std::sin(a) + jitter(rng)});
    ring.push_back({8.5 * std::cos(a) + jitter(rng),
                    8.5 * std::sin(a) + jitter(rng)});
  }
  auto ring_tris = delaunay(ring);
  violations = 0;
  for (const auto & t : ring_tris) {
    for (int q = 0; q < static_cast<int>(ring.size()); ++q) {
      if (q == t.a || q == t.b || q == t.c) continue;
      if (in_circumcircle(ring[t.a], ring[t.b], ring[t.c], ring[q])) ++violations;
    }
  }
  std::printf("2. ring delaunay: %zu tris, violations: %d\n",
              ring_tris.size(), violations);
  if (ring_tris.empty() || violations != 0) { std::printf("FAIL\n"); return 1; }

  // ---- 3. Catmull-Rom on a radius-10 arc: curvature must be ~0.1,
  // sample spacing ~0.5 m, headings tangent to the circle.
  std::vector<P2> arc;
  for (int i = 0; i <= 20; ++i) {
    const double a = M_PI * i / 40.0;   // quarter circle, points every ~0.8 m
    arc.push_back({10.0 * std::cos(a), 10.0 * std::sin(a)});
  }
  auto samples = catmull_rom(arc, 0.5);
  if (samples.size() < 20) { std::printf("FAIL: too few samples\n"); return 1; }
  double max_curv_err = 0.0, max_r_err = 0.0, max_ds = 0.0, min_ds = 1e9;
  for (size_t i = 0; i < samples.size(); ++i) {
    // interior only: phantom-endpoint segments are extrapolations
    if (i > 2 && i + 3 < samples.size()) {
      max_curv_err = std::max(max_curv_err, std::abs(std::abs(samples[i].curvature) - 0.1));
    }
    max_r_err = std::max(max_r_err,
      std::abs(std::hypot(samples[i].x, samples[i].y) - 10.0));
    if (i > 0) {
      const double ds = std::hypot(samples[i].x - samples[i-1].x,
                                   samples[i].y - samples[i-1].y);
      max_ds = std::max(max_ds, ds);
      min_ds = std::min(min_ds, ds);
    }
  }
  std::printf("3. catmull-rom: %zu samples, |k-0.1| max %.4f, radius err %.3f m, "
              "ds in [%.2f, %.2f]\n",
              samples.size(), max_curv_err, max_r_err, min_ds, max_ds);
  if (max_curv_err > 0.02 || max_r_err > 0.05 || max_ds > 1.0 || min_ds < 0.1) {
    std::printf("FAIL\n");
    return 1;
  }

  // ---- 4. Degenerate inputs must not crash or return garbage.
  if (!delaunay({}).empty() ||
      !delaunay({{0, 0}, {1, 1}}).empty())
  {
    std::printf("FAIL: degenerate input\n");
    return 1;
  }
  auto collinear = delaunay({{0, 0}, {1, 0}, {2, 0}, {3, 0}});
  std::printf("4. degenerate inputs OK (collinear -> %zu tris)\n", collinear.size());

  std::printf("\nALL C++ ALGORITHM TESTS PASSED\n");
  return 0;
}
