// BLOCK 5 — PATH PLANNING (C++). Two regimes, chosen by what Block 4 knows:
//
// EXPLORE (first lap): Delaunay (self-contained Bowyer-Watson) over a local
// cone window -> opposite-side edge midpoints -> greedy forward ordering ->
// Catmull-Rom spline resampled at 0.5 m with analytic heading and curvature.
// Speed is capped by the sensor horizon.
//
// KNOWN (map closed): the same midpoint extraction over the WHOLE map, ordered
// into a closed loop, then a bounded minimum-curvature optimization turns that
// centerline into a racing line. Built once, cached, and published as a
// rolling forward window that wraps past the start line. Speed is no longer
// bounded by what the camera can see, because the geometry is already known.
//
// OBSTACLE AVOIDANCE in both regimes: mapped objects inside the corridor are
// removed from the boundary set and passed on whichever side has room, by
// deforming the published path with a raised-cosine lateral offset. No
// feasible gap -> keep publishing a valid path but cap speed at zero.
//
// Fallback ladder identical to the Python reference: hold last valid
// (DEGRADED), then empty path + ERROR after 1 s.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <numeric>
#include <iterator>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <fsd_msgs/msg/cone_map.hpp>
#include <fsd_msgs/msg/cone_map_entry.hpp>
#include <fsd_msgs/msg/cone3_d_array.hpp>
#include <fsd_msgs/msg/cone_detection2_d.hpp>
#include <fsd_msgs/msg/path_point.hpp>
#include <fsd_msgs/msg/path_point_array.hpp>
#include <fsd_msgs/msg/speed_limit.hpp>
#include <fsd_msgs/msg/track_status.hpp>
#include <fsd_msgs/msg/heartbeat.hpp>

#include "fsd_cpp/common.hpp"

using fsd_msgs::msg::ConeDetection2D;
using fsd_msgs::msg::ConeMapEntry;
using fsd_msgs::msg::Heartbeat;
using fsd_msgs::msg::PathPointArray;
using fsd_msgs::msg::SpeedLimit;
using fsd_msgs::msg::TrackStatus;

namespace
{

struct P2
{
  double x, y;
};

inline double dist(const P2 & a, const P2 & b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

// ------------------------------------------------ Bowyer-Watson Delaunay
// True iff q lies inside the circumcircle of triangle (a, b, c).
bool in_circumcircle(const P2 & a, const P2 & b, const P2 & c, const P2 & q)
{
  const double ax = a.x - q.x, ay = a.y - q.y;
  const double bx = b.x - q.x, by = b.y - q.y;
  const double cx = c.x - q.x, cy = c.y - q.y;
  const double det =
    (ax * ax + ay * ay) * (bx * cy - cx * by) -
    (bx * bx + by * by) * (ax * cy - cx * ay) +
    (cx * cx + cy * cy) * (ax * by - bx * ay);
  const double orient = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
  return orient > 0.0 ? det > 0.0 : det < 0.0;
}

struct Tri
{
  int a, b, c;
};

// O(n^2) incremental Delaunay — plenty fast for <=200 cones at 10 Hz.
std::vector<Tri> delaunay(const std::vector<P2> & pts_in)
{
  const int n = static_cast<int>(pts_in.size());
  if (n < 3) {
    return {};
  }
  std::vector<P2> pts = pts_in;

  double min_x = pts[0].x, max_x = pts[0].x;
  double min_y = pts[0].y, max_y = pts[0].y;
  for (const auto & p : pts) {
    min_x = std::min(min_x, p.x);
    max_x = std::max(max_x, p.x);
    min_y = std::min(min_y, p.y);
    max_y = std::max(max_y, p.y);
  }
  const double d = std::max({max_x - min_x, max_y - min_y, 1.0}) * 100.0;
  const double mx = (min_x + max_x) / 2.0;
  const double my = (min_y + max_y) / 2.0;
  pts.push_back({mx - 20.0 * d, my - d});        // index n
  pts.push_back({mx + 20.0 * d, my - d});        // index n+1
  pts.push_back({mx, my + 20.0 * d});            // index n+2

  std::vector<Tri> tris{{n, n + 1, n + 2}};
  for (int i = 0; i < n; ++i) {
    std::vector<Tri> bad, good;
    for (const auto & t : tris) {
      if (in_circumcircle(pts[t.a], pts[t.b], pts[t.c], pts[i])) {
        bad.push_back(t);
      } else {
        good.push_back(t);
      }
    }
    // Boundary polygon: edges of bad triangles that appear exactly once.
    std::map<std::pair<int, int>, int> edge_count;
    for (const auto & t : bad) {
      const std::pair<int, int> es[3] = {
        {std::min(t.a, t.b), std::max(t.a, t.b)},
        {std::min(t.b, t.c), std::max(t.b, t.c)},
        {std::min(t.c, t.a), std::max(t.c, t.a)}};
      for (const auto & e : es) {
        edge_count[e] += 1;
      }
    }
    tris.swap(good);
    for (const auto & [e, cnt] : edge_count) {
      if (cnt == 1) {
        tris.push_back({e.first, e.second, i});
      }
    }
  }
  // Drop triangles touching the super-triangle.
  std::vector<Tri> out;
  for (const auto & t : tris) {
    if (t.a < n && t.b < n && t.c < n) {
      out.push_back(t);
    }
  }
  return out;
}

// ---------------------------------------------------------- Catmull-Rom
struct Sample
{
  double x, y, heading, curvature;
};

// Uniform Catmull-Rom through the ordered chain, resampled at ~spacing.
std::vector<Sample> catmull_rom(const std::vector<P2> & chain, double spacing)
{
  if (chain.size() < 2) {
    return {};
  }
  // Phantom endpoints: reflect first/last so the curve reaches both ends.
  std::vector<P2> c;
  c.push_back({2.0 * chain.front().x - chain[1].x,
               2.0 * chain.front().y - chain[1].y});
  c.insert(c.end(), chain.begin(), chain.end());
  const size_t m = chain.size();
  c.push_back({2.0 * chain.back().x - chain[m - 2].x,
               2.0 * chain.back().y - chain[m - 2].y});

  std::vector<Sample> out;
  for (size_t seg = 1; seg + 2 < c.size(); ++seg) {
    const P2 & p0 = c[seg - 1];
    const P2 & p1 = c[seg];
    const P2 & p2 = c[seg + 1];
    const P2 & p3 = c[seg + 2];
    const double seg_len = dist(p1, p2);
    const int steps = std::max(1, static_cast<int>(std::round(seg_len / spacing)));
    const bool last_seg = (seg + 3 == c.size());
    const int t_end = last_seg ? steps : steps - 1;   // avoid duplicate joints
    for (int j = 0; j <= t_end; ++j) {
      const double t = static_cast<double>(j) / steps;
      const double t2 = t * t;
      const double t3 = t2 * t;
      // C(t)     = 0.5 * (2P1 + (-P0+P2)t + (2P0-5P1+4P2-P3)t^2 + (-P0+3P1-3P2+P3)t^3)
      const double ax = 2.0 * p0.x - 5.0 * p1.x + 4.0 * p2.x - p3.x;
      const double ay = 2.0 * p0.y - 5.0 * p1.y + 4.0 * p2.y - p3.y;
      const double bx = -p0.x + 3.0 * p1.x - 3.0 * p2.x + p3.x;
      const double by = -p0.y + 3.0 * p1.y - 3.0 * p2.y + p3.y;
      const double x = 0.5 * (2.0 * p1.x + (-p0.x + p2.x) * t + ax * t2 + bx * t3);
      const double y = 0.5 * (2.0 * p1.y + (-p0.y + p2.y) * t + ay * t2 + by * t3);
      const double dx = 0.5 * ((-p0.x + p2.x) + 2.0 * ax * t + 3.0 * bx * t2);
      const double dy = 0.5 * ((-p0.y + p2.y) + 2.0 * ay * t + 3.0 * by * t2);
      const double ddx = 0.5 * (2.0 * ax + 6.0 * bx * t);
      const double ddy = 0.5 * (2.0 * ay + 6.0 * by * t);
      const double denom = std::pow(dx * dx + dy * dy, 1.5);
      out.push_back({x, y, std::atan2(dy, dx),
                     denom > 1e-9 ? (dx * ddy - dy * ddx) / denom : 0.0});
    }
  }
  return out;
}

// Closed Catmull-Rom: control-point indices wrap, so curvature stays
// continuous across the start line instead of showing a kink there.
// resample=false emits one sample per input point (heading and curvature OF
// those points), which keeps an optimized line index-aligned with the
// centerline it came from.
std::vector<Sample> periodic_catmull_rom(const std::vector<P2> & chain,
                                        double spacing,
                                        const std::vector<double> & widths,
                                        std::vector<double> * out_widths,
                                        bool resample = true)
{
  const size_t n = chain.size();
  std::vector<Sample> out;
  if (out_widths) {
    out_widths->clear();
  }
  if (n < 4) {
    return out;
  }
  for (size_t i = 0; i < n; ++i) {
    const P2 & p0 = chain[(i + n - 1) % n];
    const P2 & p1 = chain[i];
    const P2 & p2 = chain[(i + 1) % n];
    const P2 & p3 = chain[(i + 2) % n];
    const double w1 = widths.empty() ? 0.0 : widths[i];
    const double w2 = widths.empty() ? 0.0 : widths[(i + 1) % n];
    int steps = 1;
    if (resample) {
      steps = std::max(1, static_cast<int>(std::round(dist(p1, p2) / spacing)));
    }
    for (int j = 0; j < steps; ++j) {     // j == steps belongs to the next seg
      const double t = static_cast<double>(j) / steps;
      const double t2 = t * t;
      const double t3 = t2 * t;
      const double ax = 2.0 * p0.x - 5.0 * p1.x + 4.0 * p2.x - p3.x;
      const double ay = 2.0 * p0.y - 5.0 * p1.y + 4.0 * p2.y - p3.y;
      const double bx = -p0.x + 3.0 * p1.x - 3.0 * p2.x + p3.x;
      const double by = -p0.y + 3.0 * p1.y - 3.0 * p2.y + p3.y;
      const double x = 0.5 * (2.0 * p1.x + (-p0.x + p2.x) * t + ax * t2 + bx * t3);
      const double y = 0.5 * (2.0 * p1.y + (-p0.y + p2.y) * t + ay * t2 + by * t3);
      const double dx = 0.5 * ((-p0.x + p2.x) + 2.0 * ax * t + 3.0 * bx * t2);
      const double dy = 0.5 * ((-p0.y + p2.y) + 2.0 * ay * t + 3.0 * by * t2);
      const double ddx = 0.5 * (2.0 * ax + 6.0 * bx * t);
      const double ddy = 0.5 * (2.0 * ay + 6.0 * by * t);
      const double denom = std::pow(dx * dx + dy * dy, 1.5);
      out.push_back({x, y, std::atan2(dy, dx),
                     denom > 1e-9 ? (dx * ddy - dy * ddx) / denom : 0.0});
      if (out_widths) {
        out_widths->push_back(w1 + (w2 - w1) * t);
      }
    }
  }
  return out;
}

// Light Laplacian smoothing of a control-point chain.
//
// Delaunay midpoints are unevenly spaced (1.1-1.9 m on a 3 m cone pitch) and
// zig-zag by centimetres. Uniformly parameterised Catmull-Rom turns that
// jitter into curvature spikes — measured 0.51 1/m, a 2 m radius, on a track
// whose real minimum is 0.139 — and the velocity profile then brakes for
// corners that do not exist. One pass drops the peak to 0.17 1/m. The cost is
// a small inward bias on the tightest corners (half the chord sagitta, ~8 cm),
// which is why callers still verify cone clearance afterwards.
std::vector<P2> smooth_chain(const std::vector<P2> & pts, int passes,
                             double weight, bool closed)
{
  std::vector<P2> out = pts;
  const size_t n = out.size();
  if (n < 4 || passes <= 0) {
    return out;
  }
  for (int pass = 0; pass < passes; ++pass) {
    const std::vector<P2> prev = out;
    for (size_t i = 0; i < n; ++i) {
      if (!closed && (i == 0 || i + 1 == n)) {
        continue;                        // keep the ends of an open chain put
      }
      const P2 & a = prev[(i + n - 1) % n];
      const P2 & b = prev[i];
      const P2 & c = prev[(i + 1) % n];
      out[i] = {(1.0 - 2.0 * weight) * b.x + weight * (a.x + c.x),
                (1.0 - 2.0 * weight) * b.y + weight * (a.y + c.y)};
    }
  }
  return out;
}

// Bounded minimum-curvature line through a closed corridor. Each point may
// slide along its normal by alpha_i, |alpha_i| <= bounds_i. Minimizes
// sum |p_{i-1} - 2 p_i + p_{i+1}|^2 (discrete curvature energy at uniform
// spacing) plus reg * sum alpha_i^2 by projected gradient descent — convex, so
// the projection cannot get stuck. The regularizer matters on straights, where
// curvature does not care where in the corridor the line runs; it keeps those
// stretches mid-track instead of drifting onto a boundary. `step` must stay
// below 2/lambda_max of the 4th-difference operator (= 32) to contract.
std::vector<double> optimize_raceline(const std::vector<P2> & pts,
                                      const std::vector<P2> & normals,
                                      const std::vector<double> & bounds,
                                      int iters, double step, double reg)
{
  const size_t n = pts.size();
  std::vector<double> a(n, 0.0);
  if (n < 8) {
    return a;
  }
  std::vector<P2> p(n), r(n);
  for (int it = 0; it < iters; ++it) {
    for (size_t i = 0; i < n; ++i) {
      p[i] = {pts[i].x + a[i] * normals[i].x, pts[i].y + a[i] * normals[i].y};
    }
    for (size_t i = 0; i < n; ++i) {
      const P2 & pm = p[(i + n - 1) % n];
      const P2 & pp = p[(i + 1) % n];
      r[i] = {pm.x - 2.0 * p[i].x + pp.x, pm.y - 2.0 * p[i].y + pp.y};
    }
    for (size_t i = 0; i < n; ++i) {
      const P2 & rm = r[(i + n - 1) % n];
      const P2 & rp = r[(i + 1) % n];
      const double gx = rm.x - 2.0 * r[i].x + rp.x;
      const double gy = rm.y - 2.0 * r[i].y + rp.y;
      const double grad = 2.0 * (gx * normals[i].x + gy * normals[i].y) +
                          2.0 * reg * a[i];
      a[i] = std::clamp(a[i] - step * grad, -bounds[i], bounds[i]);
    }
  }
  return a;
}

std::vector<double> path_stations(const std::vector<P2> & pts)
{
  std::vector<double> cum(pts.size(), 0.0);
  for (size_t i = 1; i < pts.size(); ++i) {
    cum[i] = cum[i - 1] + dist(pts[i], pts[i - 1]);
  }
  return cum;
}

struct Hit
{
  double station, lateral, distance;
};

// Closest point on a polyline; lateral is positive to the LEFT of travel.
std::optional<Hit> point_to_path(const std::vector<P2> & pts,
                                 const std::vector<double> & cum,
                                 double x, double y)
{
  std::optional<Hit> best;
  for (size_t i = 0; i + 1 < pts.size(); ++i) {
    const double ex = pts[i + 1].x - pts[i].x;
    const double ey = pts[i + 1].y - pts[i].y;
    const double seg2 = ex * ex + ey * ey;
    if (seg2 < 1e-12) {
      continue;
    }
    double t = ((x - pts[i].x) * ex + (y - pts[i].y) * ey) / seg2;
    t = std::clamp(t, 0.0, 1.0);
    const double cx = pts[i].x + t * ex;
    const double cy = pts[i].y + t * ey;
    const double d = std::hypot(x - cx, y - cy);
    if (!best || d < best->distance) {
      const double seg = std::sqrt(seg2);
      const double lat = ((x - pts[i].x) * (-ey) + (y - pts[i].y) * ex) / seg;
      best = Hit{cum[i] + t * seg, lat, d};
    }
  }
  return best;
}

double min_clearance(const std::vector<Sample> & samples,
                     const std::vector<P2> & cones)
{
  if (cones.empty()) {
    return std::numeric_limits<double>::infinity();
  }
  double best = std::numeric_limits<double>::infinity();
  for (const auto & s : samples) {
    for (const auto & c : cones) {
      best = std::min(best, std::hypot(s.x - c.x, s.y - c.y));
    }
  }
  return best;
}

// 5-point moving average on curvature only. The car cannot respond to
// half-metre curvature spikes and the velocity profile turns them into speed
// chatter. `closed` wraps the window; otherwise it clamps at the ends.
std::vector<Sample> smooth_curvature(const std::vector<Sample> & samples,
                                     int half, bool closed)
{
  const int n = static_cast<int>(samples.size());
  std::vector<Sample> out = samples;
  for (int i = 0; i < n; ++i) {
    double acc = 0.0;
    int cnt = 0;
    for (int j = i - half; j <= i + half; ++j) {
      const int k = closed ? ((j % n) + n) % n : std::clamp(j, 0, n - 1);
      acc += samples[k].curvature;
      ++cnt;
    }
    out[i].curvature = acc / cnt;
  }
  return out;
}

struct PassChoice
{
  double alpha;         // lateral shift of the line, left positive
  double clearance;     // gap actually left to the object
};

// Which side to pass an object on, and by how much to move the line.
//
// The object sits `lat` to the left of the line; we need |lat - alpha| >=
// `clearance` while |alpha| stays inside `bound` (the corridor budget).
// Passing on the right means alpha <= lat - clearance, on the left
// alpha >= lat + clearance. Prefer whichever moves the line less; if neither
// fits, take the side that gets closest and report the reduced clearance so the
// caller can decide to stop instead.
PassChoice choose_pass(double lat, double bound, double clearance)
{
  const double a_right = lat - clearance;
  const double a_left = lat + clearance;
  const bool r_ok = std::abs(a_right) <= bound;
  const bool l_ok = std::abs(a_left) <= bound;
  double alpha;
  if (r_ok && l_ok) {
    alpha = std::abs(a_right) <= std::abs(a_left) ? a_right : a_left;
  } else if (r_ok) {
    alpha = a_right;
  } else if (l_ok) {
    alpha = a_left;
  } else {
    const double pick = std::abs(a_right) < std::abs(a_left) ? a_right : a_left;
    alpha = std::clamp(pick, -bound, bound);
  }
  return {alpha, std::abs(lat - alpha)};
}

// Heading and curvature of an OPEN deformed polyline: three-point circle
// through each point's neighbours — no spline refit needed at 0.5 m spacing.
// The end points have no two-sided neighbourhood, so their curvature is
// carried over from the adjacent point rather than left at zero; zero at the
// end of the window reads as "straight ahead, carry speed" to the velocity
// profile, and was the largest single source of disagreement with the
// analytic spline on identical geometry (0.059 1/m at the end against
// 0.001 1/m in the interior).
std::vector<Sample> resample_derivatives(const std::vector<P2> & pts,
                                         const std::vector<Sample> & fallback)
{
  const int n = static_cast<int>(pts.size());
  std::vector<Sample> out;
  out.reserve(n);
  if (n < 3) {
    for (int i = 0; i < n; ++i) {
      out.push_back({pts[i].x, pts[i].y, fallback[i].heading, 0.0});
    }
    return out;
  }
  for (int i = 0; i < n; ++i) {
    const int i0 = std::max(0, i - 1);
    const int i2 = std::min(n - 1, i + 1);
    const double h = std::atan2(pts[i2].y - pts[i0].y, pts[i2].x - pts[i0].x);
    double k = 0.0;
    if (i0 != i && i2 != i) {
      const double a = dist(pts[i], pts[i0]);
      const double b = dist(pts[i2], pts[i]);
      const double c = dist(pts[i2], pts[i0]);
      const double area2 = (pts[i].x - pts[i0].x) * (pts[i2].y - pts[i0].y) -
                           (pts[i].y - pts[i0].y) * (pts[i2].x - pts[i0].x);
      if (a * b * c > 1e-9) {
        k = 2.0 * area2 / (a * b * c);
      }
    }
    out.push_back({pts[i].x, pts[i].y, h, k});
  }
  out[0].curvature = out[1].curvature;
  out[n - 1].curvature = out[n - 2].curvature;
  return smooth_curvature(out, 2, false);
}

// Raised-cosine lateral offsets over +-blend. Where two objects overlap the
// strongest demand wins rather than the sum: opposing offsets must not cancel
// into a path that drives through both of them.
std::vector<Sample> apply_offsets(
  const std::vector<Sample> & samples, const std::vector<double> & cum_s,
  const std::vector<std::pair<double, double>> & shifts, double blend)
{
  std::vector<P2> pts;
  pts.reserve(samples.size());
  for (size_t i = 0; i < samples.size(); ++i) {
    double best = 0.0;
    for (const auto & [station, alpha] : shifts) {
      const double d = std::abs(cum_s[i] - station);
      if (d >= blend) {
        continue;
      }
      const double w = 0.5 * (1.0 + std::cos(M_PI * d / blend));
      if (std::abs(alpha * w) > std::abs(best)) {
        best = alpha * w;
      }
    }
    pts.push_back({samples[i].x + best * -std::sin(samples[i].heading),
                   samples[i].y + best * std::cos(samples[i].heading)});
  }
  return resample_derivatives(pts, samples);
}

}  // namespace

class PathPlanningNode : public rclcpp::Node
{
public:
  PathPlanningNode() : Node("path_planning")
  {
    declare_parameter<double>("window_ahead_m", 25.0);
    declare_parameter<double>("window_behind_m", 5.0);
    declare_parameter<double>("edge_min_m", 1.5);
    declare_parameter<double>("edge_max_m", 6.0);
    declare_parameter<double>("sample_spacing_m", 0.5);
    declare_parameter<double>("publish_rate_hz", 10.0);
    declare_parameter<double>("default_track_width_m", 3.5);
    declare_parameter<int>("chain_smooth_passes", 1);
    declare_parameter<double>("chain_smooth_weight", 0.25);
    // --- known-track racing line
    declare_parameter<bool>("raceline_enable", true);
    declare_parameter<double>("known_window_ahead_m", 60.0);
    declare_parameter<double>("known_window_behind_m", 3.0);
    declare_parameter<double>("raceline_margin_m", 1.0);
    declare_parameter<int>("raceline_iters", 400);
    declare_parameter<double>("raceline_step", 0.03);
    declare_parameter<double>("raceline_reg", 0.002);
    declare_parameter<double>("raceline_min_clearance_m", 0.9);
    declare_parameter<int>("global_rebuild_cone_delta", 3);
    declare_parameter<double>("global_rebuild_min_period_s", 2.0);
    // --- obstacle avoidance. obstacle_gate_m MUST stay below half of the
    // narrowest track width or boundary cones start looking like obstacles.
    declare_parameter<double>("obstacle_gate_m", 1.1);
    declare_parameter<double>("obstacle_clearance_m", 0.9);
    declare_parameter<double>("obstacle_min_clearance_m", 0.5);
    declare_parameter<double>("obstacle_blend_m", 5.0);
    // A lateral offset A blended over distance D peaks at pi^2*A*v^2/(2*D^2)
    // of lateral acceleration, so a fixed blend distance becomes undriveable
    // as speed rises. A blend TIME makes the demand speed-invariant:
    // D = t*v gives 7.3 m/s^2 at t = 0.8 s for the widest offset commanded.
    declare_parameter<double>("obstacle_blend_time_s", 0.8);
    declare_parameter<double>("obstacle_blend_max_m", 12.0);
    declare_parameter<double>("obstacle_edge_margin_m", 0.8);
    declare_parameter<double>("obstacle_lookahead_m", 20.0);
    declare_parameter<int>("obstacle_confirm_frames", 2);
    declare_parameter<double>("obstacle_speed_mps", 3.0);
    declare_parameter<int>("blocked_confirm_frames", 3);
    // Big orange cones mark the start/finish line and legitimately sit near
    // the corridor edge — never treat them as things to dodge.
    declare_parameter<bool>("obstacle_ignore_big_orange", true);
    // A blue or yellow cone IS the track edge, by definition (FSG/FB D2.4) —
    // it can never be an object to drive around. Only something that is not a
    // boundary marker can be: debris, an unknown object, a knocked-over cone
    // that lost its colour vote.
    //
    // Without this, a sparse or noisy map wanders the centerline far enough
    // that genuine boundary cones fall inside the corridor gate, get flagged,
    // and are then REMOVED from the boundary set — which thins the map
    // further and collapses the path entirely. Seen live in FSDS: "obstacle in
    // corridor: cone 7 at (7.8, 0.8)" was a real track cone, and the run died
    // seconds later with no path.
    declare_parameter<bool>("obstacle_require_non_track_colour", true);
    // --- speed regimes (the cap published to Block 6)
    declare_parameter<double>("v_explore_mps", 6.0);
    declare_parameter<double>("v_race_mps", 11.0);
    // How long a path may go missing before it is an emergency. The old 1.0 s
    // was tight enough that a single cycle of thin geometry — normal while the
    // map is still short — latched the EBS mid-run. The held path stays
    // geometrically valid for a few metres, so 2 s at exploration speed is
    // about 5 m of travel on a stale but correct line.
    declare_parameter<double>("path_error_timeout_s", 2.0);
    declare_parameter<int>("min_cone_observations", 1);
    // Midpoint chain search (see order_chain).
    declare_parameter<double>("chain_start_radius_m", 12.0);
    declare_parameter<int>("chain_max_depth", 40);
    declare_parameter<int>("chain_beam", 8);
    declare_parameter<double>("chain_max_turn_deg", 70.0);
    declare_parameter<double>("chain_turn_weight", 6.0);
    // Max gap between consecutive cones on one track edge. FS cone pitch is
    // ~4 m on straights and wider on the outside of a corner, so this must
    // exceed it or a boundary breaks at every corner.
    declare_parameter<double>("boundary_link_m", 7.0);
    declare_parameter<double>("local_detection_timeout_s", 0.35);
    declare_parameter<double>("local_detection_max_sigma_m", 1.5);
    declare_parameter<int>("local_detection_min_track_cones", 3);
    declare_parameter<double>("local_cache_timeout_s", 1.5);
    declare_parameter<double>("local_cache_merge_m", 0.75);
    // Above this speed, losing the path is an emergency; at or below it, the
    // car is still bootstrapping and no path is expected yet. It must sit
    // ABOVE Block 6's creep speed (1.5 m/s): creep-start deliberately drives
    // the car forward with no path at all, to bring the first cones into view
    // from a start box where the opposite-colour gate is still out of pairing
    // range. Treating that as an emergency latches the EBS a few seconds into
    // every run, before the car has seen the track. Blind running stays
    // covered by the supervisor, which EBSes on >0.5 m/s with zero mapped
    // cones for 3 s.
    declare_parameter<double>("stationary_speed_mps", 2.0);

    ahead_ = get_parameter("window_ahead_m").as_double();
    behind_ = get_parameter("window_behind_m").as_double();
    edge_min_ = get_parameter("edge_min_m").as_double();
    edge_max_ = get_parameter("edge_max_m").as_double();
    spacing_ = get_parameter("sample_spacing_m").as_double();
    track_w_ = get_parameter("default_track_width_m").as_double();
    chain_passes_ = get_parameter("chain_smooth_passes").as_int();
    chain_weight_ = get_parameter("chain_smooth_weight").as_double();
    race_enable_ = get_parameter("raceline_enable").as_bool();
    known_ahead_ = get_parameter("known_window_ahead_m").as_double();
    known_behind_ = get_parameter("known_window_behind_m").as_double();
    race_margin_ = get_parameter("raceline_margin_m").as_double();
    race_iters_ = get_parameter("raceline_iters").as_int();
    race_step_ = get_parameter("raceline_step").as_double();
    race_reg_ = get_parameter("raceline_reg").as_double();
    race_min_clear_ = get_parameter("raceline_min_clearance_m").as_double();
    rebuild_delta_ = get_parameter("global_rebuild_cone_delta").as_int();
    rebuild_period_ = get_parameter("global_rebuild_min_period_s").as_double();
    obs_gate_ = get_parameter("obstacle_gate_m").as_double();
    obs_clear_ = get_parameter("obstacle_clearance_m").as_double();
    obs_min_clear_ = get_parameter("obstacle_min_clearance_m").as_double();
    obs_blend_ = get_parameter("obstacle_blend_m").as_double();
    obs_blend_t_ = get_parameter("obstacle_blend_time_s").as_double();
    obs_blend_max_ = get_parameter("obstacle_blend_max_m").as_double();
    obs_edge_margin_ = get_parameter("obstacle_edge_margin_m").as_double();
    obs_lookahead_ = get_parameter("obstacle_lookahead_m").as_double();
    obs_confirm_ = get_parameter("obstacle_confirm_frames").as_int();
    obs_speed_ = get_parameter("obstacle_speed_mps").as_double();
    blocked_confirm_ = get_parameter("blocked_confirm_frames").as_int();
    obs_ignore_big_ = get_parameter("obstacle_ignore_big_orange").as_bool();
    obs_non_track_only_ =
      get_parameter("obstacle_require_non_track_colour").as_bool();
    v_explore_ = get_parameter("v_explore_mps").as_double();
    v_race_ = get_parameter("v_race_mps").as_double();
    stationary_speed_ = get_parameter("stationary_speed_mps").as_double();
    path_error_timeout_ = get_parameter("path_error_timeout_s").as_double();
    min_obs_ = static_cast<uint16_t>(get_parameter("min_cone_observations").as_int());
    chain_start_radius_ = get_parameter("chain_start_radius_m").as_double();
    chain_max_depth_ = get_parameter("chain_max_depth").as_int();
    chain_beam_ = get_parameter("chain_beam").as_int();
    chain_min_cos_ =
      std::cos(get_parameter("chain_max_turn_deg").as_double() * M_PI / 180.0);
    chain_turn_weight_ = get_parameter("chain_turn_weight").as_double();
    boundary_link_m_ = get_parameter("boundary_link_m").as_double();
    local_timeout_ = get_parameter("local_detection_timeout_s").as_double();
    local_max_sigma_ = get_parameter("local_detection_max_sigma_m").as_double();
    local_min_cones_ = get_parameter("local_detection_min_track_cones").as_int();
    local_cache_timeout_ = get_parameter("local_cache_timeout_s").as_double();
    local_cache_merge_ = get_parameter("local_cache_merge_m").as_double();
    cap_ = {v_explore_, SpeedLimit::REASON_EXPLORE, "startup"};

    map_sub_ = create_subscription<fsd_msgs::msg::ConeMap>(
      "/mapping/track", fsd::qos_reliable(5),
      [this](fsd_msgs::msg::ConeMap::ConstSharedPtr m) { map_ = m; });
    local_sub_ = create_subscription<fsd_msgs::msg::Cone3DArray>(
      "/perception/cones", fsd::qos_reliable(5),
      [this](fsd_msgs::msg::Cone3DArray::ConstSharedPtr m) { on_local_cones(m); });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odometry/filtered", fsd::qos_reliable(10),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {
        pose_ = fsd::Pose2D{
          m->pose.pose.position.x, m->pose.pose.position.y,
          fsd::yaw_from_quaternion(m->pose.pose.orientation)};
        speed_ = std::abs(m->twist.twist.linear.x);
      });
    status_sub_ = create_subscription<TrackStatus>(
      "/mapping/status", fsd::qos_reliable(5),
      [this](TrackStatus::ConstSharedPtr m) { mode_ = m->mode; });
    pub_ = create_publisher<PathPointArray>("/planning/path", fsd::qos_reliable(5));
    cap_pub_ = create_publisher<SpeedLimit>("/planning/speed_limit",
                                            fsd::qos_reliable(5));
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / get_parameter("publish_rate_hz").as_double()),
      [this]() { plan(); });
    hb_ = std::make_unique<fsd::HeartbeatEmitter>(this, "path_planning");
  }

private:
  struct Cap
  {
    double v;
    uint8_t reason;
    std::string detail;
  };

  struct GlobalLine
  {
    std::vector<Sample> race, centre;
    std::vector<double> widths;
    double total{0.0};
  };

  struct Obs
  {
    uint32_t id;
    double x, y, station, lateral;
  };

  struct LocalCone
  {
    uint8_t color;
    double x, y, sigma, seen_t;
  };

  void on_local_cones(fsd_msgs::msg::Cone3DArray::ConstSharedPtr msg)
  {
    local_cones_ = msg;
    local_t_ = now().seconds();
    if (!pose_) {
      return;
    }
    const double cy = std::cos(pose_->yaw), sy = std::sin(pose_->yaw);
    for (const auto & c : msg->cones) {
      const bool track = c.color == ConeDetection2D::COLOR_BLUE ||
                         c.color == ConeDetection2D::COLOR_YELLOW;
      if (!track || c.x <= 0.25 || c.depth_sigma > local_max_sigma_) {
        continue;
      }
      const double wx = pose_->x + cy * c.x - sy * c.y;
      const double wy = pose_->y + sy * c.x + cy * c.y;
      LocalCone * match = nullptr;
      double best = local_cache_merge_;
      for (auto & old : local_cache_) {
        if (old.color != c.color) {
          continue;
        }
        const double d = std::hypot(old.x - wx, old.y - wy);
        if (d < best) {
          best = d;
          match = &old;
        }
      }
      if (match) {
        // Prefer the close-range observation: monocular depth uncertainty
        // drops quadratically as the car approaches a cone.
        if (c.depth_sigma <= match->sigma) {
          match->x = wx;
          match->y = wy;
          match->sigma = c.depth_sigma;
        }
        match->seen_t = local_t_;
      } else {
        local_cache_.push_back({c.color, wx, wy, c.depth_sigma, local_t_});
      }
    }
  }

  // True if the two cones bound OPPOSITE sides of the track.
  //
  // Colour is definitive when both cones have a track colour: two blue cones
  // are the same edge no matter what the side votes say. The side fallback
  // exists only for cones whose colour cannot decide it (orange, unknown).
  //
  // Getting this wrong is expensive and was: the fallback used to be reached
  // for same-colour pairs too, so two duplicate landmarks of ONE blue cone
  // whose side votes had split (which happens whenever a cone passes nearly
  // straight ahead and measurement noise flips the sign of its lateral
  // coordinate) produced a midpoint sitting on the track boundary. That
  // dragged the centerline into the cones, which then measured as objects
  // inside the corridor, which removed them from the boundary set and tore
  // holes in it. Symptom seen in the sim: real boundary cones reported "0.02 m
  // off the centerline" and the global loop refusing to close.
  static bool opposite(const ConeMapEntry & a, const ConeMapEntry & b)
  {
    const bool a_track = a.color == ConeDetection2D::COLOR_BLUE ||
                         a.color == ConeDetection2D::COLOR_YELLOW;
    const bool b_track = b.color == ConeDetection2D::COLOR_BLUE ||
                         b.color == ConeDetection2D::COLOR_YELLOW;
    if (a_track && b_track) {
      return a.color != b.color;
    }
    if (a.side != ConeMapEntry::SIDE_UNKNOWN &&
        b.side != ConeMapEntry::SIDE_UNKNOWN)
    {
      return a.side != b.side;
    }
    return false;
  }

  void plan()
  {
    const double t = now().seconds();
    auto path = compute();
    if (path) {
      last_valid_ = *path;
      last_valid_t_ = t;
      hb_->set_status(Heartbeat::STATUS_OK);
      pub_->publish(*path);
      publish_cap();
      return;
    }
    // ERROR latches the EBS, so it is reserved for losing a path the car is
    // relying on RIGHT NOW — i.e. while moving. A stationary car without a
    // path is a waiting state, not an emergency, and it is the normal state at
    // the start line: FSDS parks the car among the big orange start markers
    // with the first blue/yellow gate still out of pairing range, so the
    // planner legitimately flickers in and out of validity until the car
    // creeps far enough forward to see both edges. Latching the EBS there
    // ends the run before it begins. The supervisor's moving-blind check
    // still covers the dangerous case.
    if (last_valid_t_ < 0.0 || speed_ < stationary_speed_) {
      hb_->set_status(Heartbeat::STATUS_DEGRADED,
                      last_valid_t_ < 0.0 ? "waiting for first valid path"
                                          : "no path yet, below committed speed");
      PathPointArray empty;
      empty.header.stamp = now();
      empty.header.frame_id = "odom";
      pub_->publish(empty);
    } else if (t - last_valid_t_ <= path_error_timeout_) {
      hb_->set_status(Heartbeat::STATUS_DEGRADED, "no fresh path, holding last valid");
      pub_->publish(last_valid_);
    } else {
      hb_->set_status(Heartbeat::STATUS_ERROR, "no valid path for > 1 s");
      PathPointArray empty;
      empty.header.stamp = now();
      empty.header.frame_id = "odom";
      pub_->publish(empty);
    }
    publish_cap();
  }

  void publish_cap()
  {
    SpeedLimit m;
    m.header.stamp = now();
    m.header.frame_id = "odom";
    m.v_max_mps = static_cast<float>(cap_.v);
    m.reason = cap_.reason;
    m.detail = cap_.detail;
    cap_pub_->publish(m);
  }

  std::optional<PathPointArray> compute()
  {
    if (!pose_) {
      return std::nullopt;
    }
    // Confirmed obstacles are not track boundary. Nor are landmarks that have
    // only been seen once or twice: with one camera, a cone first seen at 15 m
    // carries metres of range error (sigma grows as z^2), and midpoints built
    // from those scatter far enough that no forward chain can be linked
    // through them — seen live as "midpoints found (9) but no forward chain".
    // Those landmarks stay in the map and sharpen as the car closes on them;
    // they just do not get a vote on geometry until they have.
    std::vector<ConeMapEntry> boundary;
    const double t = now().seconds();
    local_cache_.erase(
      std::remove_if(local_cache_.begin(), local_cache_.end(),
        [this, t](const LocalCone & c) { return t - c.seen_t > local_cache_timeout_; }),
      local_cache_.end());
    const bool local_fresh = local_cones_ && t - local_t_ <= local_timeout_;
    if (local_fresh || !local_cache_.empty()) {
      uint32_t id = 0x80000000u;
      for (const auto & c : local_cache_) {
        ConeMapEntry e;
        e.id = id++;
        e.color = c.color;
        e.x = static_cast<float>(c.x);
        e.y = static_cast<float>(c.y);
        const double dx = c.x - pose_->x, dy = c.y - pose_->y;
        const double lateral = -dx * std::sin(pose_->yaw) + dy * std::cos(pose_->yaw);
        e.side = lateral >= 0.0 ? ConeMapEntry::SIDE_LEFT : ConeMapEntry::SIDE_RIGHT;
        e.observation_count = 100;
        boundary.push_back(e);
      }
    }
    // Keep current-frame geometry coherent. Only use the persistent map when
    // the current frame cannot form a local corridor; never merge both sets.
    if (static_cast<int>(boundary.size()) < local_min_cones_ && map_) {
      boundary.clear();
      for (const auto & c : map_->cones) {
        if (!obstacle_ids_.count(c.id) && c.observation_count >= min_obs_) {
          boundary.push_back(c);
        }
      }
    }

    std::vector<Sample> samples, centre;
    std::vector<double> widths;
    bool race_source = false;
    if (race_enable_ && mode_ == TrackStatus::MODE_KNOWN &&
        known_window(boundary, samples, centre, widths))
    {
      race_source = true;
    }
    if (samples.empty()) {
      if (!local_window(boundary, samples, widths)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                             "no path: %s (map has %zu cones, %zu are flagged "
                             "as objects)", fail_.c_str(),
                             map_ ? map_->cones.size() : 0u,
                             obstacle_ids_.size());
        return std::nullopt;
      }
      centre = samples;                  // the local path IS the centerline
    }

    if (map_) {
      samples = avoid(samples, centre, widths, race_source);
    }

    PathPointArray msg;
    msg.header.stamp = now();
    msg.header.frame_id = "odom";
    for (size_t i = 0; i < samples.size(); ++i) {
      fsd_msgs::msg::PathPoint p;
      p.x = static_cast<float>(samples[i].x);
      p.y = static_cast<float>(samples[i].y);
      p.heading = static_cast<float>(samples[i].heading);
      p.curvature = static_cast<float>(samples[i].curvature);
      const double w = i < widths.size() ? widths[i] : track_w_;
      p.track_width = static_cast<float>(w > 0.0 ? w : track_w_);
      msg.points.push_back(p);
    }
    return msg;
  }

  // ---------------------------------------------------- EXPLORE regime
  // Records WHY a cycle produced no path, so a failure in the field is one log
  // line instead of a guessing game.
  // Chain one colour into a boundary polyline: start from the cone nearest the
  // car that is ahead of it, then repeatedly take the nearest unused cone that
  // keeps going forward. One colour at a time, so a missing cone on the far
  // side cannot break this side.
  std::vector<P2> chain_boundary(const std::vector<P2> & cones) const
  {
    const int n = static_cast<int>(cones.size());
    if (n == 0) {
      return {};
    }
    const P2 car{pose_->x, pose_->y};
    P2 dir{std::cos(pose_->yaw), std::sin(pose_->yaw)};

    int start = -1;
    double best = 1e18;
    for (int i = 0; i < n; ++i) {
      const double sx = cones[i].x - car.x, sy = cones[i].y - car.y;
      const double d = std::hypot(sx, sy);
      if (d < 1e-6 || (sx * dir.x + sy * dir.y) / d < -0.3) {
        continue;                          // behind the car
      }
      if (d < best) {
        best = d;
        start = i;
      }
    }
    if (start < 0) {
      return {};
    }
    std::vector<bool> used(n, false);
    std::vector<P2> chain{cones[start]};
    used[start] = true;
    for (int step = 0; step < n; ++step) {
      int pick = -1;
      double pd = boundary_link_m_;
      for (int i = 0; i < n; ++i) {
        if (used[i]) {
          continue;
        }
        const double sx = cones[i].x - chain.back().x;
        const double sy = cones[i].y - chain.back().y;
        const double d = std::hypot(sx, sy);
        if (d < 1e-6 || d > boundary_link_m_) {
          continue;
        }
        if ((sx * dir.x + sy * dir.y) / d < 0.0) {
          continue;                        // must keep going forward
        }
        if (d < pd) {
          pd = d;
          pick = i;
        }
      }
      if (pick < 0) {
        break;
      }
      const double sx = cones[pick].x - chain.back().x;
      const double sy = cones[pick].y - chain.back().y;
      const double d = std::hypot(sx, sy);
      dir = {sx / d, sy / d};
      used[pick] = true;
      chain.push_back(cones[pick]);
    }
    return chain;
  }

  // Centerline from the track BOUNDARIES rather than from cone pairs.
  //
  // The Delaunay-midpoint route needs a blue and a yellow cone 1.5-6 m apart to
  // produce even one point. With a single camera the map is routinely lopsided
  // — four blue and one yellow through a corner — and that route then yields
  // nothing at all, which is what left the car driving blind in FSDS. Tracking
  // each colour as its own boundary still works with one side missing: offset
  // the side we do have by half a track width. This is the approach FaSTTUBe's
  // planner is built around, and it is why theirs survives corners.
  bool boundary_centerline(const std::vector<ConeMapEntry> & cones,
                           std::vector<Sample> & out,
                           std::vector<double> & widths)
  {
    const double cos_y = std::cos(pose_->yaw), sin_y = std::sin(pose_->yaw);
    std::vector<P2> blue, yellow;
    for (const auto & c : cones) {
      const double dx = c.x - pose_->x, dy = c.y - pose_->y;
      const double lx = dx * cos_y + dy * sin_y;
      const double ly = -dx * sin_y + dy * cos_y;
      if (lx < -behind_ || lx > ahead_ || std::abs(ly) > 12.0) {
        continue;
      }
      if (c.color == ConeDetection2D::COLOR_BLUE) {
        blue.push_back({static_cast<double>(c.x), static_cast<double>(c.y)});
      } else if (c.color == ConeDetection2D::COLOR_YELLOW) {
        yellow.push_back({static_cast<double>(c.x), static_cast<double>(c.y)});
      }
    }
    const auto left = chain_boundary(blue);     // blue bounds the left edge
    const auto right = chain_boundary(yellow);  // yellow bounds the right edge
    const double half = track_w_ / 2.0;

    std::vector<P2> centre;
    auto offset_from = [&](const std::vector<P2> & b, double sign) {
      // sign +1: boundary is on the LEFT, centre lies to its right.
      for (size_t i = 0; i + 1 < b.size(); ++i) {
        const double sx = b[i + 1].x - b[i].x, sy = b[i + 1].y - b[i].y;
        const double d = std::hypot(sx, sy);
        if (d < 1e-6) {
          continue;
        }
        const P2 nrm{-sy / d, sx / d};       // left normal of travel
        centre.push_back({b[i].x - sign * half * nrm.x,
                          b[i].y - sign * half * nrm.y});
      }
      if (b.size() >= 2) {
        const size_t i = b.size() - 2;
        const double sx = b[i + 1].x - b[i].x, sy = b[i + 1].y - b[i].y;
        const double d = std::hypot(sx, sy);
        const P2 nrm{-sy / d, sx / d};
        centre.push_back({b.back().x - sign * half * nrm.x,
                          b.back().y - sign * half * nrm.y});
      }
    };

    if (left.size() >= 2 && right.size() >= 2) {
      // Both edges known: walk the longer one and pair by closest approach.
      const auto & lg = left.size() >= right.size() ? left : right;
      const auto & sh = left.size() >= right.size() ? right : left;
      for (const auto & p : lg) {
        double bd = 1e18;
        P2 q = sh.front();
        for (const auto & s : sh) {
          const double d = dist(p, s);
          if (d < bd) {
            bd = d;
            q = s;
          }
        }
        if (bd > 8.0) {
          continue;                          // implausible pairing, skip
        }
        centre.push_back({(p.x + q.x) / 2.0, (p.y + q.y) / 2.0});
        widths.push_back(bd);
      }
    } else if (left.size() >= 2 || right.size() >= 2) {
      // Only one edge visible. Which way to offset is decided by where that
      // edge actually IS relative to the car, not by its colour: a single
      // misclassified cone would otherwise push the car off the track in the
      // wrong direction, and colour is the least reliable thing we measure.
      const auto & only = left.size() >= 2 ? left : right;
      double mean_lat = 0.0;
      for (const auto & p : only) {
        const double dx = p.x - pose_->x, dy = p.y - pose_->y;
        mean_lat += -dx * sin_y + dy * cos_y;
      }
      mean_lat /= static_cast<double>(only.size());
      offset_from(only, mean_lat >= 0.0 ? +1.0 : -1.0);
    } else {
      fail_ = "no boundary: " + std::to_string(blue.size()) + " blue, " +
              std::to_string(yellow.size()) + " yellow in window";
      return false;
    }
    if (centre.size() < 2) {
      fail_ = "boundary gave " + std::to_string(centre.size()) + " centre points";
      return false;
    }

    const auto smoothed = smooth_chain(centre, chain_passes_, chain_weight_, false);
    out = catmull_rom(smoothed, spacing_);
    if (out.size() < 3) {
      fail_ = "boundary centreline too short to spline";
      out.clear();
      return false;
    }
    if (widths.size() != out.size()) {
      const double w = widths.empty()
        ? track_w_
        : std::accumulate(widths.begin(), widths.end(), 0.0) / widths.size();
      widths.assign(out.size(), w > 0.0 ? w : track_w_);
    }
    return true;
  }

  bool local_window(const std::vector<ConeMapEntry> & all,
                    std::vector<Sample> & out, std::vector<double> & widths)
  {
    fail_ = "";
    // Boundary tracking first: it survives a lopsided map, which is the normal
    // case with one camera. Delaunay midpoints stay as the fallback for when
    // colours are unreliable but the pairing geometry is good.
    if (boundary_centerline(all, out, widths)) {
      return true;
    }
    out.clear();
    widths.clear();
    const double cos_y = std::cos(pose_->yaw);
    const double sin_y = std::sin(pose_->yaw);
    std::vector<ConeMapEntry> cones;
    for (const auto & c : all) {
      const double dx = c.x - pose_->x;
      const double dy = c.y - pose_->y;
      const double lx = dx * cos_y + dy * sin_y;
      const double ly = -dx * sin_y + dy * cos_y;
      if (lx >= -behind_ && lx <= ahead_ && std::abs(ly) <= 15.0) {
        cones.push_back(c);
      }
    }
    std::vector<P2> kept;
    std::vector<double> kept_w;
    if (!midpoints(cones, kept, kept_w)) {
      fail_ = "no opposite-colour cone pairs in the " +
              std::to_string(cones.size()) + "-cone window";
      return false;
    }
    const auto chain = order_chain(kept);
    if (chain.size() < 2) {
      fail_ = "midpoints found (" + std::to_string(kept.size()) +
              ") but no forward chain through them";
      return false;
    }
    std::vector<P2> ordered;
    double w_sum = 0.0;
    for (int idx : chain) {
      ordered.push_back(kept[idx]);
      w_sum += kept_w[idx];
    }
    const double w_mean = w_sum / chain.size();
    // Same jitter cure as the global loop: without it the local path carries
    // phantom 2 m-radius corners the velocity profile brakes for.
    ordered = smooth_chain(ordered, chain_passes_, chain_weight_, false);
    out = catmull_rom(ordered, spacing_);
    if (out.size() < 3) {
      fail_ = "chain of " + std::to_string(chain.size()) +
              " midpoints was too short to spline";
      out.clear();
      return false;
    }
    widths.assign(out.size(), w_mean > 0.0 ? w_mean : track_w_);
    return true;
  }

  bool midpoints(const std::vector<ConeMapEntry> & cones,
                 std::vector<P2> & kept, std::vector<double> & kept_w) const
  {
    kept.clear();
    kept_w.clear();
    if (cones.size() < 3) {              // Delaunay needs >=3 points
      return false;
    }
    std::vector<P2> pts;
    pts.reserve(cones.size());
    for (const auto & c : cones) {
      pts.push_back({static_cast<double>(c.x), static_cast<double>(c.y)});
    }
    const auto tris = delaunay(pts);
    if (tris.empty()) {
      return false;
    }
    std::map<std::pair<int, int>, bool> seen;
    std::vector<P2> mids;
    std::vector<double> ws;
    for (const auto & t : tris) {
      const std::pair<int, int> es[3] = {
        {std::min(t.a, t.b), std::max(t.a, t.b)},
        {std::min(t.b, t.c), std::max(t.b, t.c)},
        {std::min(t.c, t.a), std::max(t.c, t.a)}};
      for (const auto & e : es) {
        if (seen[e]) {
          continue;
        }
        seen[e] = true;
        if (!opposite(cones[e.first], cones[e.second])) {
          continue;
        }
        const double len = dist(pts[e.first], pts[e.second]);
        if (len < edge_min_ || len > edge_max_) {
          continue;
        }
        mids.push_back({(pts[e.first].x + pts[e.second].x) / 2.0,
                        (pts[e.first].y + pts[e.second].y) / 2.0});
        ws.push_back(len);
      }
    }
    // Dedupe near-identical midpoints (shared cones).
    for (size_t i = 0; i < mids.size(); ++i) {
      bool dup = false;
      for (const auto & k : kept) {
        if (dist(mids[i], k) <= 0.3) {
          dup = true;
          break;
        }
      }
      if (!dup) {
        kept.push_back(mids[i]);
        kept_w.push_back(ws[i]);
      }
    }
    return kept.size() >= 2;             // 2 midpoints = a short usable segment
  }

  // Ordering the midpoints into a driveable chain, by BEAM TREE SEARCH.
  //
  // This used to be a greedy nearest-neighbour walk: take the closest unused
  // midpoint ahead, commit, repeat. Greedy commits to its first choice, so one
  // bad step — a midpoint slightly off to the side, a gap where a cone was not
  // yet mapped — dead-ends the whole chain. Live in FSDS that showed up as
  // "midpoints found (9) but no forward chain through them" while the car had
  // plenty of cones: the geometry was there, the walk just could not find it.
  //
  // The teams that win this competition do not use greedy here. Urinay (BCN
  // eMotorsport) and FaSTTUBe both run a height-limited search over midpoint
  // candidates scored by a heuristic and keep the best BRANCH, which is what
  // makes them robust to exactly this — gaps, scatter, and one side of the
  // track not being visible through a corner.
  //
  // Beam search: expand every surviving branch by one step per round, keep the
  // `beam` cheapest, stop at `max_depth`. Cost per step prefers even spacing
  // and small heading changes, so a smooth chain outranks a longer ragged one.
  std::vector<int> order_chain(const std::vector<P2> & pts) const
  {
    const int n = static_cast<int>(pts.size());
    if (n == 0) {
      return {};
    }
    const P2 car{pose_->x, pose_->y};
    const P2 heading{std::cos(pose_->yaw), std::sin(pose_->yaw)};

    struct Node
    {
      int idx, parent, depth;
      double cost;
      P2 dir;
    };
    std::vector<Node> tree;

    // Seed with every midpoint ahead of the car, not just the nearest one:
    // the nearest is sometimes the one that leads nowhere.
    for (int i = 0; i < n; ++i) {
      const double sx = pts[i].x - car.x;
      const double sy = pts[i].y - car.y;
      const double d = std::hypot(sx, sy);
      if (d < 1e-6 || d > chain_start_radius_) {
        continue;
      }
      if ((sx * heading.x + sy * heading.y) / d < 0.0) {
        continue;                          // strictly ahead of the car
      }
      tree.push_back({i, -1, 1, d, {sx / d, sy / d}});
    }
    if (tree.empty()) {
      return {};
    }

    std::vector<int> frontier(tree.size());
    for (size_t i = 0; i < tree.size(); ++i) {
      frontier[i] = static_cast<int>(i);
    }
    int best_node = frontier[0];

    for (int depth = 1; depth < chain_max_depth_ && !frontier.empty(); ++depth) {
      std::vector<int> next;
      for (int ni : frontier) {
        const Node cur = tree[ni];
        for (int i = 0; i < n; ++i) {
          const double sx = pts[i].x - pts[cur.idx].x;
          const double sy = pts[i].y - pts[cur.idx].y;
          const double d = std::hypot(sx, sy);
          if (d < 1e-6 || d > edge_max_) {
            continue;
          }
          const P2 step{sx / d, sy / d};
          const double turn = step.x * cur.dir.x + step.y * cur.dir.y;
          if (turn < chain_min_cos_) {
            continue;                      // too sharp to be the same track
          }
          // Already on this branch? (loops are the other greedy failure)
          bool seen = false;
          for (int p = ni; p != -1; p = tree[p].parent) {
            if (tree[p].idx == i) {
              seen = true;
              break;
            }
          }
          if (seen) {
            continue;
          }
          const double step_cost = d + chain_turn_weight_ * (1.0 - turn);
          tree.push_back({i, ni, cur.depth + 1, cur.cost + step_cost, step});
          next.push_back(static_cast<int>(tree.size()) - 1);
        }
      }
      // Keep the cheapest `beam` branches; cost grows with length, so compare
      // on cost per step to avoid always preferring the shortest branch.
      std::sort(next.begin(), next.end(), [&](int a, int b) {
        return tree[a].cost / tree[a].depth < tree[b].cost / tree[b].depth;
      });
      if (static_cast<int>(next.size()) > chain_beam_) {
        next.resize(chain_beam_);
      }
      for (int ni : next) {
        if (tree[ni].depth > tree[best_node].depth ||
            (tree[ni].depth == tree[best_node].depth &&
             tree[ni].cost < tree[best_node].cost))
        {
          best_node = ni;
        }
      }
      frontier.swap(next);
    }

    std::vector<int> chain;
    for (int p = best_node; p != -1; p = tree[p].parent) {
      chain.push_back(tree[p].idx);
    }
    std::reverse(chain.begin(), chain.end());
    return chain;
  }

  // ------------------------------------------------------ KNOWN regime
  bool known_window(const std::vector<ConeMapEntry> & boundary,
                    std::vector<Sample> & race_out,
                    std::vector<Sample> & centre_out,
                    std::vector<double> & width_out)
  {
    const double t = now().seconds();
    const bool stale =
      !global_ || obstacle_ids_ != global_obstacles_ ||
      static_cast<int>(boundary.size()) - global_cones_ >= rebuild_delta_;
    if (stale && (!global_ || global_t_ < 0.0 ||
                  t - global_t_ >= rebuild_period_))
    {
      auto built = build_global(boundary);
      global_t_ = t;
      if (built) {
        global_ = std::move(*built);
        global_cones_ = static_cast<int>(boundary.size());
        global_obstacles_ = obstacle_ids_;
        global_failed_logged_ = false;
        RCLCPP_INFO(get_logger(),
                    "global racing line: %zu pts, %.1f m lap, from %zu cones",
                    global_->race.size(), global_->total, boundary.size());
      } else if (!global_failed_logged_) {
        global_failed_logged_ = true;
        RCLCPP_WARN(get_logger(),
                    "known-track mode: could not close a global loop from the "
                    "map — staying on the local window");
      }
    }
    if (!global_) {
      return false;
    }

    const auto & race = global_->race;
    const auto & centre = global_->centre;
    const int n = static_cast<int>(race.size());
    int i_near = 0;
    double best = 1e18;
    for (int i = 0; i < n; ++i) {
      const double d2 = std::pow(race[i].x - pose_->x, 2) +
                        std::pow(race[i].y - pose_->y, 2);
      if (d2 < best) {
        best = d2;
        i_near = i;
      }
    }
    // A few points behind the car so Block 6's nearest-point search has
    // somewhere to sit even if the car is slightly ahead of the slice.
    const int back = static_cast<int>(std::round(known_behind_ / spacing_));
    const int start = ((i_near - back) % n + n) % n;
    race_out.clear();
    centre_out.clear();
    width_out.clear();
    const double limit = known_ahead_ + known_behind_;
    double acc = 0.0;
    for (int step = 0; step < n; ++step) {
      const int idx = (start + step) % n;
      if (step > 0) {
        const int prev = (start + step - 1) % n;
        acc += dist({race[idx].x, race[idx].y}, {race[prev].x, race[prev].y});
        if (acc > limit) {
          break;
        }
      }
      race_out.push_back(race[idx]);
      centre_out.push_back(centre[idx]);
      width_out.push_back(global_->widths[idx]);
    }
    if (race_out.size() < 6) {
      race_out.clear();
      centre_out.clear();
      width_out.clear();
      return false;
    }
    return true;
  }

  std::optional<GlobalLine> build_global(const std::vector<ConeMapEntry> & boundary)
  {
    if (boundary.size() < 12) {
      return std::nullopt;
    }
    std::vector<P2> kept;
    std::vector<double> kept_w;
    if (!midpoints(boundary, kept, kept_w)) {
      return std::nullopt;
    }
    const auto chain = order_chain(kept);
    if (chain.size() < 12) {
      return std::nullopt;
    }
    // A real lap: the walk came back to where it started AND used most of the
    // midpoints. Both matter — a shortcut across a hairpin can close a loop
    // while skipping half the track.
    std::vector<P2> loop;
    std::vector<double> loop_w;
    for (int idx : chain) {
      loop.push_back(kept[idx]);
      loop_w.push_back(kept_w[idx]);
    }
    if (dist(loop.back(), loop.front()) > edge_max_ ||
        chain.size() < 0.75 * kept.size())
    {
      return std::nullopt;
    }

    GlobalLine g;
    const auto ordered = smooth_chain(loop, chain_passes_, chain_weight_, true);
    g.centre = periodic_catmull_rom(ordered, spacing_, loop_w, &g.widths);
    if (g.centre.size() < 12) {
      return std::nullopt;
    }
    for (size_t i = 0; i < g.centre.size(); ++i) {
      const size_t j = (i + 1) % g.centre.size();
      g.total += dist({g.centre[i].x, g.centre[i].y},
                      {g.centre[j].x, g.centre[j].y});
    }

    std::vector<P2> base, normals;
    std::vector<double> bounds;
    for (size_t i = 0; i < g.centre.size(); ++i) {
      base.push_back({g.centre[i].x, g.centre[i].y});
      normals.push_back({-std::sin(g.centre[i].heading),
                         std::cos(g.centre[i].heading)});
      bounds.push_back(std::max(0.0, g.widths[i] / 2.0 - race_margin_));
    }
    const auto alpha = optimize_raceline(base, normals, bounds, race_iters_,
                                         race_step_, race_reg_);
    std::vector<P2> shifted;
    for (size_t i = 0; i < base.size(); ++i) {
      shifted.push_back({base[i].x + alpha[i] * normals[i].x,
                         base[i].y + alpha[i] * normals[i].y});
    }
    // resample=false keeps the racing line index-aligned with its centerline.
    std::vector<double> ignored;
    g.race = periodic_catmull_rom(shifted, spacing_, g.widths, &ignored, false);
    if (g.race.size() != g.centre.size()) {
      return std::nullopt;
    }

    std::vector<P2> cones;
    for (const auto & c : boundary) {
      cones.push_back({static_cast<double>(c.x), static_cast<double>(c.y)});
    }
    // Verify, then trust: if the optimized line does not keep clear of the
    // cones that bounded it, the width estimate was wrong somewhere.
    if (min_clearance(g.race, cones) < race_min_clear_) {
      RCLCPP_WARN(get_logger(), "racing line failed the clearance check — "
                                "using the centerline for this map");
      g.race = g.centre;
    }
    g.race = smooth_curvature(g.race, 2, true);
    return g;
  }

  // ------------------------------------------------- obstacle handling
  std::vector<Sample> avoid(const std::vector<Sample> & samples,
                            const std::vector<Sample> & centre,
                            const std::vector<double> & widths,
                            bool race_source)
  {
    const double base_v = race_source ? v_race_ : v_explore_;
    const uint8_t base_reason = race_source ? SpeedLimit::REASON_RACE
                                            : SpeedLimit::REASON_EXPLORE;
    const std::string base_detail =
      race_source ? "known track, global racing line"
                  : "exploring: capped by sensor horizon";

    std::vector<P2> path_xy, centre_xy;
    for (const auto & s : samples) {
      path_xy.push_back({s.x, s.y});
    }
    for (const auto & s : centre) {
      centre_xy.push_back({s.x, s.y});
    }
    const auto cum_s = path_stations(path_xy);
    const auto centre_s = path_stations(centre_xy);

    std::vector<Obs> obstacles;
    scan_obstacles(centre_xy, centre_s, path_xy, cum_s, obstacles);
    if (obstacles.empty()) {
      blocked_frames_ = 0;
      cap_ = {base_v, base_reason, base_detail};
      return samples;
    }

    const auto car = point_to_path(path_xy, cum_s, pose_->x, pose_->y);
    const double car_s = car ? car->station : 0.0;

    std::vector<std::pair<double, double>> shifts;   // (station, alpha)
    for (const auto & o : obstacles) {
      if (o.station < car_s - 1.0 || o.station > car_s + obs_lookahead_) {
        continue;
      }
      size_t i_w = 0;
      double bw = 1e18;
      for (size_t i = 0; i < cum_s.size(); ++i) {
        if (std::abs(cum_s[i] - o.station) < bw) {
          bw = std::abs(cum_s[i] - o.station);
          i_w = i;
        }
      }
      const double w = (i_w < widths.size() && widths[i_w] > 0.0) ? widths[i_w]
                                                                  : track_w_;
      const double bound = std::max(0.0, w / 2.0 - obs_edge_margin_);
      shifts.emplace_back(o.station,
                          choose_pass(o.lateral, bound, obs_clear_).alpha);
    }
    if (shifts.empty()) {
      blocked_frames_ = 0;
      cap_ = {base_v, base_reason, base_detail};
      return samples;
    }

    const double blend = std::min(obs_blend_max_,
                                  std::max(obs_blend_, obs_blend_t_ * speed_));
    const auto out = apply_offsets(samples, cum_s, shifts, blend);

    // Final gate: measure the clearance actually achieved. Overlapping
    // obstacles can fight each other, and geometry beats intent.
    std::vector<P2> out_xy;
    for (const auto & s : out) {
      out_xy.push_back({s.x, s.y});
    }
    const auto out_s = path_stations(out_xy);
    uint32_t worst_id = 0;
    double worst = std::numeric_limits<double>::infinity();
    for (const auto & o : obstacles) {
      if (o.station < car_s - 1.0 || o.station > car_s + obs_lookahead_) {
        continue;
      }
      const auto hit = point_to_path(out_xy, out_s, o.x, o.y);
      if (hit && hit->distance < worst) {
        worst = hit->distance;
        worst_id = o.id;
      }
    }
    char detail[160];
    if (worst < obs_min_clear_) {
      if (++blocked_frames_ >= blocked_confirm_) {
        std::snprintf(detail, sizeof(detail),
                      "corridor blocked by cone %u: only %.2f m of gap — stopping",
                      worst_id, worst);
        cap_ = {0.0, SpeedLimit::REASON_BLOCKED, detail};
        return out;
      }
    } else {
      blocked_frames_ = 0;
    }
    std::snprintf(detail, sizeof(detail),
                  "avoiding %zu object(s) in the corridor", shifts.size());
    cap_ = {std::min(base_v, obs_speed_), SpeedLimit::REASON_OBSTACLE, detail};
    return out;
  }

  // Membership is decided against the CENTERLINE; the station and lateral
  // offset that place the manoeuvre are measured on the path actually being
  // published. Measuring membership on a line that has already been pushed
  // sideways would flag the boundary cones it was pushed toward, and dropping
  // those from the boundary set tears a hole in the corridor.
  //
  // Flags are sticky once confirmed on obstacle_confirm_frames consecutive
  // cycles: a cone we decided to drive around must not flicker back into the
  // boundary set, which would make the path oscillate.
  void scan_obstacles(const std::vector<P2> & centre_xy,
                      const std::vector<double> & centre_s,
                      const std::vector<P2> & path_xy,
                      const std::vector<double> & cum_s,
                      std::vector<Obs> & found)
  {
    const double reach = obs_lookahead_ + 10.0;
    std::unordered_set<uint32_t> alive;
    for (const auto & c : map_->cones) {
      if (obs_ignore_big_ && c.color == ConeDetection2D::COLOR_ORANGE_BIG) {
        continue;
      }
      // Every colour the rulebook defines is track furniture: blue and yellow
      // are the edges, orange marks start/finish. Only a cone perception could
      // NOT classify is a candidate foreign object.
      if (obs_non_track_only_ && c.color != ConeDetection2D::COLOR_UNKNOWN) {
        continue;
      }
      if (std::hypot(c.x - pose_->x, c.y - pose_->y) > reach) {
        continue;
      }
      const auto ref = point_to_path(centre_xy, centre_s, c.x, c.y);
      if (!ref || ref->distance > obs_gate_) {
        continue;
      }
      alive.insert(c.id);
      if (!obstacle_ids_.count(c.id)) {
        const int seen = ++obstacle_seen_[c.id];
        if (seen < obs_confirm_) {
          continue;
        }
        obstacle_ids_.insert(c.id);
        RCLCPP_WARN(get_logger(),
                    "obstacle in corridor: cone %u at (%.1f, %.1f), %.2f m off "
                    "the centerline", c.id, c.x, c.y, ref->distance);
      }
      const auto hit = point_to_path(path_xy, cum_s, c.x, c.y);
      if (hit) {
        found.push_back({c.id, static_cast<double>(c.x), static_cast<double>(c.y),
                         hit->station, hit->lateral});
      }
    }
    for (auto it = obstacle_seen_.begin(); it != obstacle_seen_.end();) {
      it = alive.count(it->first) ? std::next(it) : obstacle_seen_.erase(it);
    }
  }

  double ahead_, behind_, edge_min_, edge_max_, spacing_, track_w_;
  double chain_weight_, known_ahead_, known_behind_, race_margin_;
  double race_step_, race_reg_, race_min_clear_, rebuild_period_;
  double obs_gate_, obs_clear_, obs_min_clear_, obs_blend_, obs_blend_t_;
  double obs_blend_max_, obs_edge_margin_, obs_lookahead_, obs_speed_;
  double v_explore_, v_race_, stationary_speed_{2.0}, path_error_timeout_{2.0};
  int chain_passes_, race_iters_, rebuild_delta_, obs_confirm_, blocked_confirm_;
  bool race_enable_, obs_ignore_big_, obs_non_track_only_{true};
  std::string fail_;
  uint16_t min_obs_{1};
  double chain_start_radius_{12.0}, chain_min_cos_{0.34}, chain_turn_weight_{6.0};
  int chain_max_depth_{40}, chain_beam_{8};
  double boundary_link_m_{7.0};
  double local_timeout_{0.35}, local_max_sigma_{1.5}, local_t_{-1.0};
  double local_cache_timeout_{1.5}, local_cache_merge_{0.75};
  int local_min_cones_{3};

  fsd_msgs::msg::ConeMap::ConstSharedPtr map_;
  fsd_msgs::msg::Cone3DArray::ConstSharedPtr local_cones_;
  std::vector<LocalCone> local_cache_;
  std::optional<fsd::Pose2D> pose_;
  double speed_{0.0};
  uint8_t mode_{TrackStatus::MODE_EXPLORE};
  PathPointArray last_valid_;
  double last_valid_t_{-1.0};
  Cap cap_;

  std::unordered_set<uint32_t> obstacle_ids_, global_obstacles_;
  std::unordered_map<uint32_t, int> obstacle_seen_;
  int blocked_frames_{0};
  std::optional<GlobalLine> global_;
  int global_cones_{0};
  double global_t_{-1.0};
  bool global_failed_logged_{false};

  rclcpp::Subscription<fsd_msgs::msg::ConeMap>::SharedPtr map_sub_;
  rclcpp::Subscription<fsd_msgs::msg::Cone3DArray>::SharedPtr local_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<TrackStatus>::SharedPtr status_sub_;
  rclcpp::Publisher<PathPointArray>::SharedPtr pub_;
  rclcpp::Publisher<SpeedLimit>::SharedPtr cap_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<fsd::HeartbeatEmitter> hb_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathPlanningNode>());
  rclcpp::shutdown();
  return 0;
}
