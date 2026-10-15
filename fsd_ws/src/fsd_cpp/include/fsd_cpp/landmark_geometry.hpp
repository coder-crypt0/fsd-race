#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <set>
#include <vector>

namespace fsd_mapping {

struct Ray {
  double x, y, dx, dy;  // camera origin and unit bearing, in odom
};

struct Triangulation {
  double x, y, sigma, parallax;
};

// Least-squares intersection of successive monocular bearing rays on the
// track plane. A forward-moving camera has a weak baseline for distant cones;
// reject that geometry instead of manufacturing depth from parallel rays.
inline std::optional<Triangulation> triangulate(const std::vector<Ray> & rays,
                                               double min_baseline = 0.4,
                                               double min_parallax = 0.025)
{
  if (rays.size() < 3) { return std::nullopt; }
  double baseline = 0.0, angle = 0.0, a = 0.0, b = 0.0, c = 0.0, ex = 0.0, ey = 0.0;
  for (size_t i = 0; i < rays.size(); ++i) {
    const auto & r = rays[i];
    if (!std::isfinite(r.x + r.y + r.dx + r.dy) ||
        std::abs(std::hypot(r.dx, r.dy) - 1.0) > 1e-3) { return std::nullopt; }
    for (size_t j = 0; j < i; ++j) {
      baseline = std::max(baseline, std::hypot(r.x-rays[j].x, r.y-rays[j].y));
      angle = std::max(angle, std::abs(std::atan2(r.dx*rays[j].dy-r.dy*rays[j].dx,
                                                 r.dx*rays[j].dx+r.dy*rays[j].dy)));
    }
    const double nx = -r.dy, ny = r.dx, d = nx*r.x + ny*r.y;
    a += nx*nx; b += nx*ny; c += ny*ny; ex += nx*d; ey += ny*d;
  }
  const double determinant = a*c-b*b;
  if (baseline < min_baseline || angle < min_parallax ||
      determinant < 1e-5*(a+c)*(a+c)) { return std::nullopt; }
  const double x = (c*ex-b*ey)/determinant, y = (a*ey-b*ex)/determinant;
  double residual2 = 0.0;
  for (const auto & r : rays) {
    const double vx = x-r.x, vy = y-r.y;
    const double along = vx*r.dx + vy*r.dy;
    const double cross = vx*r.dy - vy*r.dx;
    if (along < 0.5 || along > 25.0 || std::abs(std::atan2(cross, along)) > 0.02) {
      return std::nullopt;
    }
    residual2 += cross*cross;
  }
  // Include a pose/bearing noise floor; intersecting rays is not independent
  // survey truth and must not collapse the landmark covariance to zero.
  const double sigma = std::max(0.10, std::sqrt(residual2/rays.size()) /
                                      std::max(std::sin(angle), 0.025));
  if (!std::isfinite(x+y+sigma) || sigma > 0.75) { return std::nullopt; }
  return Triangulation{x, y, sigma, angle};
}

struct Observation {
  double x, y, sigma;
  uint8_t color;
};

struct Target {
  uint32_t id;
  double x, y;
  uint8_t color;
};

struct Match {
  size_t observation, target;
  double distance;
};

inline std::vector<Match> correspondences(const std::vector<Observation> & observations,
                                         const std::vector<Target> & targets,
                                         double px, double py, double yaw,
                                         double gate)
{
  std::vector<Match> edges;
  const double co = std::cos(yaw), si = std::sin(yaw);
  for (size_t i=0; i<observations.size(); ++i) {
    const auto & o = observations[i];
    if (o.color > 1 || !std::isfinite(o.x+o.y+o.sigma) || o.sigma < 0 || o.sigma > 1.2) { continue; }
    const double x = px+co*o.x-si*o.y, y = py+si*o.x+co*o.y;
    for (size_t j=0; j<targets.size(); ++j) {
      if (targets[j].color != o.color) { continue; }
      const double d = std::hypot(x-targets[j].x, y-targets[j].y);
      if (d <= gate) { edges.push_back({i,j,d}); }
    }
  }
  std::sort(edges.begin(), edges.end(), [](const Match & a, const Match & b) { return a.distance < b.distance; });
  std::set<size_t> used_observations, used_targets;
  std::vector<Match> result;
  for (const auto & edge : edges) {
    if (used_observations.count(edge.observation) || used_targets.count(edge.target)) { continue; }
    used_observations.insert(edge.observation); used_targets.insert(edge.target);
    result.push_back(edge);
  }
  return result;
}

struct Registration {
  double dx{0}, dy{0}, dyaw{0}, rms{0};
  std::vector<Match> matches;
  bool valid{false};
};

// Color-constrained, one-to-one registration with residual rejection. The
// target map is read-only: a failed fit cannot move it to fit bad observations.
inline Registration register_frame(const std::vector<Observation> & observations,
                                    const std::vector<Target> & targets,
                                    double px, double py, double yaw,
                                    double gate = 1.0, size_t min_inliers = 4,
                                    double max_rms = 0.35, bool lock_heading = false)
{
  Registration result;
  double wx=px, wy=py, wa=yaw;
  for (int iteration=0; iteration<4; ++iteration) {
    auto matches = correspondences(observations, targets, wx, wy, wa, gate);
    if (matches.size() < min_inliers) { return result; }
    double ox=0, oy=0, tx=0, ty=0, weight_sum=0;
    for (const auto & m : matches) {
      const auto & o=observations[m.observation]; const auto & t=targets[m.target];
      const double w=1.0/std::max(0.04, o.sigma*o.sigma);
      ox+=w*o.x; oy+=w*o.y; tx+=w*t.x; ty+=w*t.y; weight_sum+=w;
    }
    ox/=weight_sum; oy/=weight_sum; tx/=weight_sum; ty/=weight_sum;
    double dot=0, cross=0, spread=0;
    for (const auto & m : matches) {
      const auto & o=observations[m.observation]; const auto & t=targets[m.target];
      const double w=1.0/std::max(0.04, o.sigma*o.sigma);
      const double a=o.x-ox, b=o.y-oy, c=t.x-tx, d=t.y-ty;
      dot+=w*(a*c+b*d); cross+=w*(a*d-b*c); spread+=w*(a*a+b*b);
    }
    if (spread/weight_sum < 2.0) { return result; }
    const double fitted_yaw=std::atan2(cross,dot);
    const double delta=std::atan2(std::sin(fitted_yaw-yaw),std::cos(fitted_yaw-yaw));
    // A real IMU anchors heading. Restrict noisy monocular fits to small yaw corrections.
    if (std::abs(delta)>0.12) { return result; }
    wa=lock_heading ? yaw : yaw+std::clamp(delta,-0.03,0.03);
    wx=tx-std::cos(wa)*ox+std::sin(wa)*oy;
    wy=ty-std::sin(wa)*ox-std::cos(wa)*oy;
    gate=std::min(gate,0.8);
  }
  result.matches=correspondences(observations,targets,wx,wy,wa,gate);
  if (result.matches.size()<min_inliers) { return Registration{}; }
  double rss=0;
  for (const auto & m:result.matches) { rss+=m.distance*m.distance; }
  result.rms=std::sqrt(rss/result.matches.size());
  result.dx=wx-px; result.dy=wy-py;
  result.dyaw=std::atan2(std::sin(wa-yaw),std::cos(wa-yaw));
  result.valid=std::isfinite(result.dx+result.dy+result.dyaw+result.rms) &&
    std::hypot(result.dx,result.dy)<=1.0 && result.rms<=max_rms;
  return result;
}
}  // namespace fsd_mapping
