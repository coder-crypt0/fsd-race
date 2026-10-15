#pragma once
#include <algorithm>
#include <cmath>
#include <vector>

namespace fsd_planning {
struct SpeedPoint { double x, y, curvature; };
struct LapProfile { std::vector<double> speed; double seconds{0.0}; bool valid{false}; };

// Periodic acceleration/braking envelope. Repeated passes propagate limits
// across the start line, rather than treating it as a stop at an open path end.
inline LapProfile lap_profile(const std::vector<SpeedPoint> & points, double cap,
                              double a_lat, double a_accel, double a_brake)
{
  LapProfile out;
  const size_t n=points.size();
  if (n<4 || !std::isfinite(cap+a_lat+a_accel+a_brake) ||
      cap<=0 || a_lat<=0 || a_accel<=0 || a_brake<=0) { return out; }
  std::vector<double> ds(n);
  out.speed.resize(n);
  for (size_t i=0;i<n;++i) {
    const auto & p=points[i]; const auto & q=points[(i+1)%n];
    if (!std::isfinite(p.x+p.y+p.curvature)) { return LapProfile{}; }
    ds[i]=std::hypot(q.x-p.x,q.y-p.y);
    if (ds[i]<1e-5) { return LapProfile{}; }
    out.speed[i]=std::min(cap,std::sqrt(a_lat/std::max(std::abs(p.curvature),1e-4)));
  }
  // One complete cycle per direction reaches all vertices; a second cycle
  // settles constraints introduced across the seam by the first pass.
  for (int pass=0;pass<3;++pass) {
    for (size_t step=0;step<n;++step) {
      const size_t i=n-1-step,j=(i+1)%n;
      out.speed[i]=std::min(out.speed[i],std::sqrt(out.speed[j]*out.speed[j]+2*a_brake*ds[i]));
    }
    for (size_t i=0;i<n;++i) {
      const size_t j=(i+1)%n;
      out.speed[j]=std::min(out.speed[j],std::sqrt(out.speed[i]*out.speed[i]+2*a_accel*ds[i]));
    }
  }
  for (size_t i=0;i<n;++i) { out.seconds+=2*ds[i]/(out.speed[i]+out.speed[(i+1)%n]); }
  out.valid=std::isfinite(out.seconds);
  return out;
}
}  // namespace fsd_planning
