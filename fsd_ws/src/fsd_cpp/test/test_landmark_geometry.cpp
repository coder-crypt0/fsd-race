#include <cassert>
#include <cmath>
#include <cstdio>
#include <set>
#include "../include/fsd_cpp/landmark_geometry.hpp"
#include "../include/fsd_cpp/speed_profile.hpp"

int main() {
  using namespace fsd_mapping;
  std::vector<Ray> rays;
  for (double x:{0.0,0.5,1.0,1.5}) {
    const double dx=8-x, dy=2, length=std::hypot(dx,dy);
    rays.push_back({x,0,dx/length,dy/length});
  }
  auto t=triangulate(rays);
  assert(t && std::hypot(t->x-8,t->y-2)<1e-8 && t->sigma>=0.1);
  assert(!triangulate({{0,0,1,0},{0.5,0,1,0},{1,0,1,0}}));
  assert(!triangulate({{0,0,-1,0},{0.5,0,-1,0},{1,0,-1,0}}));
  assert(!triangulate({{0,0,0,0},{1,0,1,0},{2,0,1,0}}));
  std::vector<Target> targets{{1,3,2,0},{2,6,2,0},{3,9,2,0},
                               {4,3,-2,1},{5,6,-2,1},{6,9,-2,1}};
  std::vector<Observation> observations;
  for (auto p:targets) { observations.push_back({p.x,p.y,0.1,p.color}); }
  observations.push_back(observations.front());  // One target cannot explain two detections.
  auto matches=correspondences(observations,targets,0,0,0,1.0);
  assert(matches.size()==6);
  std::set<size_t> seen;
  for (auto m:matches) { assert(seen.insert(m.target).second); }
  auto fit=register_frame(observations,targets,0.3,-0.2,0.015);
  assert(fit.valid && std::abs(fit.dx+0.3)<0.03 && std::abs(fit.dy-0.2)<0.03 && fit.rms<0.03);
  const auto locked=register_frame(observations,targets,0.3,-0.2,0.0,1.0,4,0.35,true);
  assert(locked.valid && locked.dyaw==0.0 && locked.rms<0.03);
  for (auto & o:observations) { o.color=255; }
  assert(!register_frame(observations,targets,0,0,0).valid);
  assert(!register_frame({{3,2,0.1,0}},targets,0,0,0).valid);
  std::vector<fsd_planning::SpeedPoint> loop;
  for (int i=0;i<100;++i) {
    double a=i*2*3.141592653589793/100;
    loop.push_back({10*std::cos(a),10*std::sin(a),i==0 ? 1.0 : 0.1});
  }
  auto profile=fsd_planning::lap_profile(loop,10,6,3,6);
  assert(profile.valid && profile.seconds>0 && profile.speed.back()<10);
  for (size_t i=0;i<loop.size();++i) {
    const size_t j=(i+1)%loop.size(); const double ds=std::hypot(loop[i].x-loop[j].x,loop[i].y-loop[j].y);
    assert(profile.speed[j]*profile.speed[j]<=profile.speed[i]*profile.speed[i]+6*ds+1e-8);
    assert(profile.speed[i]*profile.speed[i]<=profile.speed[j]*profile.speed[j]+12*ds+1e-8);
    assert(profile.speed[i]*profile.speed[i]*std::abs(loop[i].curvature)<=6+1e-8);
  }
  assert(!fsd_planning::lap_profile(loop,-1,6,3,6).valid);
  std::puts("PASS: triangulation/parallax, unique color correspondence, pose fit and periodic speed limits");
}
