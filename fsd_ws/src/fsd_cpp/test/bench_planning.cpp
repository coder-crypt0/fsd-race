#include <chrono>
#include <cstdio>
#include <random>
#include <map>
#include <algorithm>
#include <vector>
#include <cmath>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include <optional>
#include "algo_extract.hpp"
using Clock = std::chrono::steady_clock;
int main(){
  std::mt19937 rng(3);
  std::uniform_real_distribution<double> j(-0.06,0.06);
  for (int n_pairs : {20, 40, 75, 150}) {
    std::vector<P2> pts;
    for(int i=0;i<n_pairs;++i){
      double a=2*M_PI*i/n_pairs;
      pts.push_back({40*std::cos(a)+j(rng),25*std::sin(a)+j(rng)});
      pts.push_back({36.5*std::cos(a)+j(rng),21.5*std::sin(a)+j(rng)});
    }
    // Delaunay benchmark
    auto t0=Clock::now();
    int reps=50;
    size_t tris=0;
    for(int r=0;r<reps;++r) tris=delaunay(pts).size();
    double ms=std::chrono::duration<double,std::milli>(Clock::now()-t0).count()/reps;
    // spline benchmark on half the midpoint count
    std::vector<P2> chain;
    for(int i=0;i<n_pairs;++i){double a=2*M_PI*i/n_pairs;chain.push_back({38.25*std::cos(a),23.25*std::sin(a)});}
    auto t1=Clock::now();
    size_t samp=0;
    for(int r=0;r<reps;++r) samp=catmull_rom(chain,0.5).size();
    double ms2=std::chrono::duration<double,std::milli>(Clock::now()-t1).count()/reps;
    printf("%3zu cones: delaunay %7.3f ms (%zu tris) | spline %6.3f ms (%zu pts)\n",
           pts.size(), ms, tris, ms2, samp);
  }
  return 0;
}
