// Test of the C++ data-association core (Landmark + GridIndex slot map,
// extracted VERBATIM from fsd_cpp/src/cone_mapping_node.cpp) driving the
// exact association loop from the node: state-flip promotion, lazy grid
// deletion, cell-crossing re-insert, periodic compaction.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <optional>
#include <random>
#include <unordered_map>
#include <vector>

// The extracted ICP core deliberately uses the production angle helper. Keep
// the standalone harness dependency-free while exercising identical math.
namespace fsd
{
inline double wrap_angle(double a)
{
  return std::atan2(std::sin(a), std::cos(a));
}
}  // namespace fsd

#include "mapping_extract.hpp"

static constexpr double SEARCH_R = 1.2;
static constexpr double GATE = 0.5;
static constexpr uint32_t N_CONFIRM = 3;

std::vector<Landmark> slots_;
GridIndex grid_confirmed_, grid_tentative_;
uint32_t next_id_ = 1;

std::optional<size_t> best_match(const GridIndex & grid, LmState want,
                                 double x, double y)
{
  std::optional<size_t> best;
  double best_d2 = GATE * GATE;
  for (size_t s : grid.query(slots_, x, y, SEARCH_R)) {
    if (slots_[s].state != want) {
      continue;
    }
    const double dx = slots_[s].x - x;
    const double dy = slots_[s].y - y;
    const double d2 = dx * dx + dy * dy;
    if (d2 < best_d2) {
      best = s;
      best_d2 = d2;
    }
  }
  return best;
}

void kalman_update_slot(size_t s, double ox, double oy, double meas_var,
                        uint8_t color, uint8_t side, double t)
{
  Landmark & lm = slots_[s];
  GridIndex & grid = lm.state == LmState::CONFIRMED ? grid_confirmed_
                                                    : grid_tentative_;
  const int64_t k0 = grid.key_of(lm.x, lm.y);
  lm.update(ox, oy, meas_var, color, side, t);
  if (grid.key_of(lm.x, lm.y) != k0) {
    grid.insert(s, lm.x, lm.y);
  }
}

// Mirrors ConeMappingNode::on_cones per-detection logic exactly.
void associate(double ox, double oy, double meas_var, uint8_t color,
               uint8_t side, double t)
{
  if (auto s = best_match(grid_confirmed_, LmState::CONFIRMED, ox, oy)) {
    kalman_update_slot(*s, ox, oy, meas_var, color, side, t);
    return;
  }
  if (auto s = best_match(grid_tentative_, LmState::TENTATIVE, ox, oy)) {
    Landmark & lm = slots_[*s];
    kalman_update_slot(*s, ox, oy, meas_var, color, side, t);
    if (lm.obs_count >= N_CONFIRM) {
      lm.id = next_id_++;
      lm.state = LmState::CONFIRMED;
      grid_confirmed_.insert(*s, lm.x, lm.y);
    }
  } else {
    slots_.emplace_back(ox, oy, meas_var, color, side, t);
    grid_tentative_.insert(slots_.size() - 1, ox, oy);
  }
}

int main()
{
  std::mt19937 rng(11);
  std::normal_distribution<double> noise(0.0, 0.08);

  // 40 physical cones on a ring, cone spacing ~3 m.
  std::vector<std::array<double, 3>> truth;  // x, y, color
  for (int i = 0; i < 20; ++i) {
    const double a = 2.0 * 3.14159265358979 * i / 20;
    truth.push_back({12.0 * std::cos(a), 12.0 * std::sin(a), 0});
    truth.push_back({8.5 * std::cos(a), 8.5 * std::sin(a), 1});
  }

  // 25 noisy frames, shuffled so promotions interleave with queries, plus
  // pure-noise detections that must die in the tentative buffer, plus a
  // full frame of flipped colors that must be outvoted.
  std::vector<int> order(truth.size());
  for (size_t i = 0; i < order.size(); ++i) order[i] = static_cast<int>(i);
  std::uniform_real_distribution<double> upos(-45.0, 45.0);
  for (int frame = 0; frame < 25; ++frame) {
    std::shuffle(order.begin(), order.end(), rng);
    for (int k : order) {
      const uint8_t color = (frame == 3) ? static_cast<uint8_t>(1 - truth[k][2])
                                         : static_cast<uint8_t>(truth[k][2]);
      associate(truth[k][0] + noise(rng), truth[k][1] + noise(rng),
                0.01, color, 1, frame * 0.033);
    }
    // one random far-away false positive per frame (never re-observed)
    associate(upos(rng) + 100.0, upos(rng) + 100.0, 0.01, 0, 1, frame * 0.033);
    // GC pass (as in the node)
    const double t = frame * 0.033;
    for (auto & lm : slots_) {
      if (lm.state == LmState::TENTATIVE && t - lm.first_seen > 0.3 &&
          lm.obs_count < N_CONFIRM)
      {
        lm.state = LmState::DEAD;
      }
    }
    // periodic compaction (as in the node, just more frequent here)
    if (frame % 7 == 6) {
      grid_confirmed_.compact(slots_, LmState::CONFIRMED);
      grid_tentative_.compact(slots_, LmState::TENTATIVE);
    }
  }

  size_t confirmed = 0, dead = 0, tentative = 0;
  for (const auto & lm : slots_) {
    if (lm.state == LmState::CONFIRMED) ++confirmed;
    else if (lm.state == LmState::DEAD) ++dead;
    else ++tentative;
  }
  std::printf("slots: %zu total | confirmed %zu (expect 40) | dead %zu | "
              "tentative %zu\n", slots_.size(), confirmed, dead, tentative);
  if (confirmed != 40) {
    std::printf("FAIL: duplicate or missing landmarks\n");
    return 1;
  }
  if (dead == 0) {
    std::printf("FAIL: noise detections were never garbage-collected\n");
    return 1;
  }
  double max_err = 0.0;
  int color_wrong = 0;
  for (const auto & lm : slots_) {
    if (lm.state != LmState::CONFIRMED) continue;
    double best = 1e18;
    int best_k = -1;
    for (size_t k = 0; k < truth.size(); ++k) {
      const double d = std::hypot(lm.x - truth[k][0], lm.y - truth[k][1]);
      if (d < best) {
        best = d;
        best_k = static_cast<int>(k);
      }
    }
    max_err = std::max(max_err, best);
    if (lm.best_color() != static_cast<uint8_t>(truth[best_k][2])) {
      ++color_wrong;
    }
  }
  std::printf("max position error: %.3f m, wrong colors after voting: %d\n",
              max_err, color_wrong);
  if (max_err > 0.15 || color_wrong != 0) {
    std::printf("FAIL\n");
    return 1;
  }
  std::printf("\nC++ DATA ASSOCIATION TEST PASSED\n");
  return 0;
}
