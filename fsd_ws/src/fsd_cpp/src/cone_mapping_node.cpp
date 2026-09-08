// BLOCK 4 — CONE MAPPING (C++). Normative data-association algorithm:
// confirmed-first matching, tentative buffer with N_CONFIRM promotion,
// tentative-only garbage collection, color/side by majority vote, pose
// interpolated at the detection timestamp.
//
// Data-structure design (v2): a SLOT MAP. All landmarks live in one
// append-only vector (stable indices, entries are never erased — state
// transitions TENTATIVE -> CONFIRMED or TENTATIVE -> DEAD instead). The
// spatial hash grids store slot indices with O(1) incremental insert and
// LAZY deletion: queries filter by state, so stale grid entries are
// skipped instead of invalidating anything. A landmark whose Kalman
// update moves it across a cell border is re-inserted into the new cell
// (the old entry remains as a harmless duplicate — dedup happens naturally
// because both resolve to the same slot). Grids are compacted on a slow
// timer, not per mutation. Result: O(1) amortized insert/update vs the
// previous O(n) rebuild per mutation, and no index-invalidation hazards.
//
// On top of the map, this node owns the two things that make a known track
// usable at speed:
//
//   * MAP-RELATIVE LOCALIZATION — a 2D rigid fit (Kabsch) of the current
//     observations onto the confirmed landmarks, published on
//     /localization/correction. Dead reckoning alone drifts metres per lap,
//     which used to re-map the same physical cone as a new landmark on every
//     lap (measured: 186 landmarks for 70 real cones). Block 3 applies the
//     correction slew-limited so the pose stays continuous.
//   * LAP COUNTING — start-line crossings, published on /mapping/status. One
//     completed lap means the whole track has been seen, which is what
//     licenses race speed downstream.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <unordered_map>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <fsd_msgs/msg/cone3_d_array.hpp>
#include <fsd_msgs/msg/cone_map.hpp>
#include <fsd_msgs/msg/cone_map_entry.hpp>
#include <fsd_msgs/msg/pose_correction.hpp>
#include <fsd_msgs/msg/track_status.hpp>

#include "fsd_cpp/common.hpp"

using fsd_msgs::msg::ConeMapEntry;
using fsd_msgs::msg::PoseCorrection;
using fsd_msgs::msg::TrackStatus;

namespace
{

enum class LmState : uint8_t { TENTATIVE, CONFIRMED, DEAD };

struct Landmark
{
  uint32_t id{0};
  double x, y, var;
  LmState state{LmState::TENTATIVE};
  std::array<uint32_t, 256> color_votes{};   // indexed by color enum
  std::array<uint32_t, 3> side_votes{};      // indexed by side enum
  uint32_t obs_count{1};
  double first_seen, last_seen;

  Landmark(double px, double py, double v, uint8_t color, uint8_t side, double t)
  : x(px), y(py), var(v), first_seen(t), last_seen(t)
  {
    color_votes[color] = 1;
    side_votes[side] = 1;
  }

  void update(double px, double py, double meas_var, uint8_t color,
              uint8_t side, double t)
  {
    const double k = var / (var + meas_var);
    x += k * (px - x);
    y += k * (py - y);
    var = std::max(var * (1.0 - k), 1e-4);
    color_votes[color] += 1;
    side_votes[side] += 1;
    obs_count += 1;
    last_seen = t;
  }

  uint8_t best_color() const
  {
    size_t best = 0;
    for (size_t i = 1; i < color_votes.size(); ++i) {
      if (color_votes[i] > color_votes[best]) {
        best = i;
      }
    }
    return static_cast<uint8_t>(best);
  }

  uint8_t best_side() const
  {
    size_t best = 0;
    for (size_t i = 1; i < side_votes.size(); ++i) {
      if (side_votes[i] > side_votes[best]) {
        best = i;
      }
    }
    return static_cast<uint8_t>(best);
  }
};

// Spatial hash grid over slot indices. Incremental O(1) insert, lazy
// deletion (callers filter candidates by Landmark state), duplicate
// entries allowed and harmless. compact() rebuilds from live slots.
class GridIndex
{
public:
  explicit GridIndex(double cell = 2.0) : cell_(cell) {}

  int64_t key_of(double x, double y) const
  {
    return pack(static_cast<int>(std::floor(x / cell_)),
                static_cast<int>(std::floor(y / cell_)));
  }

  void insert(size_t slot, double x, double y)
  {
    cells_[key_of(x, y)].push_back(slot);
  }

  // Candidate slot indices within radius r of (x, y). May contain
  // duplicates and slots whose state no longer matches — callers filter.
  std::vector<size_t> query(const std::vector<Landmark> & slots,
                            double x, double y, double r) const
  {
    std::vector<size_t> out;
    const int span = static_cast<int>(std::ceil(r / cell_));
    const int kx = static_cast<int>(std::floor(x / cell_));
    const int ky = static_cast<int>(std::floor(y / cell_));
    for (int ix = kx - span; ix <= kx + span; ++ix) {
      for (int iy = ky - span; iy <= ky + span; ++iy) {
        const auto it = cells_.find(pack(ix, iy));
        if (it == cells_.end()) {
          continue;
        }
        for (size_t slot : it->second) {
          const double dx = slots[slot].x - x;
          const double dy = slots[slot].y - y;
          if (dx * dx + dy * dy <= r * r) {
            out.push_back(slot);
          }
        }
      }
    }
    return out;
  }

  // Periodic compaction: rebuild from slots currently in `keep_state`,
  // dropping stale duplicates and dead entries.
  void compact(const std::vector<Landmark> & slots, LmState keep_state)
  {
    cells_.clear();
    for (size_t i = 0; i < slots.size(); ++i) {
      if (slots[i].state == keep_state) {
        insert(i, slots[i].x, slots[i].y);
      }
    }
  }

private:
  static int64_t pack(int ix, int iy)
  {
    return (static_cast<int64_t>(ix) << 32) | static_cast<uint32_t>(iy);
  }

  double cell_;
  std::unordered_map<int64_t, std::vector<size_t>> cells_;
};

// ------------------------------------------------- map-relative localization
struct PoseFix
{
  double dx{0.0}, dy{0.0}, dyaw{0.0};
  uint32_t n_inliers{0};
  double rms{0.0};
  bool valid{false};
};

struct IcpParams
{
  double gate{1.0};
  uint32_t min_inliers{6};
  int iters{3};
  double min_spread_m2{8.0};
  double max_trans_m{0.6};
  double max_rot_rad{0.15};
  double reject_trans_m{2.0};
  double reject_rot_rad{0.4};
};

// Rigid 2D fit of body-frame detections onto the confirmed map.
//
// Iterated closest point with a closed-form Kabsch solve per iteration:
// transform the detections into odom with the working pose, pair each with its
// nearest landmark inside `gate`, solve for the (rotation, translation) that
// best maps observations onto landmarks, move the working pose by it, repeat.
//
// Returns a correction to (px, py, pyaw), or valid=false when the solve is not
// trustworthy. Guards, in order of what they catch:
//   * min_inliers    — too few pairs to constrain 3 DoF
//   * min_spread_m2  — pairs clustered in one spot: rotation ill-determined
//   * reject_*       — solve so large it is more likely a mis-association
//                      (cone spacing is ~3 m) than real drift
//   * max_*          — plausible but big: clamp, converge over more frames
//
// `query` takes (x, y, r) and returns the eligible landmark positions nearby.
template <typename QueryFn>
PoseFix solve_pose_correction(double px, double py, double pyaw,
                              const std::vector<std::pair<double, double>> & dets,
                              QueryFn query, const IcpParams & p)
{
  double wx = px, wy = py, wyaw = pyaw;
  PoseFix fix;

  for (int it = 0; it < p.iters; ++it) {
    const double cos_y = std::cos(wyaw);
    const double sin_y = std::sin(wyaw);
    // (qx, qy, mx, my) pairs: observation in odom, matched landmark.
    std::vector<std::array<double, 4>> pairs;
    for (const auto & [bx, by] : dets) {
      const double qx = wx + bx * cos_y - by * sin_y;
      const double qy = wy + bx * sin_y + by * cos_y;
      bool have = false;
      double best_d2 = p.gate * p.gate, mx = 0.0, my = 0.0;
      for (const auto & [lx, ly] : query(qx, qy, p.gate)) {
        const double d2 = (lx - qx) * (lx - qx) + (ly - qy) * (ly - qy);
        if (d2 < best_d2) {
          best_d2 = d2;
          mx = lx;
          my = ly;
          have = true;
        }
      }
      if (have) {
        pairs.push_back({qx, qy, mx, my});
      }
    }

    const size_t n = pairs.size();
    if (n < p.min_inliers) {
      return fix;                        // valid == false
    }
    double qmx = 0.0, qmy = 0.0, mmx = 0.0, mmy = 0.0;
    for (const auto & pr : pairs) {
      qmx += pr[0];
      qmy += pr[1];
      mmx += pr[2];
      mmy += pr[3];
    }
    qmx /= n;
    qmy /= n;
    mmx /= n;
    mmy /= n;
    double spread = 0.0;
    for (const auto & pr : pairs) {
      spread += (pr[0] - qmx) * (pr[0] - qmx) + (pr[1] - qmy) * (pr[1] - qmy);
    }
    if (spread < p.min_spread_m2) {
      return fix;
    }

    // Kabsch in 2D: the optimal rotation is the argument of the summed
    // cross/dot products of the centred point sets.
    double num = 0.0, den = 0.0;
    for (const auto & pr : pairs) {
      num += (pr[0] - qmx) * (pr[3] - mmy) - (pr[1] - qmy) * (pr[2] - mmx);
      den += (pr[0] - qmx) * (pr[2] - mmx) + (pr[1] - qmy) * (pr[3] - mmy);
    }
    const double dpsi = std::atan2(num, den);
    const double c = std::cos(dpsi), s = std::sin(dpsi);
    const double tx = mmx - (c * qmx - s * qmy);
    const double ty = mmy - (s * qmx + c * qmy);

    double rss = 0.0;
    for (const auto & pr : pairs) {
      const double rx = c * pr[0] - s * pr[1] + tx - pr[2];
      const double ry = s * pr[0] + c * pr[1] + ty - pr[3];
      rss += rx * rx + ry * ry;
    }
    fix.n_inliers = static_cast<uint32_t>(n);
    fix.rms = std::sqrt(rss / n);

    // Move the working pose by the same rigid transform.
    const double nx = c * wx - s * wy + tx;
    const double ny = s * wx + c * wy + ty;
    wx = nx;
    wy = ny;
    wyaw = fsd::wrap_angle(wyaw + dpsi);
  }

  double dx = wx - px, dy = wy - py;
  double dyaw = fsd::wrap_angle(wyaw - pyaw);
  const double trans = std::hypot(dx, dy);
  if (trans > p.reject_trans_m || std::abs(dyaw) > p.reject_rot_rad) {
    fix.valid = false;
    return fix;
  }
  if (trans > p.max_trans_m) {
    const double scale = p.max_trans_m / trans;
    dx *= scale;
    dy *= scale;
  }
  fix.dx = dx;
  fix.dy = dy;
  fix.dyaw = std::clamp(dyaw, -p.max_rot_rad, p.max_rot_rad);
  fix.valid = true;
  return fix;
}

// ------------------------------------------------------------- lap counting
// The line is the perpendicular to the heading at the pose where this node
// first saw odometry (both FSDS and the kinematic sim start the car on the
// start line; a mid-track start just measures laps from there instead).
//
// A crossing needs all three of: the car has driven at least `min_lap_m` since
// the last one (so the line cannot re-trigger while sitting on it), the
// along-track coordinate goes from behind the line to in front of it, and the
// car is inside `corridor_m` laterally — which is what rejects the far side of
// the track, where the same plane is crossed the other way.
class LapCounter
{
public:
  LapCounter(double min_lap_m = 20.0, double corridor_m = 6.0)
  : min_lap_m_(min_lap_m), corridor_m_(corridor_m) {}

  bool update(double x, double y, double yaw, double dist)
  {
    if (!have_origin_) {
      ox_ = x;
      oy_ = y;
      fx_ = std::cos(yaw);
      fy_ = std::sin(yaw);
      lap_start_dist_ = dist;
      prev_s_ = 0.0;
      have_origin_ = true;
      return false;
    }
    const double s = (x - ox_) * fx_ + (y - oy_) * fy_;
    const double lat = -(x - ox_) * fy_ + (y - oy_) * fx_;
    bool crossed = false;
    if (dist - lap_start_dist_ >= min_lap_m_ && prev_s_ < 0.0 && s >= 0.0 &&
        std::abs(lat) <= corridor_m_)
    {
      ++laps_;
      track_length_ = dist - lap_start_dist_;
      lap_start_dist_ = dist;
      crossed = true;
    }
    prev_s_ = s;
    return crossed;
  }

  uint32_t laps() const { return laps_; }
  double track_length() const { return track_length_; }
  double lap_start_dist() const { return lap_start_dist_; }

private:
  double min_lap_m_, corridor_m_;
  double ox_{0.0}, oy_{0.0}, fx_{1.0}, fy_{0.0};
  double lap_start_dist_{0.0}, track_length_{0.0}, prev_s_{0.0};
  uint32_t laps_{0};
  bool have_origin_{false};
};

}  // namespace

class ConeMappingNode : public rclcpp::Node
{
public:
  ConeMappingNode() : Node("cone_mapping")
  {
    declare_parameter<double>("search_radius_m", 1.2);
    declare_parameter<double>("gate_threshold_m", 0.5);
    declare_parameter<int>("n_confirm", 3);
    declare_parameter<double>("tentative_timeout_s", 1.5);
    declare_parameter<double>("publish_rate_hz", 10.0);
    // A detection whose own range uncertainty is comparable to the association
    // gate cannot be told apart from a neighbour, so it must not be allowed to
    // CREATE a landmark — it lands outside the gate of the cone it really is,
    // and three such in a row get promoted into a permanent duplicate that no
    // later observation can remove. Such detections still update landmarks they
    // do match, and still feed localization; the cone gets mapped properly when
    // the car is closer.
    declare_parameter<double>("max_spawn_sigma_m", 0.5);
    // A FIXED association gate assumes every detection is equally precise.
    // With one camera it is not: mono range error grows as z^2 (sigma =
    // 1.5*z/h_px and h_px falls as 1/z), so a cone at 10 m can legitimately
    // land a metre from its landmark while one at 4 m lands within 10 cm. A
    // fixed 0.5 m gate therefore refuses the far sighting and spawns a
    // duplicate. Scaling the gate by the detection's own sigma matches the
    // gate to the measurement — the standard treatment — with a hard ceiling
    // well under the cone pitch so neighbouring cones can never merge.
    declare_parameter<double>("gate_sigma_scale", 3.0);
    declare_parameter<double>("gate_max_m", 1.5);
    declare_parameter<bool>("localize_enable", true);
    declare_parameter<double>("localize_gate_m", 1.0);
    declare_parameter<int>("localize_min_inliers", 6);
    declare_parameter<int>("localize_min_obs", 5);
    declare_parameter<int>("localize_min_landmarks", 12);
    declare_parameter<double>("min_lap_distance_m", 20.0);
    declare_parameter<double>("start_corridor_m", 6.0);
    declare_parameter<int>("laps_to_known", 1);

    search_r_ = get_parameter("search_radius_m").as_double();
    gate_ = get_parameter("gate_threshold_m").as_double();
    n_confirm_ = static_cast<uint32_t>(get_parameter("n_confirm").as_int());
    tent_timeout_ = get_parameter("tentative_timeout_s").as_double();
    max_spawn_var_ = std::pow(get_parameter("max_spawn_sigma_m").as_double(), 2);
    gate_scale_ = get_parameter("gate_sigma_scale").as_double();
    gate_max_ = get_parameter("gate_max_m").as_double();
    loc_enable_ = get_parameter("localize_enable").as_bool();
    icp_.gate = get_parameter("localize_gate_m").as_double();
    icp_.min_inliers =
      static_cast<uint32_t>(get_parameter("localize_min_inliers").as_int());
    loc_min_obs_ =
      static_cast<uint32_t>(get_parameter("localize_min_obs").as_int());
    loc_min_landmarks_ =
      static_cast<size_t>(get_parameter("localize_min_landmarks").as_int());
    laps_to_known_ =
      static_cast<uint32_t>(get_parameter("laps_to_known").as_int());
    laps_ = LapCounter(get_parameter("min_lap_distance_m").as_double(),
                       get_parameter("start_corridor_m").as_double());

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odometry/filtered", fsd::qos_reliable(10),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr m) { on_odom(m); });
    cones_sub_ = create_subscription<fsd_msgs::msg::Cone3DArray>(
      "/perception/cones", fsd::qos_reliable(5),
      [this](fsd_msgs::msg::Cone3DArray::ConstSharedPtr m) { on_cones(m); });
    pub_ = create_publisher<fsd_msgs::msg::ConeMap>(
      "/mapping/track", fsd::qos_reliable(5));
    status_pub_ = create_publisher<TrackStatus>(
      "/mapping/status", fsd::qos_reliable(5));
    corr_pub_ = create_publisher<PoseCorrection>(
      "/localization/correction", fsd::qos_reliable(10));

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / get_parameter("publish_rate_hz").as_double()),
      [this]() { publish_map(); });
    compact_timer_ = create_wall_timer(std::chrono::seconds(10), [this]() {
      grid_confirmed_.compact(slots_, LmState::CONFIRMED);
      grid_tentative_.compact(slots_, LmState::TENTATIVE);
    });
    hb_ = std::make_unique<fsd::HeartbeatEmitter>(this, "cone_mapping");
  }

private:
  void on_odom(nav_msgs::msg::Odometry::ConstSharedPtr m)
  {
    const double x = m->pose.pose.position.x;
    const double y = m->pose.pose.position.y;
    const double yaw = fsd::yaw_from_quaternion(m->pose.pose.orientation);
    const double t = fsd::stamp_to_sec(m->header.stamp);
    poses_.add(t, x, y, yaw);

    // Odometer from speed, not from position deltas: localization corrections
    // move the pose without the car having travelled.
    if (last_odom_t_ > 0.0) {
      const double dt = t - last_odom_t_;
      if (dt > 0.0 && dt < 0.5) {
        dist_ += std::abs(m->twist.twist.linear.x) * dt;
      }
    }
    last_odom_t_ = t;
    if (laps_.update(x, y, yaw, dist_)) {
      RCLCPP_INFO(get_logger(), "lap %u complete: %.1f m%s", laps_.laps(),
                  laps_.track_length(),
                  laps_.laps() >= laps_to_known_
                    ? " — track KNOWN, race speed unlocked" : "");
    }
  }

  void on_cones(fsd_msgs::msg::Cone3DArray::ConstSharedPtr msg)
  {
    const auto pose = poses_.query(fsd::stamp_to_sec(msg->header.stamp));
    if (!pose) {
      return;
    }
    double px = pose->x, py = pose->y, pyaw = pose->yaw;

    // Map-relative localization, BEFORE the map is updated with this frame:
    // the fit must be against landmarks the frame did not create.
    if (loc_enable_ && confirmed_count_ >= loc_min_landmarks_ &&
        msg->cones.size() >= icp_.min_inliers)
    {
      std::vector<std::pair<double, double>> dets;
      dets.reserve(msg->cones.size());
      for (const auto & d : msg->cones) {
        dets.emplace_back(d.x, d.y);
      }
      const auto fix = solve_pose_correction(
        px, py, pyaw, dets,
        [this](double x, double y, double r) {
          std::vector<std::pair<double, double>> out;
          for (size_t s : grid_confirmed_.query(slots_, x, y, r)) {
            if (slots_[s].state == LmState::CONFIRMED &&
                slots_[s].obs_count >= loc_min_obs_)
            {
              out.emplace_back(slots_[s].x, slots_[s].y);
            }
          }
          return out;
        },
        icp_);

      PoseCorrection out;
      out.header.stamp = now();
      out.header.frame_id = "odom";
      out.valid = fix.valid;
      if (fix.valid) {
        out.dx = static_cast<float>(fix.dx);
        out.dy = static_cast<float>(fix.dy);
        out.dyaw = static_cast<float>(fix.dyaw);
        out.n_inliers = static_cast<uint16_t>(std::min<uint32_t>(fix.n_inliers, 65535));
        out.rms_m = static_cast<float>(fix.rms);
        loc_inliers_ = fix.n_inliers;
        loc_rms_ = fix.rms;
        // Use the corrected pose for THIS frame's association too — the
        // correction is measured now, and waiting a cycle only feeds the map a
        // pose we already know is off.
        px += fix.dx;
        py += fix.dy;
        pyaw = fsd::wrap_angle(pyaw + fix.dyaw);
      } else {
        loc_inliers_ = 0;
        loc_rms_ = 0.0;
      }
      corr_pub_->publish(out);
    }

    const double cos_y = std::cos(pyaw);
    const double sin_y = std::sin(pyaw);
    const double t = now().seconds();

    for (const auto & d : msg->cones) {
      const double ox = px + d.x * cos_y - d.y * sin_y;
      const double oy = py + d.x * sin_y + d.y * cos_y;
      const double sigma = std::max<double>(d.depth_sigma, 0.05);
      const double meas_var = sigma * sigma;
      const double gate = std::clamp(gate_scale_ * sigma, gate_, gate_max_);
      const uint8_t side = d.y > 0 ? ConeMapEntry::SIDE_LEFT
                                   : ConeMapEntry::SIDE_RIGHT;

      // 1. confirmed map first
      if (auto s = best_match(grid_confirmed_, LmState::CONFIRMED, ox, oy, gate)) {
        kalman_update_slot(*s, ox, oy, meas_var, d.color, side, t);
        continue;
      }
      // 2. tentative buffer
      if (auto s = best_match(grid_tentative_, LmState::TENTATIVE, ox, oy, gate)) {
        Landmark & lm = slots_[*s];
        kalman_update_slot(*s, ox, oy, meas_var, d.color, side, t);
        if (lm.obs_count >= n_confirm_) {
          lm.id = next_id_++;
          lm.state = LmState::CONFIRMED;                // no erase — state flip
          grid_confirmed_.insert(*s, lm.x, lm.y);
          ++confirmed_count_;
        }
      } else if (meas_var <= max_spawn_var_) {
        slots_.emplace_back(ox, oy, meas_var, d.color, side, t);
        grid_tentative_.insert(slots_.size() - 1, ox, oy);
      }
    }

    // 3. GC: tentative -> DEAD only. Confirmed is never deleted mid-run.
    for (auto & lm : slots_) {
      if (lm.state == LmState::TENTATIVE &&
          t - lm.first_seen > tent_timeout_ && lm.obs_count < n_confirm_)
      {
        lm.state = LmState::DEAD;   // lazy: grid entries filtered at query
      }
    }
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
      grid.insert(s, lm.x, lm.y);   // old entry stays: harmless duplicate
    }
  }

  std::optional<size_t> best_match(const GridIndex & grid, LmState want,
                                   double x, double y, double gate) const
  {
    std::optional<size_t> best;
    double best_d2 = gate * gate;
    for (size_t s : grid.query(slots_, x, y, std::max(search_r_, gate))) {
      if (slots_[s].state != want) {
        continue;                    // lazy deletion / stale-entry filter
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

  void publish_map()
  {
    fsd_msgs::msg::ConeMap msg;
    msg.header.stamp = now();
    msg.header.frame_id = "odom";
    msg.cones.reserve(confirmed_count_);
    for (const auto & lm : slots_) {
      if (lm.state != LmState::CONFIRMED) {
        continue;
      }
      ConeMapEntry e;
      e.id = lm.id;
      e.color = lm.best_color();
      e.x = static_cast<float>(lm.x);
      e.y = static_cast<float>(lm.y);
      e.side = lm.best_side();
      e.observation_count = static_cast<uint16_t>(std::min<uint32_t>(lm.obs_count, 65535));
      msg.cones.push_back(e);
    }
    pub_->publish(msg);

    TrackStatus st;
    st.header.stamp = msg.header.stamp;
    st.header.frame_id = "odom";
    st.mode = laps_.laps() >= laps_to_known_ ? TrackStatus::MODE_KNOWN
                                             : TrackStatus::MODE_EXPLORE;
    st.laps_completed = static_cast<uint16_t>(std::min<uint32_t>(laps_.laps(), 65535));
    st.loop_closed = laps_.laps() > 0;
    st.lap_distance_m = static_cast<float>(dist_ - laps_.lap_start_dist());
    st.track_length_m = static_cast<float>(laps_.track_length());
    st.confirmed_cones =
      static_cast<uint16_t>(std::min<size_t>(confirmed_count_, 65535));
    st.localized_cones =
      static_cast<uint16_t>(std::min<uint32_t>(loc_inliers_, 65535));
    st.localization_rms_m = static_cast<float>(loc_rms_);
    status_pub_->publish(st);
  }

  double search_r_, gate_, tent_timeout_, max_spawn_var_{0.25};
  double gate_scale_{3.0}, gate_max_{1.5};
  uint32_t n_confirm_, next_id_{1};
  size_t confirmed_count_{0};
  std::vector<Landmark> slots_;              // append-only slot map
  GridIndex grid_confirmed_, grid_tentative_;
  fsd::PoseBuffer poses_;

  bool loc_enable_{true};
  IcpParams icp_;
  uint32_t loc_min_obs_{5}, loc_inliers_{0}, laps_to_known_{1};
  size_t loc_min_landmarks_{12};
  double loc_rms_{0.0}, dist_{0.0}, last_odom_t_{-1.0};
  LapCounter laps_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<fsd_msgs::msg::Cone3DArray>::SharedPtr cones_sub_;
  rclcpp::Publisher<fsd_msgs::msg::ConeMap>::SharedPtr pub_;
  rclcpp::Publisher<TrackStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<PoseCorrection>::SharedPtr corr_pub_;
  rclcpp::TimerBase::SharedPtr timer_, compact_timer_;
  std::unique_ptr<fsd::HeartbeatEmitter> hb_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ConeMappingNode>());
  rclcpp::shutdown();
  return 0;
}
