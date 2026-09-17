// BLOCKS 1+2 — STEREO CONE PERCEPTION (C++/CUDA).
//
// /camera/left/image_raw + /camera/right/image_raw
//     -> /perception/cones           (fsd_msgs/Cone3DArray, base_link)
//     -> /perception/cone_detections (fsd_msgs/ConeDetection2DArray, debug/dashboard)
//
// Pipeline:
//   1. Color segmentation: one-pass BGR->HSV->3 masks. On the GPU via the
//      CUDA kernel when built with CUDA (Jetson), identical CPU path
//      otherwise or on any CUDA runtime error.
//   2. Contours + geometric filtering per mask (CPU, cheap).
//   3. Range per detection: stereo SAD patch matching along the epipolar
//      row in the right image (cameras are a rectified horizontal pair —
//      true by construction in FSDS); monocular known-height fallback when
//      the match fails the ratio test. depth_sigma reflects the source.
//
// The FSDS launch remaps the camera topics onto the /fsds/... ones.

#include <algorithm>
#include <chrono>
#include <climits>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>

#include <fsd_msgs/msg/cone_detection2_d.hpp>
#include <fsd_msgs/msg/cone_detection2_d_array.hpp>
#include <fsd_msgs/msg/cone3_d.hpp>
#include <fsd_msgs/msg/cone3_d_array.hpp>
#include <fsd_msgs/msg/heartbeat.hpp>

#include "fsd_cpp/common.hpp"
#include "fsd_cpp/hsv_thresholds.hpp"
#ifdef FSD_USE_CUDA
#include "fsd_cpp/segmentation.hpp"
#endif

using fsd_msgs::msg::ConeDetection2D;
using fsd_msgs::msg::Heartbeat;

namespace
{

struct Det
{
  uint8_t color;
  cv::Rect box;
};

constexpr double kConeHeightSmall = 0.325;
constexpr double kConeHeightBig = 0.505;

double cone_height_for(uint8_t color)
{
  return color == ConeDetection2D::COLOR_ORANGE_BIG ? kConeHeightBig
                                                    : kConeHeightSmall;
}

}  // namespace

class StereoConeNode : public rclcpp::Node
{
public:
  StereoConeNode() : Node("stereo_cone")
  {
    declare_parameter<double>("fx", 320.0);
    declare_parameter<double>("fy", 320.0);
    declare_parameter<double>("cx", 320.0);
    declare_parameter<double>("cy", 240.0);
    declare_parameter<double>("baseline_m", 0.12);
    declare_parameter<double>("max_range_m", 18.0);
    declare_parameter<double>("cam_offset_x", 1.0);
    declare_parameter<double>("cam_offset_y", 0.0);
    declare_parameter<double>("cam_offset_z", 0.8);
    // A calibrated, level-camera flat-ground model. Zero retains known-height
    // ranging for uncalibrated hardware; do not reuse the FSDS height on a car.
    declare_parameter<double>("ground_camera_height_m", 0.0);
    declare_parameter<int>("min_contour_area_px", 60);
    // Height of the vertical CLOSE applied to each colour mask, in pixels.
    // FS cones are STRIPED: a white band across the middle cuts the coloured
    // region into two blobs. Their bounding boxes are each about half the cone
    // tall, and mono range is fy*H/h_px, so an unmerged fragment reports a cone
    // at roughly twice its real distance — measured live at 640x480, one yellow
    // cone came out as 17 px / 6.1 m in fragments against 36 px / 2.9 m merged.
    // That error alone stretches the corridor past the pairing gate and no path
    // is ever produced. CLOSE (dilate then erode) bridges the stripe without
    // eroding small distant blobs, which is why it is used here and OPEN is
    // not. Set 0 to disable at very low resolution, where cones are a few px
    // tall and there is no stripe to bridge.
    declare_parameter<int>("mask_close_px", 9);
    // A cone is always taller than it is wide (0.325 m / 0.228 m = 1.4), so a
    // wide, short blob is a stripe fragment or a reflection, never a cone.
    // This matters far more for mono than for stereo: here the height IS the
    // range measurement.
    declare_parameter<double>("min_aspect", 0.7);
    declare_parameter<double>("max_aspect", 3.5);
    // Row above which detections are rejected as sky. -1 disables the gate;
    // set it to the MEASURED horizon row for the camera, never assume cy.
    declare_parameter<int>("horizon_row_px", -1);
    declare_parameter<int>("max_disparity_px", 128);
    declare_parameter<int>("patch_half_px", 7);
    // SINGLE-CAMERA MODE. The primary build target is mono known-height
    // ranging: no rectification, no baseline to keep true on a car that
    // vibrates. Bearing is as accurate as any stereo path (same pixel, same
    // intrinsics); only range degrades, as sigma = 1.5 * z / h_px.
    declare_parameter<bool>("mono_only", true);
    // MULTI-CAMERA. One forward camera sees +-45 deg, and that is not enough
    // in a corner: the inside boundary leaves the frame sideways exactly when
    // the car needs it, which is why runs survived straights and died on
    // turns. Three cameras at -45/0/+45 cover ~180 deg so a boundary stays in
    // view through the corner. Each camera has its own yaw and mounting point;
    // detections are rotated into base_link by that yaw, so a cone seen by the
    // side camera lands in the same frame as one seen ahead.
    //
    // A single empty string is the "no list given" sentinel, NOT an empty
    // array: rclcpp declares an array parameter with an empty default as
    // statically typed but uninitialized, and then throws
    // UninitializedStaticallyTypedParameterException on the first read —
    // inside the constructor, killing the node before it subscribes. That is
    // what made the camera stream at 10 Hz while perception published nothing.
    declare_parameter<std::vector<std::string>>("camera_topics", {""});
    declare_parameter<std::vector<double>>("camera_yaw_deg", {0.0});
    declare_parameter<std::vector<double>>("camera_x", {0.0});
    declare_parameter<std::vector<double>>("camera_y", {0.0});

    fx_ = get_parameter("fx").as_double();
    fy_ = get_parameter("fy").as_double();
    cx_ = get_parameter("cx").as_double();
    cy_ = get_parameter("cy").as_double();
    baseline_ = get_parameter("baseline_m").as_double();
    max_range_ = get_parameter("max_range_m").as_double();
    off_x_ = get_parameter("cam_offset_x").as_double();
    off_y_ = get_parameter("cam_offset_y").as_double();
    off_z_ = get_parameter("cam_offset_z").as_double();
    ground_height_ = get_parameter("ground_camera_height_m").as_double();
    min_area_ = get_parameter("min_contour_area_px").as_int();
    close_px_ = get_parameter("mask_close_px").as_int();
    min_aspect_ = get_parameter("min_aspect").as_double();
    max_aspect_ = get_parameter("max_aspect").as_double();
    horizon_row_px_ = get_parameter("horizon_row_px").as_int();
    max_disp_ = get_parameter("max_disparity_px").as_int();
    patch_half_ = get_parameter("patch_half_px").as_int();
    mono_only_ = get_parameter("mono_only").as_bool();

    cones_pub_ = create_publisher<fsd_msgs::msg::Cone3DArray>(
      "/perception/cones", fsd::qos_reliable(5));
    dets_pub_ = create_publisher<fsd_msgs::msg::ConeDetection2DArray>(
      "/perception/cone_detections", fsd::qos_reliable(5));

    // A parameter declared with an EMPTY array default can come back as
    // NOT_SET, and as_string_array() then THROWS — inside the constructor,
    // which kills the node before it ever subscribes. That is exactly what
    // happened when the camera list was commented out of the YAML: the camera
    // streamed at 10 Hz and perception published nothing at all. Read these
    // defensively.
    auto str_array = [this](const char * name) {
      std::vector<std::string> v;
      try {
        const auto p = get_parameter(name);
        if (p.get_type() == rclcpp::ParameterType::PARAMETER_STRING_ARRAY) {
          v = p.as_string_array();
        }
      } catch (const std::exception &) {
      }
      return v;
    };
    auto dbl_array = [this](const char * name) {
      std::vector<double> v;
      try {
        const auto p = get_parameter(name);
        if (p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
          v = p.as_double_array();
        }
      } catch (const std::exception &) {
      }
      return v;
    };
    auto topics = str_array("camera_topics");
    topics.erase(std::remove_if(topics.begin(), topics.end(),
                                [](const std::string & s) { return s.empty(); }),
                 topics.end());
    const auto yaws = dbl_array("camera_yaw_deg");
    const auto cxs = dbl_array("camera_x");
    const auto cys = dbl_array("camera_y");
    if (topics.empty()) {
      cams_.push_back({"/camera/left/image_raw", 0.0, off_x_, off_y_});
    } else {
      for (size_t i = 0; i < topics.size(); ++i) {
        cams_.push_back({topics[i],
                         (i < yaws.size() ? yaws[i] : 0.0) * M_PI / 180.0,
                         i < cxs.size() ? cxs[i] : off_x_,
                         i < cys.size() ? cys[i] : off_y_});
      }
    }
    for (size_t i = 0; i < cams_.size(); ++i) {
      cam_subs_.push_back(create_subscription<sensor_msgs::msg::Image>(
        cams_[i].topic, fsd::qos_best_effort(1),
        [this, i](sensor_msgs::msg::Image::ConstSharedPtr msg) {
          on_image(msg, i);
        }));
    }
    if (!mono_only_) {
      right_sub_ = create_subscription<sensor_msgs::msg::Image>(
        "/camera/right/image_raw", fsd::qos_best_effort(1),
        [this](sensor_msgs::msg::Image::ConstSharedPtr msg) { on_right(msg); });
    }

    hb_ = std::make_unique<fsd::HeartbeatEmitter>(this, "stereo_cone");
    RCLCPP_INFO(get_logger(), "ranging: %s, %zu camera(s)",
                mono_only_ ? (ground_height_ > 0.0 ? "MONO (calibrated ground plane)" :
                  "MONO (known cone height)") : "stereo + mono fallback",
                cams_.size());
    for (const auto & c : cams_) {
      RCLCPP_INFO(get_logger(), "  cam %s yaw %+.0f deg at (%.2f, %.2f)",
                  c.topic.c_str(), c.yaw * 180.0 / M_PI, c.x, c.y);
    }
#ifdef FSD_USE_CUDA
    RCLCPP_INFO(get_logger(), "built with CUDA segmentation");
#else
    RCLCPP_INFO(get_logger(), "built CPU-only (no CUDA at compile time)");
#endif
  }

private:
  void on_right(sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    auto cvp = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    // CPU gray kept as the fallback path even when the GPU one succeeds.
    cv::cvtColor(cvp->image, right_gray_, cv::COLOR_BGR2GRAY);
    right_stamp_ = fsd::stamp_to_sec(msg->header.stamp);
    right_on_gpu_ = false;
#ifdef FSD_USE_CUDA
    right_on_gpu_ = cvp->image.isContinuous() &&
                    fsd_cuda::upload_right(cvp->image.ptr<uint8_t>(),
                                           cvp->image.cols, cvp->image.rows);
#endif
  }

  void on_image(sensor_msgs::msg::Image::ConstSharedPtr msg, size_t cam_idx)
  {
    const Cam & cam = cams_[cam_idx];
    auto cvp = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    const cv::Mat & frame = cvp->image;
    cv::Mat left_gray;
    cv::cvtColor(frame, left_gray, cv::COLOR_BGR2GRAY);

    // ---- 1. segmentation (GPU with CPU fallback)
    cv::Mat mb(frame.rows, frame.cols, CV_8UC1);
    cv::Mat my(frame.rows, frame.cols, CV_8UC1);
    cv::Mat mo(frame.rows, frame.cols, CV_8UC1);
    bool gpu_ok = false;
#ifdef FSD_USE_CUDA
    gpu_ok = frame.isContinuous() &&
             fsd_cuda::segment(frame.ptr<uint8_t>(), frame.cols, frame.rows,
                               mb.ptr<uint8_t>(), my.ptr<uint8_t>(),
                               mo.ptr<uint8_t>());
#endif
    if (!gpu_ok) {
      segment_cpu(frame, mb, my, mo);
    }

    // ---- 2. contours per mask
    std::vector<Det> dets;
    extract(mb, ConeDetection2D::COLOR_BLUE, dets);
    extract(my, ConeDetection2D::COLOR_YELLOW, dets);
    extract(mo, ConeDetection2D::COLOR_ORANGE_SMALL, dets);

    // ---- publish 2D detections (debug/dashboard)
    fsd_msgs::msg::ConeDetection2DArray dmsg;
    dmsg.header = msg->header;
    for (const auto & d : dets) {
      ConeDetection2D dd;
      dd.color = d.color;
      dd.confidence = 0.8f;
      dd.cx = d.box.x + d.box.width / 2.0f;
      dd.cy = d.box.y + d.box.height / 2.0f;
      dd.width = static_cast<float>(d.box.width);
      dd.height = static_cast<float>(d.box.height);
      dd.source = ConeDetection2D::SOURCE_HSV;
      dmsg.detections.push_back(dd);
    }
    dets_pub_->publish(dmsg);

    // ---- 3. range + 3D output
    const bool stereo_ok =
      !mono_only_ && !right_gray_.empty() &&
      std::abs(fsd::stamp_to_sec(msg->header.stamp) - right_stamp_) < 0.05 &&
      right_gray_.size() == left_gray.size();

    // Batched GPU disparity for every eligible detection in one kernel
    // launch (left gray was cached on-device by segment()).
    std::vector<int> gpu_disp(dets.size(), -1);
    std::vector<float> gpu_ratio(dets.size(), 1.0f);
    bool gpu_match_ok = false;
#ifdef FSD_USE_CUDA
    if (stereo_ok && gpu_ok && right_on_gpu_) {
      std::vector<int> cxs, cys, idx;
      for (size_t i = 0; i < dets.size(); ++i) {
        if (patch_in_bounds(left_gray, dets[i])) {
          cxs.push_back(dets[i].box.x + dets[i].box.width / 2);
          cys.push_back(dets[i].box.y + dets[i].box.height / 2);
          idx.push_back(static_cast<int>(i));
        }
      }
      if (!cxs.empty()) {
        std::vector<int> disp(cxs.size());
        std::vector<float> ratio(cxs.size());
        gpu_match_ok = fsd_cuda::match_disparities(
          cxs.data(), cys.data(), static_cast<int>(cxs.size()),
          patch_half_, max_disp_, disp.data(), ratio.data());
        if (gpu_match_ok) {
          for (size_t k = 0; k < idx.size(); ++k) {
            gpu_disp[idx[k]] = disp[k];
            gpu_ratio[idx[k]] = ratio[k];
          }
        }
      } else {
        gpu_match_ok = true;   // nothing eligible; don't fall back to CPU
      }
    }
#endif

    fsd_msgs::msg::Cone3DArray out;
    out.header.stamp = msg->header.stamp;
    out.header.frame_id = "base_link";
    uint32_t next_id = 0;
    int n_stereo = 0;
    for (size_t di = 0; di < dets.size(); ++di) {
      const auto & d = dets[di];
      double depth = -1.0, sigma = 1e9;
      if (gpu_match_ok) {
        // GPU result; ratio test identical to the CPU path.
        if (gpu_disp[di] >= 1 && gpu_ratio[di] <= 0.8f) {
          depth = fx_ * baseline_ / gpu_disp[di];
          sigma = depth * depth / (fx_ * baseline_);
          ++n_stereo;
        } else {
          mono_range(d, depth, sigma);
        }
      } else if (stereo_ok && match_stereo(left_gray, d, depth, sigma)) {
        ++n_stereo;
      } else {
        mono_range(d, depth, sigma);
      }
      if (depth <= 0.5 || depth > max_range_ || sigma > 4.0) {
        continue;
      }
      const double u = d.box.x + d.box.width / 2.0;
      const double v = d.box.y + d.box.height / 2.0;
      const double cam_x = (u - cx_) * depth / fx_;  // right
      const double cam_y = (v - cy_) * depth / fy_;  // down
      // Camera frame -> base_link: rotate by this camera's yaw, then translate
      // by its mounting point. For the forward camera (yaw 0) this reduces to
      // the original expression.
      const double fwd = depth;                       // along the camera axis
      const double lft = -cam_x;                      // left of the camera
      const double cyaw = std::cos(cam.yaw), syaw = std::sin(cam.yaw);
      fsd_msgs::msg::Cone3D c;
      c.id = next_id++;
      c.color = d.color;
      c.confidence = 0.8f;
      c.x = static_cast<float>(cam.x + fwd * cyaw - lft * syaw);
      c.y = static_cast<float>(cam.y + fwd * syaw + lft * cyaw);
      c.z = static_cast<float>(-cam_y + off_z_);
      c.depth_sigma = static_cast<float>(sigma);
      out.cones.push_back(c);
    }
    cones_pub_->publish(out);

    // Mono is the designed configuration, not a degradation. Only a stereo
    // build that has LOST its second camera is degraded.
    if (!mono_only_ && !stereo_ok) {
      hb_->set_status(Heartbeat::STATUS_DEGRADED, "no fresh right image, mono ranging");
    } else {
      hb_->set_status(Heartbeat::STATUS_OK);
    }
    (void)n_stereo;
  }

  // CPU fallback: identical thresholds to the CUDA kernel.
  static void segment_cpu(const cv::Mat & bgr, cv::Mat & mb, cv::Mat & my,
                          cv::Mat & mo)
  {
    // Thresholds must match segmentation.cu exactly.
    //
    // Captured FSDS asphalt overlapped the old blue S>=50,V>=60 range, joining
    // real cones to the entire road. Separate that background here; sky and
    // implausible physical dimensions are rejected by extract().
    cv::Mat hsv;
    cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);
    using namespace fsd_hsv;
    cv::inRange(hsv, cv::Scalar(blue_h_min, blue_s_min, blue_v_min),
                cv::Scalar(blue_h_max, 255, 255), mb);
    cv::inRange(hsv, cv::Scalar(yellow_h_min, yellow_s_min, yellow_v_min),
                cv::Scalar(yellow_h_max, 255, 255), my);
    cv::inRange(hsv, cv::Scalar(orange_h_min, orange_s_min, orange_v_min),
                cv::Scalar(orange_h_max, 255, 255), mo);
  }

  void extract(cv::Mat & mask, uint8_t color, std::vector<Det> & dets) const
  {
    if (ground_height_ > 0.0) {
      // Cones below a level elevated camera cannot extend into the sky. Cut it
      // before morphology so sky/fence pixels cannot join a ground candidate.
      const int row = std::clamp(static_cast<int>(cy_) + 1, 0, mask.rows);
      if (row > 0) { mask.rowRange(0, row).setTo(0); }
    }
    // Vertical CLOSE first: bridge the white stripe so one cone is one blob of
    // full height. No OPEN anywhere — it would erode distant cones, which are
    // only a few pixels, and the sim image is clean enough not to need it.
    if (close_px_ > 0) {
      const cv::Mat k = cv::getStructuringElement(cv::MORPH_RECT,
                                                  cv::Size(3, close_px_));
      cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, k);
    }
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    std::vector<cv::Rect> boxes;
    for (const auto & c : contours) {
      if (cv::contourArea(c) >= 2.0) { boxes.push_back(cv::boundingRect(c)); }
    }
    // Rejoin stripes adaptively, without a tall fixed kernel that connects two
    // separate distant cones. Components must share a vertical centre axis.
    for (size_t i = 0; i < boxes.size(); ++i) {
      for (size_t j = i + 1; j < boxes.size();) {
        const auto a = boxes[i], b = boxes[j];
        const double dx = std::abs(a.x + a.width / 2.0 - b.x - b.width / 2.0);
        const int gap = std::max(a.y, b.y) - std::min(a.y+a.height, b.y+b.height);
        const auto united = a | b;
        if (dx <= 1.0 + 0.25 * std::max(a.width, b.width) && gap >= 0 &&
            gap <= std::max(3.0, 0.8 * std::max(a.height, b.height)) &&
            united.height <= 3.5 * united.width) {
          boxes[i] = united;
          boxes.erase(boxes.begin() + j);
          j = i + 1;
        } else { ++j; }
      }
    }
    for (const auto & box : boxes) {
      if (cv::countNonZero(mask(box)) < min_area_) { continue; }
      const double aspect = static_cast<double>(box.height) / std::max(box.width, 1);
      if (aspect < min_aspect_ || aspect > max_aspect_) {
        continue;
      }
      // A ground cone may START above the horizon when it is close; its BASE
      // cannot finish above it. The previous gate tested box.y (the top), so it
      // rejected exactly the close cones needed for turning. Gate the bounding
      // box bottom instead. This removes blue sky fragments while retaining a
      // cone whose top crosses the horizon.
      if (horizon_row_px_ >= 0 && box.y + box.height < horizon_row_px_) {
        continue;
      }
      if (ground_height_ > 0.0) {
        const double below = box.y + box.height - cy_;
        if (below <= 2.0 || box.y <= cy_ + 1 ||
            box.y + box.height >= mask.rows - 1 || box.x <= 0 ||
            box.x + box.width >= mask.cols - 1) { continue; }
        const double depth = fy_ * ground_height_ / below;
        const double height = depth * box.height / fy_;
        const double width = depth * box.width / fx_;
        // Reject sky remnants, road texture and physically implausible blobs.
        // Broad size bounds allow a partly desaturated cone, not arbitrary sky.
        if (height < 0.12 || height > 0.65 || width < 0.06 || width > 0.65) { continue; }
      }
      dets.push_back({color, box});
    }
  }

  bool patch_in_bounds(const cv::Mat & img, const Det & d) const
  {
    const int u = d.box.x + d.box.width / 2;
    const int v = d.box.y + d.box.height / 2;
    return u - patch_half_ >= 0 && v - patch_half_ >= 0 &&
           u + patch_half_ < img.cols && v + patch_half_ < img.rows;
  }

  // CPU SAD patch match along the epipolar row (fallback when the GPU
  // batch path is unavailable). Rectified pair: the right-image match sits
  // at u_left - disparity on the same row.
  bool match_stereo(const cv::Mat & left_gray, const Det & d,
                    double & depth, double & sigma) const
  {
    const int u = d.box.x + d.box.width / 2;
    const int v = d.box.y + d.box.height / 2;
    const int ph = patch_half_;
    if (!patch_in_bounds(left_gray, d)) {
      return false;
    }
    const cv::Mat patch = left_gray(cv::Range(v - ph, v + ph + 1),
                                    cv::Range(u - ph, u + ph + 1));
    long best = LONG_MAX, second = LONG_MAX;
    int best_disp = -1;
    const int max_d = std::min(max_disp_, u - ph);
    for (int disp = 1; disp <= max_d; ++disp) {
      const int ur = u - disp;
      const cv::Mat cand = right_gray_(cv::Range(v - ph, v + ph + 1),
                                       cv::Range(ur - ph, ur + ph + 1));
      const long sad = static_cast<long>(cv::norm(patch, cand, cv::NORM_L1));
      if (sad < best) {
        second = best;
        best = sad;
        best_disp = disp;
      } else if (sad < second) {
        second = sad;
      }
    }
    // Ratio test: an ambiguous match is worse than no match.
    if (best_disp < 1 || second == LONG_MAX ||
        static_cast<double>(best) > 0.8 * static_cast<double>(second))
    {
      return false;
    }
    depth = fx_ * baseline_ / best_disp;
    sigma = depth * depth / (fx_ * baseline_);  // 1 px disparity error
    return true;
  }

  void mono_range(const Det & d, double & depth, double & sigma) const
  {
    if (ground_height_ > 0.0) {
      const double below = d.box.y + d.box.height - cy_;
      if (below <= 2.0) { depth = -1.0; return; }
      depth = fy_ * ground_height_ / below;
      sigma = std::hypot(1.5 * depth / below, 0.03 * depth);
      return;
    }
    if (d.box.height < 4) {
      depth = -1.0;
      return;
    }
    depth = fy_ * cone_height_for(d.color) / d.box.height;
    // ~2 px height uncertainty propagated to range. (The earlier 3x factor
    // was so pessimistic it dropped every small/distant FSDS cone.)
    sigma = std::max(1.5 * depth / d.box.height, 0.05);
  }

  double fx_, fy_, cx_, cy_, baseline_, max_range_;
  double off_x_, off_y_, off_z_, ground_height_;
  int min_area_, max_disp_, patch_half_, close_px_{9}, horizon_row_px_{-1};
  double min_aspect_{0.7}, max_aspect_{3.5};
  bool mono_only_{true};

  struct Cam
  {
    std::string topic;
    double yaw, x, y;
  };
  std::vector<Cam> cams_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> cam_subs_;

  cv::Mat right_gray_;
  double right_stamp_{-1.0};
  bool right_on_gpu_{false};

  rclcpp::Publisher<fsd_msgs::msg::Cone3DArray>::SharedPtr cones_pub_;
  rclcpp::Publisher<fsd_msgs::msg::ConeDetection2DArray>::SharedPtr dets_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr right_sub_;
  std::unique_ptr<fsd::HeartbeatEmitter> hb_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StereoConeNode>());
  rclcpp::shutdown();
  return 0;
}
