#pragma once

// Shared by OpenCV and CUDA. HSV ranges are OpenCV's H:[0,179], S/V:[0,255].
namespace fsd_hsv {
constexpr int blue_h_min = 100, blue_h_max = 130, blue_s_min = 85, blue_v_min = 100;
constexpr int yellow_h_min = 20, yellow_h_max = 38, yellow_s_min = 25, yellow_v_min = 90;
constexpr int orange_h_min = 5, orange_h_max = 18, orange_s_min = 80, orange_v_min = 80;
}
