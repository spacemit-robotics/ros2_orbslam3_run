#pragma once

#include <array>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>

#include <sensor_msgs/msg/camera_info.hpp>
#include <unistd.h>

namespace orbslam3_run {

inline std::string formatNumber(double value) {
  std::ostringstream stream;
  stream << std::showpoint << std::setprecision(17) << value;
  return stream.str();
}

inline std::string trim(std::string value) {
  const auto first = value.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) return {};
  const auto last = value.find_last_not_of(" \t\r\n");
  return value.substr(first, last - first + 1);
}

inline std::string createRuntimeCameraSettings(
    const std::string &settings_template,
    const sensor_msgs::msg::CameraInfo &camera_info,
    const std::string &node_name) {
  if (camera_info.width == 0 || camera_info.height == 0 ||
      camera_info.k[0] <= 0.0 || camera_info.k[4] <= 0.0) {
    throw std::invalid_argument(
        "CameraInfo must contain positive dimensions and focal lengths");
  }
  if (camera_info.width != 640 || camera_info.height != 480) {
    throw std::invalid_argument("CameraInfo must describe 640x480 images");
  }
  if (!camera_info.distortion_model.empty() &&
      camera_info.distortion_model != "plumb_bob" &&
      camera_info.distortion_model != "rational_polynomial") {
    throw std::invalid_argument("unsupported CameraInfo distortion model: " +
                                camera_info.distortion_model);
  }

  const auto distortion = [&camera_info](std::size_t index) {
    return index < camera_info.d.size() ? camera_info.d[index] : 0.0;
  };

  const std::unordered_map<std::string, std::string> replacements{
      {"Camera1.fx", formatNumber(camera_info.k[0])},
      {"Camera1.fy", formatNumber(camera_info.k[4])},
      {"Camera1.cx", formatNumber(camera_info.k[2])},
      {"Camera1.cy", formatNumber(camera_info.k[5])},
      {"Camera1.k1", formatNumber(distortion(0))},
      {"Camera1.k2", formatNumber(distortion(1))},
      {"Camera1.p1", formatNumber(distortion(2))},
      {"Camera1.p2", formatNumber(distortion(3))},
      {"Camera1.k3", formatNumber(distortion(4))},
      {"Camera.width", "640"},
      {"Camera.height", "480"},
  };

  std::ifstream input(settings_template);
  if (!input) {
    throw std::runtime_error("cannot open ORB settings template: " +
                             settings_template);
  }
  const auto output_path =
      std::filesystem::temp_directory_path() /
      (node_name + "_camera_" + std::to_string(::getpid()) + ".yaml");
  std::ofstream output(output_path, std::ios::trunc);
  if (!output) {
    throw std::runtime_error("cannot create runtime ORB settings: " +
                             output_path.string());
  }

  std::unordered_set<std::string> replaced;
  std::string line;
  while (std::getline(input, line)) {
    const auto separator = line.find(':');
    const auto key =
        separator == std::string::npos ? std::string{} : trim(line.substr(0, separator));
    const auto replacement = replacements.find(key);
    if (replacement == replacements.end()) {
      output << line << '\n';
    } else {
      output << line.substr(0, separator + 1) << ' ' << replacement->second
             << '\n';
      replaced.insert(key);
    }
  }
  const std::array<const char *, 11> calibration_keys{
      "Camera1.fx", "Camera1.fy", "Camera1.cx", "Camera1.cy",
      "Camera1.k1", "Camera1.k2", "Camera1.p1", "Camera1.p2",
      "Camera1.k3", "Camera.width", "Camera.height"};
  for (const char *key : calibration_keys) {
    if (replaced.count(key) == 0) {
      output << key << ": " << replacements.at(key) << '\n';
    }
  }
  output.close();
  if (!output) {
    std::filesystem::remove(output_path);
    throw std::runtime_error("failed to write runtime ORB settings: " +
                             output_path.string());
  }
  return output_path.string();
}

}  // namespace orbslam3_run
