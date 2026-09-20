#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

#include <Eigen/Geometry>
#include <System.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <tf2/exceptions.hpp>
#include <tf2/time.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "runtime_camera_config.hpp"

class OrbSlam3OdometryNode final : public rclcpp::Node {
 public:
  using Image = sensor_msgs::msg::Image;
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<Image, Image>;

  OrbSlam3OdometryNode() : Node("orbslam3_odometry") {
    vocabulary_ = declare_parameter<std::string>("vocabulary");
    settings_template_ = declare_parameter<std::string>("settings");
    publish_tf_ = declare_parameter<bool>("publish_tf", false);
    odom_frame_ = declare_parameter<std::string>("odom_frame", "orb_odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    odom_2d_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom_2d", 10);
    state_pub_ = create_publisher<std_msgs::msg::UInt8>("tracking_state", 10);
    if (publish_tf_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        "/camera/color/camera_info", rclcpp::SensorDataQoS(),
        std::bind(&OrbSlam3OdometryNode::cameraInfoCallback, this,
                  std::placeholders::_1));

    color_sub_.subscribe(this, "/camera/color/image_raw", rmw_qos_profile_sensor_data);
    depth_sub_.subscribe(this, "/camera/aligned_depth_to_color/image_raw",
                         rmw_qos_profile_sensor_data);
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
        SyncPolicy(10), color_sub_, depth_sub_);
    sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.04));
    sync_->registerCallback(std::bind(&OrbSlam3OdometryNode::imageCallback, this,
                                      std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(get_logger(),
                "Waiting for CameraInfo; intrinsics will be read from the topic "
                "and extrinsics from TF");
  }

  ~OrbSlam3OdometryNode() override {
    if (slam_) slam_->Shutdown();
    if (!runtime_settings_.empty()) {
      std::error_code error;
      std::filesystem::remove(runtime_settings_, error);
    }
  }

 private:
  void cameraInfoCallback(
      const sensor_msgs::msg::CameraInfo::ConstSharedPtr &camera_info) {
    std::lock_guard<std::mutex> lock(track_mutex_);
    if (slam_) return;
    try {
      runtime_settings_ = orbslam3_run::createRuntimeCameraSettings(
          settings_template_, *camera_info, get_name());
      slam_ = std::make_unique<ORB_SLAM3::System>(
          vocabulary_, runtime_settings_, ORB_SLAM3::System::RGBD, false);
      RCLCPP_INFO(
          get_logger(),
          "Initialized ORB-SLAM3 from CameraInfo %ux%u (%s); runtime settings: %s",
          camera_info->width, camera_info->height,
          camera_info->header.frame_id.c_str(), runtime_settings_.c_str());
      camera_info_sub_.reset();
    } catch (const std::exception &error) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                            "Cannot initialize from CameraInfo: %s",
                            error.what());
    }
  }

  bool initializeCameraExtrinsics(const std::string &camera_frame) {
    if (have_camera_extrinsics_) return true;
    if (camera_frame.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Color image frame_id is empty; cannot look up camera extrinsics");
      return false;
    }

    try {
      // lookupTransform(target, source) returns the transform source -> target.
      const auto transform = tf_buffer_->lookupTransform(
          base_frame_, camera_frame, tf2::TimePointZero);
      const auto &translation = transform.transform.translation;
      const auto &rotation = transform.transform.rotation;
      Eigen::Quaternionf quaternion(
          static_cast<float>(rotation.w), static_cast<float>(rotation.x),
          static_cast<float>(rotation.y), static_cast<float>(rotation.z));
      if (quaternion.squaredNorm() < 1e-12f) {
        throw std::runtime_error("camera extrinsics contain a zero quaternion");
      }

      t_base_camera_optical_.setIdentity();
      t_base_camera_optical_.translation() = Eigen::Vector3f(
          static_cast<float>(translation.x), static_cast<float>(translation.y),
          static_cast<float>(translation.z));
      t_base_camera_optical_.linear() =
          quaternion.normalized().toRotationMatrix();
      have_camera_extrinsics_ = true;
      RCLCPP_INFO(get_logger(),
                  "Loaded camera extrinsics from TF: %s -> %s",
                  base_frame_.c_str(), camera_frame.c_str());
      return true;
    } catch (const tf2::TransformException &e) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Waiting for camera extrinsics TF %s -> %s: %s",
          base_frame_.c_str(), camera_frame.c_str(), e.what());
      return false;
    }
  }

  void imageCallback(const Image::ConstSharedPtr &color_msg,
                     const Image::ConstSharedPtr &depth_msg) {
    std::lock_guard<std::mutex> lock(track_mutex_);
    try {
      if (!slam_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "Waiting for color CameraInfo");
        return;
      }
      if (!initializeCameraExtrinsics(color_msg->header.frame_id)) return;

      cv::Mat color = cv_bridge::toCvShare(color_msg)->image;
      cv::Mat depth = cv_bridge::toCvShare(depth_msg)->image;
      if (color.size() != depth.size()) {
        throw std::runtime_error("color and aligned depth image sizes differ");
      }
      if (color.cols != 640 || color.rows != 480) {
        throw std::runtime_error("ORB-SLAM3 expects 640x480 RGB-D images");
      }
      const double stamp = rclcpp::Time(color_msg->header.stamp).seconds();
      const Sophus::SE3f t_camera_world = slam_->TrackRGBD(color, depth, stamp);

      std_msgs::msg::UInt8 state_msg;
      state_msg.data = static_cast<uint8_t>(slam_->GetTrackingState());
      state_pub_->publish(state_msg);
      if (state_msg.data != 2) return;  // Tracking::OK

      const Eigen::Isometry3f t_world_camera(t_camera_world.inverse().matrix());
      if (!have_origin_) {
        t_world_camera_origin_ = t_world_camera;
        have_origin_ = true;
      }
      const Eigen::Isometry3f t_camera0_camera =
          t_world_camera_origin_.inverse() * t_world_camera;
      const Eigen::Isometry3f t_base0_base =
          t_base_camera_optical_ * t_camera0_camera *
          t_base_camera_optical_.inverse();

      const Eigen::Quaternionf q(t_base0_base.rotation());
      nav_msgs::msg::Odometry odom;
      odom.header = color_msg->header;
      odom.header.frame_id = odom_frame_;
      odom.child_frame_id = base_frame_;
      odom.pose.pose.position.x = t_base0_base.translation().x();
      odom.pose.pose.position.y = t_base0_base.translation().y();
      odom.pose.pose.position.z = t_base0_base.translation().z();
      odom.pose.pose.orientation.x = q.x();
      odom.pose.pose.orientation.y = q.y();
      odom.pose.pose.orientation.z = q.z();
      odom.pose.pose.orientation.w = q.w();
      odom_pub_->publish(odom);

      nav_msgs::msg::Odometry odom_2d = odom;
      odom_2d.pose.pose.position.z = 0.0;
      const float yaw = std::atan2(t_base0_base.linear()(1, 0),
                                   t_base0_base.linear()(0, 0));
      odom_2d.pose.pose.orientation.x = 0.0;
      odom_2d.pose.pose.orientation.y = 0.0;
      odom_2d.pose.pose.orientation.z = std::sin(yaw * 0.5f);
      odom_2d.pose.pose.orientation.w = std::cos(yaw * 0.5f);
      odom_2d_pub_->publish(odom_2d);

      if (tf_broadcaster_) {
        geometry_msgs::msg::TransformStamped tf;
        tf.header = odom.header;
        tf.child_frame_id = base_frame_;
        tf.transform.translation.x = odom.pose.pose.position.x;
        tf.transform.translation.y = odom.pose.pose.position.y;
        tf.transform.translation.z = odom.pose.pose.position.z;
        tf.transform.rotation = odom.pose.pose.orientation;
        tf_broadcaster_->sendTransform(tf);
      }
    } catch (const std::exception &e) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                            "RGB-D tracking error: %s", e.what());
    }
  }

  std::unique_ptr<ORB_SLAM3::System> slam_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  message_filters::Subscriber<Image> color_sub_;
  message_filters::Subscriber<Image> depth_sub_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_2d_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr state_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::mutex track_mutex_;
  bool publish_tf_{false};
  bool have_origin_{false};
  bool have_camera_extrinsics_{false};
  std::string odom_frame_;
  std::string base_frame_;
  std::string vocabulary_;
  std::string settings_template_;
  std::string runtime_settings_;
  Eigen::Isometry3f t_world_camera_origin_{Eigen::Isometry3f::Identity()};
  Eigen::Isometry3f t_base_camera_optical_{Eigen::Isometry3f::Identity()};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OrbSlam3OdometryNode>());
  rclcpp::shutdown();
  return 0;
}
