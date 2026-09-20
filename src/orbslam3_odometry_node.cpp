#include <chrono>
#include <cmath>
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
#include <std_msgs/msg/u_int8.hpp>
#include <tf2_ros/transform_broadcaster.h>

class OrbSlam3OdometryNode final : public rclcpp::Node {
 public:
  using Image = sensor_msgs::msg::Image;
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<Image, Image>;

  OrbSlam3OdometryNode() : Node("orbslam3_odometry") {
    const auto vocabulary = declare_parameter<std::string>("vocabulary");
    const auto settings = declare_parameter<std::string>("settings");
    publish_tf_ = declare_parameter<bool>("publish_tf", false);
    odom_frame_ = declare_parameter<std::string>("odom_frame", "orb_odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
    slam_ = std::make_unique<ORB_SLAM3::System>(
        vocabulary, settings, ORB_SLAM3::System::RGBD, false);

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    odom_2d_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom_2d", 10);
    state_pub_ = create_publisher<std_msgs::msg::UInt8>("tracking_state", 10);
    if (publish_tf_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    color_sub_.subscribe(this, "/camera/color/image_raw", rmw_qos_profile_sensor_data);
    depth_sub_.subscribe(this, "/camera/aligned_depth_to_color/image_raw",
                         rmw_qos_profile_sensor_data);
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
        SyncPolicy(10), color_sub_, depth_sub_);
    sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.04));
    sync_->registerCallback(std::bind(&OrbSlam3OdometryNode::imageCallback, this,
                                      std::placeholders::_1, std::placeholders::_2));

    // base_link -> camera_link from the Linglong launch file, followed by
    // this D455's factory color extrinsics and ROS optical-frame rotation.
    Eigen::Isometry3f t_base_camera_link = Eigen::Isometry3f::Identity();
    t_base_camera_link.translation() = Eigen::Vector3f(0.035f, 0.039f, 0.24f);
    t_base_camera_link.linear() =
        Eigen::AngleAxisf(0.523f, Eigen::Vector3f::UnitY()).toRotationMatrix();

    Eigen::Isometry3f t_camera_link_color = Eigen::Isometry3f::Identity();
    t_camera_link_color.translation() =
        Eigen::Vector3f(0.0004737293f, -0.0587246418f, -0.0003930985f);
    Eigen::Quaternionf q_link_color(0.9999964833f, 0.0018288863f,
                                    -0.0018871640f, -0.0003029882f);
    t_camera_link_color.linear() = q_link_color.normalized().toRotationMatrix();

    Eigen::Isometry3f t_color_optical = Eigen::Isometry3f::Identity();
    Eigen::Quaternionf q_color_optical(0.5f, -0.5f, 0.5f, -0.5f);
    t_color_optical.linear() = q_color_optical.normalized().toRotationMatrix();
    t_base_camera_optical_ =
        t_base_camera_link * t_camera_link_color * t_color_optical;

    RCLCPP_INFO(get_logger(), "ORB-SLAM3 RGB-D ready; waiting for synchronized images");
  }

  ~OrbSlam3OdometryNode() override {
    if (slam_) slam_->Shutdown();
  }

 private:
  void imageCallback(const Image::ConstSharedPtr &color_msg,
                     const Image::ConstSharedPtr &depth_msg) {
    std::lock_guard<std::mutex> lock(track_mutex_);
    try {
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
  message_filters::Subscriber<Image> color_sub_;
  message_filters::Subscriber<Image> depth_sub_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_2d_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr state_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::mutex track_mutex_;
  bool publish_tf_{false};
  bool have_origin_{false};
  std::string odom_frame_;
  std::string base_frame_;
  Eigen::Isometry3f t_world_camera_origin_{Eigen::Isometry3f::Identity()};
  Eigen::Isometry3f t_base_camera_optical_{Eigen::Isometry3f::Identity()};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OrbSlam3OdometryNode>());
  rclcpp::shutdown();
  return 0;
}
