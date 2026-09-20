#include <cmath>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <MapPoint.h>
#include <System.h>
#include <builtin_interfaces/msg/time.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.h>

class OrbSlam3SlamNode final : public rclcpp::Node {
 public:
  using Image = sensor_msgs::msg::Image;
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<Image, Image>;
  using Trigger = std_srvs::srv::Trigger;

  OrbSlam3SlamNode() : Node("orbslam3_slam") {
    const auto vocabulary = declare_parameter<std::string>("vocabulary");
    const auto settings = declare_parameter<std::string>("settings");
    save_directory_ = declare_parameter<std::string>(
        "save_directory", "/tmp/orbslam3_slam");
    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    map_frame_ = declare_parameter<std::string>("map_frame", "orb_map");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
    map_publish_period_ = declare_parameter<double>("map_publish_period", 1.0);
    max_path_poses_ = declare_parameter<int>("max_path_poses", 10000);

    if (map_publish_period_ < 0.0 || max_path_poses_ < 1) {
      throw std::invalid_argument(
          "map_publish_period must be non-negative and max_path_poses positive");
    }

    std::filesystem::create_directories(save_directory_);
    std::filesystem::current_path(save_directory_);
    slam_ = std::make_unique<ORB_SLAM3::System>(
        vocabulary, settings, ORB_SLAM3::System::RGBD, false);

    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("pose", 10);
    path_pub_ = create_publisher<nav_msgs::msg::Path>("path", 10);
    state_pub_ = create_publisher<std_msgs::msg::UInt8>("tracking_state", 10);
    map_points_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "map_points", rclcpp::QoS(1).transient_local().reliable());
    if (publish_tf_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    finish_service_ = create_service<Trigger>(
        "finish_mapping",
        std::bind(&OrbSlam3SlamNode::finishService, this,
                  std::placeholders::_1, std::placeholders::_2));
    reset_service_ = create_service<Trigger>(
        "reset_map",
        std::bind(&OrbSlam3SlamNode::resetService, this,
                  std::placeholders::_1, std::placeholders::_2));

    color_sub_.subscribe(this, "/camera/color/image_raw",
                         rmw_qos_profile_sensor_data);
    depth_sub_.subscribe(this, "/camera/aligned_depth_to_color/image_raw",
                         rmw_qos_profile_sensor_data);
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
        SyncPolicy(10), color_sub_, depth_sub_);
    sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.06));
    sync_->registerCallback(std::bind(&OrbSlam3SlamNode::imageCallback, this,
                                      std::placeholders::_1,
                                      std::placeholders::_2));

    // D455 optical camera pose in base_footprint, matching the existing node.
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

    path_.header.frame_id = map_frame_;
    last_map_publish_time_ = now();
    RCLCPP_INFO(get_logger(),
                "ORB-SLAM3 mapping ready; output directory: %s",
                save_directory_.c_str());
  }

  ~OrbSlam3SlamNode() override { finishMappingNoThrow(); }

 private:
  void imageCallback(const Image::ConstSharedPtr &color_msg,
                     const Image::ConstSharedPtr &depth_msg) {
    std::lock_guard<std::mutex> lock(slam_mutex_);
    if (mapping_finished_) return;

    try {
      cv::Mat color = cv_bridge::toCvShare(color_msg)->image;
      cv::Mat depth = cv_bridge::toCvShare(depth_msg)->image;
      if (color.size() != depth.size()) {
        throw std::runtime_error("color and aligned depth image sizes differ");
      }
      if (color.cols != 640 || color.rows != 480) {
        throw std::runtime_error("ORB-SLAM3 expects 640x480 RGB-D images");
      }

      const double stamp_seconds = rclcpp::Time(color_msg->header.stamp).seconds();
      const Sophus::SE3f t_camera_world =
          slam_->TrackRGBD(color, depth, stamp_seconds);

      std_msgs::msg::UInt8 state_msg;
      state_msg.data = static_cast<uint8_t>(slam_->GetTrackingState());
      state_pub_->publish(state_msg);
      if (state_msg.data != 2) return;

      const Eigen::Isometry3f t_world_camera(t_camera_world.inverse().matrix());
      if (!have_origin_) {
        t_world_camera_origin_ = t_world_camera;
        t_map_world_ = t_base_camera_optical_ * t_world_camera_origin_.inverse();
        have_origin_ = true;
      }
      const Eigen::Isometry3f t_camera0_camera =
          t_world_camera_origin_.inverse() * t_world_camera;
      const Eigen::Isometry3f t_map_base =
          t_base_camera_optical_ * t_camera0_camera *
          t_base_camera_optical_.inverse();

      publishPosePathAndTf(t_map_base, color_msg->header.stamp);
      const auto current_time = now();
      if (map_publish_period_ == 0.0 ||
          (current_time - last_map_publish_time_).seconds() >=
              map_publish_period_) {
        publishMapPoints(color_msg->header.stamp);
        last_map_publish_time_ = current_time;
      }
    } catch (const std::exception &error) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                            "RGB-D SLAM error: %s", error.what());
    }
  }

  void publishPosePathAndTf(const Eigen::Isometry3f &t_map_base,
                            const builtin_interfaces::msg::Time &stamp) {
    const Eigen::Quaternionf q(t_map_base.rotation());
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = stamp;
    pose.header.frame_id = map_frame_;
    pose.pose.position.x = t_map_base.translation().x();
    pose.pose.position.y = t_map_base.translation().y();
    pose.pose.position.z = t_map_base.translation().z();
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();
    pose_pub_->publish(pose);

    path_.header.stamp = stamp;
    path_.poses.push_back(pose);
    if (path_.poses.size() > static_cast<std::size_t>(max_path_poses_)) {
      path_.poses.erase(path_.poses.begin(),
                        path_.poses.begin() +
                            (path_.poses.size() - max_path_poses_));
    }
    path_pub_->publish(path_);

    if (tf_broadcaster_) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header = pose.header;
      transform.child_frame_id = base_frame_;
      transform.transform.translation.x = pose.pose.position.x;
      transform.transform.translation.y = pose.pose.position.y;
      transform.transform.translation.z = pose.pose.position.z;
      transform.transform.rotation = pose.pose.orientation;
      tf_broadcaster_->sendTransform(transform);
    }
  }

  void publishMapPoints(const builtin_interfaces::msg::Time &stamp) {
    std::vector<Eigen::Vector3f> valid_points;
    const auto map_points = slam_->GetTrackedMapPoints();
    valid_points.reserve(map_points.size());
    for (ORB_SLAM3::MapPoint *point : map_points) {
      if (!point || point->isBad()) continue;
      const Eigen::Vector3f p = t_map_world_ * point->GetWorldPos();
      if (std::isfinite(p.x()) && std::isfinite(p.y()) &&
          std::isfinite(p.z())) {
        valid_points.push_back(p);
      }
    }

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = stamp;
    cloud.header.frame_id = map_frame_;
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(valid_points.size());
    sensor_msgs::PointCloud2Iterator<float> x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> z(cloud, "z");
    for (const auto &point : valid_points) {
      *x = point.x();
      *y = point.y();
      *z = point.z();
      ++x;
      ++y;
      ++z;
    }
    cloud.is_dense = false;
    map_points_pub_->publish(cloud);
  }

  void finishService(const std::shared_ptr<Trigger::Request>,
                     std::shared_ptr<Trigger::Response> response) {
    std::lock_guard<std::mutex> lock(slam_mutex_);
    try {
      finishMapping();
      response->success = true;
      response->message = "Atlas and TUM trajectories saved in " +
                          save_directory_;
    } catch (const std::exception &error) {
      response->success = false;
      response->message = error.what();
    }
  }

  void resetService(const std::shared_ptr<Trigger::Request>,
                    std::shared_ptr<Trigger::Response> response) {
    std::lock_guard<std::mutex> lock(slam_mutex_);
    if (mapping_finished_) {
      response->success = false;
      response->message = "mapping has already been finished";
      return;
    }
    slam_->ResetActiveMap();
    have_origin_ = false;
    path_.poses.clear();
    response->success = true;
    response->message = "active ORB-SLAM3 map reset";
  }

  void finishMapping() {
    if (mapping_finished_ || !slam_) return;
    slam_->Shutdown();
    if (have_origin_) {
      slam_->SaveTrajectoryTUM(
          (std::filesystem::path(save_directory_) / "CameraTrajectory.txt").string());
      slam_->SaveKeyFrameTrajectoryTUM(
          (std::filesystem::path(save_directory_) /
           "KeyFrameTrajectory.txt").string());
    }
    mapping_finished_ = true;
  }

  void finishMappingNoThrow() noexcept {
    try {
      std::lock_guard<std::mutex> lock(slam_mutex_);
      finishMapping();
    } catch (const std::exception &error) {
      RCLCPP_ERROR(get_logger(), "failed to save SLAM result: %s", error.what());
    }
  }

  std::unique_ptr<ORB_SLAM3::System> slam_;
  message_filters::Subscriber<Image> color_sub_;
  message_filters::Subscriber<Image> depth_sub_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr state_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_points_pub_;
  rclcpp::Service<Trigger>::SharedPtr finish_service_;
  rclcpp::Service<Trigger>::SharedPtr reset_service_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::mutex slam_mutex_;
  nav_msgs::msg::Path path_;
  bool publish_tf_{true};
  bool mapping_finished_{false};
  bool have_origin_{false};
  int max_path_poses_{10000};
  double map_publish_period_{1.0};
  std::string map_frame_;
  std::string base_frame_;
  std::string save_directory_;
  rclcpp::Time last_map_publish_time_{0, 0, RCL_ROS_TIME};
  Eigen::Isometry3f t_world_camera_origin_{Eigen::Isometry3f::Identity()};
  Eigen::Isometry3f t_map_world_{Eigen::Isometry3f::Identity()};
  Eigen::Isometry3f t_base_camera_optical_{Eigen::Isometry3f::Identity()};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OrbSlam3SlamNode>());
  rclcpp::shutdown();
  return 0;
}
