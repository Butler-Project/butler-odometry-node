#include "odometry_node/imu_fusion.hpp"

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <ddsm115_driver/msg/raw_odom.hpp>
#include <imu_driver_node/msg/imu_ekf.hpp>

#include <cmath>
#include <memory>

class OdometryNode : public rclcpp::Node {
public:
  explicit OdometryNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("odometry_node", options) {
    declare_parameter("wheel_radius", 0.048);
    declare_parameter("wheel_base", 0.073);
    declare_parameter("publish_tf", true);
    declare_parameter("use_imu_orientation", true);
    declare_parameter("imu_frame", "imu");
    declare_parameter("base_frame", "base_link");

    wheel_radius_ = get_parameter("wheel_radius").as_double();
    wheel_base_ = get_parameter("wheel_base").as_double();
    publish_tf_ = get_parameter("publish_tf").as_bool();
    use_imu_ = get_parameter("use_imu_orientation").as_bool();
    imu_frame_ = get_parameter("imu_frame").as_string();
    base_frame_ = get_parameter("base_frame").as_string();

    RCLCPP_INFO(get_logger(), "Odometry node started");
    RCLCPP_INFO(get_logger(), "  wheel_radius: %.4f m", wheel_radius_);
    RCLCPP_INFO(get_logger(), "  wheel_base: %.4f m", wheel_base_);
    RCLCPP_INFO(get_logger(), "  publish_tf: %s", publish_tf_ ? "true" : "false");
    RCLCPP_INFO(get_logger(), "  use_imu_orientation: %s", use_imu_ ? "true" : "false");
    RCLCPP_INFO(get_logger(), "  imu_frame: %s", imu_frame_.c_str());

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

    if (publish_tf_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    raw_odom_sub_ = create_subscription<ddsm115_driver::msg::RawOdom>(
      "/raw_odom", 10,
      [this](const ddsm115_driver::msg::RawOdom::SharedPtr msg) { on_raw_odom(msg); });

    if (use_imu_) {
      // TF buffer/listener to look up imu -> base_link static transform
      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
      tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

      // Timer to look up mount transform (retries until found)
      mount_lookup_timer_ = create_wall_timer(
        std::chrono::milliseconds(500),
        [this]() { lookup_mount_transform(); });

      imu_sub_ = create_subscription<imu_driver_node::msg::ImuEkf>(
        "/imu/ekf", 10,
        [this](const imu_driver_node::msg::ImuEkf::SharedPtr msg) { imu_fusion_.update(*msg); });
    }
  }

private:
  void lookup_mount_transform() {
    try {
      auto t = tf_buffer_->lookupTransform(base_frame_, imu_frame_, tf2::TimePointZero);
      auto& q = t.transform.rotation;
      imu_fusion_.set_mount_transform(q.x, q.y, q.z, q.w);
      RCLCPP_INFO(get_logger(), "IMU mount transform acquired (%s -> %s): q=[%.4f, %.4f, %.4f, %.4f]",
                  base_frame_.c_str(), imu_frame_.c_str(), q.x, q.y, q.z, q.w);
      mount_lookup_timer_->cancel();
    } catch (const tf2::TransformException& ex) {
      RCLCPP_DEBUG(get_logger(), "Waiting for %s -> %s transform: %s",
                   base_frame_.c_str(), imu_frame_.c_str(), ex.what());
    }
  }

  void on_raw_odom(const ddsm115_driver::msg::RawOdom::SharedPtr& msg) {
    auto now = this->now();
    double dt = (now - last_time_).seconds();
    last_time_ = now;

    if (dt <= 0.0 || dt > 1.0) return;

    // Wheel velocities from RPM
    double v_left  = (static_cast<double>(msg->left_actual_rpm) * 2.0 * M_PI * wheel_radius_) / 60.0;
    double v_right = (static_cast<double>(msg->right_actual_rpm) * 2.0 * M_PI * wheel_radius_) / 60.0;

    double linear_vel  = (v_left + v_right) / 2.0;
    double angular_vel = (v_right - v_left) / wheel_base_;

    // Use IMU yaw if available, otherwise integrate from wheels
    auto imu_yaw = use_imu_ ? imu_fusion_.yaw() : std::nullopt;
    if (imu_yaw) {
      theta_ = *imu_yaw;
    } else {
      theta_ += angular_vel * dt;
      theta_ = std::remainder(theta_, 2.0 * M_PI);
    }

    // Project wheel velocity onto ground plane using pitch angle.
    // When the robot tilts, wheel rotation partially goes into pitch
    // recovery rather than ground translation. cos(pitch) strips that out.
    auto imu_pitch = use_imu_ ? imu_fusion_.pitch() : std::nullopt;
    double ground_vel = imu_pitch
      ? linear_vel * std::cos(*imu_pitch)
      : linear_vel;

    // Integrate position using pitch-corrected velocity
    x_ += ground_vel * std::cos(theta_) * dt;
    y_ += ground_vel * std::sin(theta_) * dt;

    // Build orientation: full IMU (with tilt) or yaw-only from wheels
    tf2::Quaternion q;
    auto imu_ori = use_imu_ ? imu_fusion_.orientation() : std::nullopt;
    if (imu_ori) {
      q = tf2::Quaternion(imu_ori->x, imu_ori->y, imu_ori->z, imu_ori->w);
    } else {
      q.setRPY(0.0, 0.0, theta_);
    }

    double pub_angular_vel = use_imu_ ? imu_fusion_.angular_velocity_z() : angular_vel;

    // Publish odom message
    // auto odom = std::make_unique<nav_msgs::msg::Odometry>();
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = now;
    odom.header.frame_id = "odom";
    odom.child_frame_id = base_frame_;
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();
    odom.twist.twist.linear.x = linear_vel;
    odom.twist.twist.angular.z = pub_angular_vel;
    odom_pub_->publish(odom);

    // Publish odom -> base_link TF
    if (publish_tf_ && tf_broadcaster_) {
      geometry_msgs::msg::TransformStamped t;
      t.header.stamp = now;
      t.header.frame_id = "odom";
      t.child_frame_id = base_frame_;
      t.transform.translation.x = x_;
      t.transform.translation.y = y_;
      t.transform.translation.z = 0.0;
      t.transform.rotation.x = q.x();
      t.transform.rotation.y = q.y();
      t.transform.rotation.z = q.z();
      t.transform.rotation.w = q.w();
      tf_broadcaster_->sendTransform(t);
    }
  }

  // Parameters
  double wheel_radius_;
  double wheel_base_;
  bool publish_tf_;
  bool use_imu_;
  std::string imu_frame_;
  std::string base_frame_;

  // State
  double x_ = 0.0;
  double y_ = 0.0;
  double theta_ = 0.0;
  rclcpp::Time last_time_{0, 0, RCL_ROS_TIME};

  // Fusion module
  ImuFusion imu_fusion_;

  // ROS interfaces
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<ddsm115_driver::msg::RawOdom>::SharedPtr raw_odom_sub_;
  rclcpp::Subscription<imu_driver_node::msg::ImuEkf>::SharedPtr imu_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // TF lookup for IMU mount transform
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr mount_lookup_timer_;
};

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(OdometryNode)

#ifndef COMPONENT_ONLY
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdometryNode>());
  rclcpp::shutdown();
  return 0;
}
#endif
