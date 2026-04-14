#pragma once

#include <imu_driver_node/msg/imu_ekf.hpp>

#include <cmath>
#include <optional>

// Lightweight IMU-wheel fusion: takes IMU EKF orientation, transforms it
// from imu frame to base_link frame using a static mount correction,
// and provides orientation + extracted euler angles for wheel odometry.
// Replaces robot_localization EKF on resource-constrained boards.
class ImuFusion {
public:
  struct Quat {
    double x, y, z, w;
  };

  // Set the static base_link->imu rotation from TF lookup.
  // This is q_base_imu: rotates vectors from imu frame to base_link frame.
  void set_mount_transform(double qx, double qy, double qz, double qw) {
    q_base_imu_ = {qx, qy, qz, qw};
    // Precompute inverse for orientation chaining
    q_imu_base_ = conjugate(q_base_imu_);
    has_mount_ = true;
  }

  void update(const imu_driver_node::msg::ImuEkf& msg) {
    Quat q_world_imu = {
      msg.orientation.x, msg.orientation.y,
      msg.orientation.z, msg.orientation.w
    };

    // Transform orientation: q_world_base = q_world_imu * q_imu_base
    auto q_base = has_mount_
      ? quat_mul(q_world_imu, q_imu_base_)
      : q_world_imu;
    orientation_ = q_base;

    // Transform angular velocity vector from imu frame to base_link:
    // w_base = R_base_imu * w_imu  (rotate vector by q_base_imu)
    if (has_mount_) {
      auto [bx, by, bz] = rotate_vector(q_base_imu_, 0.0, 0.0,
                                          static_cast<double>(msg.angular_velocity_z));
      angular_velocity_z_ = bz;
    } else {
      angular_velocity_z_ = msg.angular_velocity_z;
    }

    // Extract euler angles from base_link orientation
    double qw = q_base.w, qx = q_base.x;
    double qy = q_base.y, qz = q_base.z;

    // Pitch (rotation about Y)
    double sinp = 2.0 * (qw * qy - qz * qx);
    pitch_ = (std::abs(sinp) >= 1.0)
      ? std::copysign(M_PI / 2.0, sinp)
      : std::asin(sinp);

    // Yaw (rotation about Z)
    double siny_cosp = 2.0 * (qw * qz + qx * qy);
    double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    yaw_ = std::atan2(siny_cosp, cosy_cosp);
  }

  [[nodiscard]] auto orientation() const -> std::optional<Quat> {
    return orientation_;
  }

  [[nodiscard]] auto yaw() const -> std::optional<double> {
    if (orientation_) return yaw_;
    return std::nullopt;
  }

  [[nodiscard]] auto pitch() const -> std::optional<double> {
    if (orientation_) return pitch_;
    return std::nullopt;
  }

  [[nodiscard]] auto angular_velocity_z() const -> double {
    return angular_velocity_z_;
  }

  [[nodiscard]] auto has_mount_correction() const -> bool {
    return has_mount_;
  }

private:
  static auto conjugate(const Quat& q) -> Quat {
    return {-q.x, -q.y, -q.z, q.w};
  }

  // Hamilton product: a * b
  static auto quat_mul(const Quat& a, const Quat& b) -> Quat {
    return {
      .x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
      .y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
      .z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
      .w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z
    };
  }

  // Rotate vector by quaternion: v' = q * v * q_conj
  struct Vec3 { double x, y, z; };
  static auto rotate_vector(const Quat& q, double vx, double vy, double vz) -> Vec3 {
    Quat v_quat = {vx, vy, vz, 0.0};
    auto result = quat_mul(quat_mul(q, v_quat), conjugate(q));
    return {result.x, result.y, result.z};
  }

  Quat q_base_imu_{};   // Rotates vectors: imu -> base_link
  Quat q_imu_base_{};   // Inverse: base_link -> imu (for orientation chaining)
  bool has_mount_ = false;

  std::optional<Quat> orientation_;
  double yaw_ = 0.0;
  double pitch_ = 0.0;
  double angular_velocity_z_ = 0.0;
};
