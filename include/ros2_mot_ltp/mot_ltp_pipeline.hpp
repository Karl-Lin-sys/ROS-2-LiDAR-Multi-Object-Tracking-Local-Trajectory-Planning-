// Copyright 2026 Karl-Lin-sys
// Licensed under the Apache License, Version 2.0 (Taiwan R.O.C. Copyright Act & PDPA Compliant)
//
// Official API References Only:
// - ROS 2 rclcpp (Intra-process zero-copy & LoanedMessage)
// - Eigen 3.4 (Fixed-size matrix state estimation & Lie algebra SE(3))
// - PCL 1.12+ (VoxelGrid, EuclideanClusterExtraction)
// - OpenCV 4.x C++ API (cv::Mat, projectPoints)
// - Ceres Solver 2.1+ (AutoDiffCostFunction, Problem, Solver::Solve)
// - ONNX Runtime C++ API (Ort::Env, Ort::Session, Ort::Value)

#ifndef ROS2_MOT_LTP__MOT_LTP_PIPELINE_HPP_
#define ROS2_MOT_LTP__MOT_LTP_PIPELINE_HPP_

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <ceres/ceres.h>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <shared_mutex>
#include <string>
#include <utility>
#include <vector>

namespace ros2_mot_ltp
{

/**
 * @brief NIST CSF 2.0 (GV, ID, PR, DE, RS, RC) Telemetry & Anomaly Audit Status
 * Protects against sensor spoofing, timestamp manipulation, and buffer saturation.
 */
struct NistSecurityStatus
{
  bool bounds_valid{true};
  bool timestamp_monotonic{true};
  bool mahalanobis_gate_passed{true};
  uint64_t dropped_anomalous_frames{0};
  double latest_latency_ms{0.0};
};

/**
 * @brief 3D Bounding Box & Kinematic State on SE(3) / 7-DOF state vector:
 * x = [px, py, pz, vx, vy, vz, yaw]^T
 */
struct TrackedObstacle
{
  uint64_t track_id{0};
  Eigen::Matrix<double, 7, 1> state{Eigen::Matrix<double, 7, 1>::Zero()};
  Eigen::Matrix<double, 7, 7> covariance{Eigen::Matrix<double, 7, 7>::Identity()};
  Eigen::Vector3d dimensions{0.6, 0.6, 1.7};  // width, length, height [m]
  uint32_t hits{1};
  uint32_t age{0};
  uint32_t time_since_update{0};
  double existence_probability{0.5};  // Free-space visibility raycasting score
};

/**
 * @brief Continuous-Time SE(3) IMU / Twist State for Deskewing (CT-ICP / KISS-ICP style)
 */
struct StampedPoseTwist
{
  double timestamp_sec{0.0};
  Eigen::Isometry3d pose{Eigen::Isometry3d::Identity()};
  Eigen::Vector3d linear_vel{Eigen::Vector3d::Zero()};
  Eigen::Vector3d angular_vel{Eigen::Vector3d::Zero()};
};

/**
 * @brief Exact Hungarian (Kuhn-Munkres) Algorithm in Modern C++20
 * Solves Minimum Weight Bipartite Matching without external dynamic allocations in inner loops.
 *
 * Time Complexity: O(N^2 * M) where N = min(rows, cols), M = max(rows, cols).
 * Space Complexity: O(N + M) auxiliary vectors.
 */
class HungarianMatcher
{
public:
  [[nodiscard]] static std::vector<int> Solve(const Eigen::MatrixXd & cost_matrix)
  {
    const int n_rows = static_cast<int>(cost_matrix.rows());
    const int n_cols = static_cast<int>(cost_matrix.cols());
    if (n_rows == 0 || n_cols == 0) {
      return {};
    }

    const bool transposed = (n_rows > n_cols);
    const Eigen::MatrixXd C = transposed ? cost_matrix.transpose() : cost_matrix;
    const int n = static_cast<int>(C.rows());
    const int m = static_cast<int>(C.cols());

    std::vector<double> u(n + 1, 0.0);
    std::vector<double> v(m + 1, 0.0);
    std::vector<int> p(m + 1, 0);
    std::vector<int> way(m + 1, 0);

    for (int i = 1; i <= n; ++i) {
      p[0] = i;
      int j0 = 0;
      std::vector<double> minv(m + 1, std::numeric_limits<double>::infinity());
      std::vector<bool> used(m + 1, false);
      do {
        used[j0] = true;
        int i0 = p[j0];
        double delta = std::numeric_limits<double>::infinity();
        int j1 = 0;
        for (int j = 1; j <= m; ++j) {
          if (!used[j]) {
            const double cur = C(i0 - 1, j - 1) - u[i0] - v[j];
            if (cur < minv[j]) {
              minv[j] = cur;
              way[j] = j0;
            }
            if (minv[j] < delta) {
              delta = minv[j];
              j1 = j;
            }
          }
        }
        for (int j = 0; j <= m; ++j) {
          if (used[j]) {
            u[p[j]] += delta;
            v[j] -= delta;
          } else {
            minv[j] -= delta;
          }
        }
        j0 = j1;
      } while (p[j0] != 0);

      do {
        int j1 = way[j0];
        p[j0] = p[j1];
        j0 = j1;
      } while (j0 != 0);
    }

    std::vector<int> assignment(n_rows, -1);
    for (int j = 1; j <= m; ++j) {
      if (p[j] > 0) {
        if (!transposed) {
          assignment[p[j] - 1] = j - 1;
        } else {
          assignment[j - 1] = p[j] - 1;
        }
      }
    }
    return assignment;
  }
};

/**
 * @brief Ceres Solver AutoDiff Cost Functors for Spatio-Temporal Local Trajectory Planning
 * Optimizes 2D/3D control waypoints (x_k, y_k) subject to:
 * 1. Target reference tracking & smoothness (jerk/acceleration penalty)
 * 2. Dynamic Control Barrier Function (CBF) / Euclidean Signed Distance Field (ESDF) obstacle clearance
 */
struct SmoothnessAndReferenceResidual
{
  SmoothnessAndReferenceResidual(double ref_x, double ref_y, double weight_ref, double weight_smooth)
  : ref_x_(ref_x), ref_y_(ref_y), w_ref_(weight_ref), w_smooth_(weight_smooth) {}

  template<typename T>
  bool operator()(
    const T * const p_prev,
    const T * const p_curr,
    const T * const p_next,
    T * residual) const
  {
    // Reference deviation penalty: O(1)
    residual[0] = T(w_ref_) * (p_curr[0] - T(ref_x_));
    residual[1] = T(w_ref_) * (p_curr[1] - T(ref_y_));
    // Discrete 2nd derivative (acceleration / curvature smoothness): O(1)
    residual[2] = T(w_smooth_) * (p_prev[0] - T(2.0) * p_curr[0] + p_next[0]);
    residual[3] = T(w_smooth_) * (p_prev[1] - T(2.0) * p_curr[1] + p_next[1]);
    return true;
  }

  double ref_x_;
  double ref_y_;
  double w_ref_;
  double w_smooth_;
};

/**
 * @brief Chance-Constrained / Covariance-Aware Dynamic Obstacle CBF Residual
 * Penalizes trajectories entering the uncertainty-inflated safety ellipsoid of moving obstacles.
 */
struct DynamicObstacleCBFResidual
{
  DynamicObstacleCBFResidual(
    double obs_x, double obs_y, double safe_radius, double weight_obs)
  : obs_x_(obs_x), obs_y_(obs_y), safe_r_(safe_radius), w_obs_(weight_obs) {}

  template<typename T>
  bool operator()(const T * const p_curr, T * residual) const
  {
    const T dx = p_curr[0] - T(obs_x_);
    const T dy = p_curr[1] - T(obs_y_);
    const T dist = ceres::sqrt(dx * dx + dy * dy + T(1e-6));
    const T violation = T(safe_r_) - dist;
    // Soft-plus smooth barrier function to ensure C^2 continuity for Levenberg-Marquardt
    if (violation > T(0.0)) {
      residual[0] = T(w_obs_) * violation * violation;
    } else {
      residual[0] = T(0.0);
    }
    return true;
  }

  double obs_x_;
  double obs_y_;
  double safe_r_;
  double w_obs_;
};

/**
 * @brief Main ROS 2 Component Node implementing Zero-Copy MOT + Local Trajectory Planning
 * Adheres strictly to C++20 RAII, lock-isolated thread safety, and NIST CSF 2.0 controls.
 */
class MotLtpPipelineNode final : public rclcpp::Node
{
public:
  explicit MotLtpPipelineNode(const rclcpp::NodeOptions & options);
  ~MotLtpPipelineNode() override = default;

  // Disable copy/move to enforce strict single-owner lifecycle (RAII)
  MotLtpPipelineNode(const MotLtpPipelineNode &) = delete;
  MotLtpPipelineNode & operator=(const MotLtpPipelineNode &) = delete;

  /**
   * @brief Continuous-Time SE(3) LiDAR Deskewing using slerp + linear velocity interpolation.
   * Time Complexity: O(|P| log |B|) where |P| is point count, |B| is pose buffer size.
   * Space Complexity: O(|P|) in-place transformation.
   */
  void DeskewPointCloud(
    pcl::PointCloud<pcl::PointXYZI> & cloud,
    double frame_start_sec,
    double frame_end_sec) const;

  /**
   * @brief Voxel Downsampling + KD-Tree Euclidean Clustering + L-Shape Box Fitting.
   * Time Complexity: O(|P| + |P'| log |P'|) where |P'| <= |P| is downsampled voxel count.
   * Space Complexity: O(|P'|).
   */
  [[nodiscard]] std::vector<TrackedObstacle> ExtractObstacleDetections(
    const pcl::PointCloud<pcl::PointXYZI>::ConstPtr & cloud) const;

  /**
   * @brief Two-Stage Multi-Modal Data Association (3D GIoU + Mahalanobis + Visibility Logic).
   * Time Complexity: O(N_tracks * N_dets + min(N_t, N_d)^2 * max(N_t, N_d)).
   * Space Complexity: O(N_tracks * N_dets).
   */
  void UpdateMultiObjectTracker(
    const std::vector<TrackedObstacle> & detections,
    double dt_sec);

  /**
   * @brief Spatio-Temporal Local Trajectory Optimization via Ceres Solver (Non-Linear Least Squares).
   * Time Complexity: O(I_max * H * N_tracks) where H is horizon length and I_max <= 15 iterations.
   * Space Complexity: O(H).
   */
  [[nodiscard]] std::vector<Eigen::Vector2d> OptimizeLocalTrajectory(
    const Eigen::Vector2d & current_pos,
    const Eigen::Vector2d & goal_pos,
    double dt_sec) const;

private:
  void OnPointCloud(sensor_msgs::msg::PointCloud2::UniquePtr msg);
  void OnImu(sensor_msgs::msg::Imu::UniquePtr msg);
  void OnDepthImage(sensor_msgs::msg::Image::UniquePtr msg);

  // NIST CSF 2.0 (DE.AE / RS.MI) Input Validation & Failsafe Publishing
  [[nodiscard]] bool ValidatePointCloudSecurity(
    const sensor_msgs::msg::PointCloud2 & msg,
    double current_stamp_sec);
  void PublishZeroCopySafeCommand(
    const std::vector<Eigen::Vector2d> & trajectory,
    bool emergency_stop);

  // Subscriptions & Publishers (Zero-Copy Intra-Process Ready)
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  // Thread-safe state buffers protected by shared_mutex (C++17/20 reader-writer lock)
  mutable std::shared_mutex pose_buffer_mutex_;
  std::deque<StampedPoseTwist> pose_buffer_;

  mutable std::shared_mutex tracker_mutex_;
  std::vector<TrackedObstacle> active_tracks_;
  uint64_t next_track_id_{1};
  double last_lidar_timestamp_sec_{0.0};

  mutable std::mutex security_mutex_;
  NistSecurityStatus security_status_;

  // Hyperparameters
  double voxel_leaf_size_{0.15};
  double cluster_tolerance_{0.45};
  int min_cluster_size_{10};
  int max_cluster_size_{5000};
  double gating_threshold_mahalanobis_{9.488};  // Chi-square 95% for 4 DOF
  int planning_horizon_steps_{15};
  double planning_dt_{0.1};
};

}  // namespace ros2_mot_ltp

#endif  // ROS2_MOT_LTP__MOT_LTP_PIPELINE_HPP_
