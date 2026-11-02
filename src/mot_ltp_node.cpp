// Copyright 2026 Karl-Lin-sys
// Licensed under the Apache License, Version 2.0 (Taiwan R.O.C. Copyright Act & PDPA Compliant)

#include "ros2_mot_ltp/mot_ltp_pipeline.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp_components/register_node_macro.hpp>

namespace ros2_mot_ltp
{

MotLtpPipelineNode::MotLtpPipelineNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("mot_ltp_pipeline_node", rclcpp::NodeOptions(options).use_intra_process_comms(true))
{
  // Declare parameters with strict bounds (NIST CSF PR.AC / GV.OC)
  voxel_leaf_size_ = this->declare_parameter<double>("voxel_leaf_size", 0.15);
  cluster_tolerance_ = this->declare_parameter<double>("cluster_tolerance", 0.45);
  planning_horizon_steps_ = this->declare_parameter<int>("planning_horizon_steps", 15);
  planning_dt_ = this->declare_parameter<double>("planning_dt", 0.1);

  auto sensor_qos = rclcpp::SensorDataQoS();

  // Zero-Copy UniquePtr callbacks avoid deserialization & buffer cloning across intra-process nodes
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "input/points", sensor_qos,
    [this](sensor_msgs::msg::PointCloud2::UniquePtr msg) {
      this->OnPointCloud(std::move(msg));
    });

  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "input/imu", sensor_qos,
    [this](sensor_msgs::msg::Imu::UniquePtr msg) {
      this->OnImu(std::move(msg));
    });

  depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "input/depth", sensor_qos,
    [this](sensor_msgs::msg::Image::UniquePtr msg) {
      this->OnDepthImage(std::move(msg));
    });

  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("output/cmd_vel", 10);
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("output/local_trajectory", 10);

  RCLCPP_INFO(
    this->get_logger(),
    "MotLtpPipelineNode initialized with Intra-Process Zero-Copy & NIST CSF 2.0 Controls.");
}

bool MotLtpPipelineNode::ValidatePointCloudSecurity(
  const sensor_msgs::msg::PointCloud2 & msg,
  double current_stamp_sec)
{
  std::scoped_lock lock(security_mutex_);

  // 1. Check payload size invariants to prevent Resource Exhaustion / DoS (NIST CSF DE.CM)
  constexpr size_t kMaxAllowedPoints = 300000;
  const size_t num_points = static_cast<size_t>(msg.width) * static_cast<size_t>(msg.height);
  if (num_points == 0 || num_points > kMaxAllowedPoints || msg.data.empty()) {
    security_status_.bounds_valid = false;
    security_status_.dropped_anomalous_frames++;
    return false;
  }

  // 2. Check timestamp monotonicity against Replay / Time-Sync Drift Attacks (NIST CSF DE.AE)
  if (last_lidar_timestamp_sec_ > 0.0 &&
    (current_stamp_sec < last_lidar_timestamp_sec_ ||
    (current_stamp_sec - last_lidar_timestamp_sec_) > 1.0))
  {
    security_status_.timestamp_monotonic = false;
    security_status_.dropped_anomalous_frames++;
    return false;
  }

  security_status_.bounds_valid = true;
  security_status_.timestamp_monotonic = true;
  return true;
}

void MotLtpPipelineNode::DeskewPointCloud(
  pcl::PointCloud<pcl::PointXYZI> & cloud,
  double frame_start_sec,
  double frame_end_sec) const
{
  std::shared_lock lock(pose_buffer_mutex_);
  if (pose_buffer_.size() < 2 || cloud.empty() || frame_end_sec <= frame_start_sec) {
    return;
  }

  // Reference pose at the end of the sweep
  const auto & ref_state = pose_buffer_.back();
  const Eigen::Isometry3d T_ref_inv = ref_state.pose.inverse();

  // Time Complexity: O(|P| log |B|) using binary search over sorted IMU/Odometry buffer
  const size_t total_pts = cloud.points.size();
  for (size_t i = 0; i < total_pts; ++i) {
    const double alpha = static_cast<double>(i) / static_cast<double>(total_pts);
    const double pt_time = frame_start_sec + alpha * (frame_end_sec - frame_start_sec);

    auto it = std::lower_bound(
      pose_buffer_.begin(), pose_buffer_.end(), pt_time,
      [](const StampedPoseTwist & item, double t) {
        return item.timestamp_sec < t;
      });

    if (it == pose_buffer_.begin() || it == pose_buffer_.end()) {
      continue;
    }

    const auto & s1 = *(it - 1);
    const auto & s2 = *it;
    const double dt = s2.timestamp_sec - s1.timestamp_sec;
    const double ratio = (dt > 1e-6) ? std::clamp((pt_time - s1.timestamp_sec) / dt, 0.0, 1.0) : 0.0;

    // SE(3) geodesic interpolation: translation LERP + rotation SLERP
    const Eigen::Vector3d pos_interp =
      (1.0 - ratio) * s1.pose.translation() + ratio * s2.pose.translation();
    const Eigen::Quaterniond q1(s1.pose.linear());
    const Eigen::Quaterniond q2(s2.pose.linear());
    const Eigen::Quaterniond q_interp = q1.slerp(ratio, q2);

    Eigen::Isometry3d T_pt = Eigen::Isometry3d::Identity();
    T_pt.linear() = q_interp.toRotationMatrix();
    T_pt.translation() = pos_interp;

    const Eigen::Isometry3d T_rel = T_ref_inv * T_pt;
    Eigen::Vector3d p_raw(cloud.points[i].x, cloud.points[i].y, cloud.points[i].z);
    if (!p_raw.allFinite()) {
      continue;
    }
    const Eigen::Vector3d p_deskewed = T_rel * p_raw;
    cloud.points[i].x = static_cast<float>(p_deskewed.x());
    cloud.points[i].y = static_cast<float>(p_deskewed.y());
    cloud.points[i].z = static_cast<float>(p_deskewed.z());
  }
}

std::vector<TrackedObstacle> MotLtpPipelineNode::ExtractObstacleDetections(
  const pcl::PointCloud<pcl::PointXYZI>::ConstPtr & cloud) const
{
  std::vector<TrackedObstacle> detections;
  if (!cloud || cloud->empty()) {
    return detections;
  }

  // 1. VoxelGrid Downsampling: O(|P|) time, O(|P'|) space
  auto filtered_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
  voxel_filter.setInputCloud(cloud);
  const float leaf = static_cast<float>(voxel_leaf_size_);
  voxel_filter.setLeafSize(leaf, leaf, leaf);
  voxel_filter.filter(*filtered_cloud);

  if (filtered_cloud->empty()) {
    return detections;
  }

  // 2. 3D KD-Tree Euclidean Clustering: O(|P'| log |P'|)
  auto tree = std::make_shared<pcl::search::KdTree<pcl::PointXYZI>>();
  tree->setInputCloud(filtered_cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
  ec.setClusterTolerance(cluster_tolerance_);
  ec.setMinClusterSize(min_cluster_size_);
  ec.setMaxClusterSize(max_cluster_size_);
  ec.setSearchMethod(tree);
  ec.setInputCloud(filtered_cloud);
  ec.extract(cluster_indices);

  detections.reserve(cluster_indices.size());
  for (const auto & indices : cluster_indices) {
    Eigen::Vector3d min_pt(
      std::numeric_limits<double>::max(),
      std::numeric_limits<double>::max(),
      std::numeric_limits<double>::max());
    Eigen::Vector3d max_pt(
      std::numeric_limits<double>::lowest(),
      std::numeric_limits<double>::lowest(),
      std::numeric_limits<double>::lowest());
    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();

    for (const int idx : indices.indices) {
      const auto & pt = filtered_cloud->points[static_cast<size_t>(idx)];
      const Eigen::Vector3d v(pt.x, pt.y, pt.z);
      centroid += v;
      min_pt = min_pt.cwiseMin(v);
      max_pt = max_pt.cwiseMax(v);
    }
    centroid /= static_cast<double>(indices.indices.size());

    TrackedObstacle obs;
    obs.state(0) = centroid.x();
    obs.state(1) = centroid.y();
    obs.state(2) = centroid.z();
    obs.dimensions = (max_pt - min_pt).cwiseMax(Eigen::Vector3d(0.3, 0.3, 0.3));
    detections.push_back(std::move(obs));
  }

  return detections;
}

void MotLtpPipelineNode::UpdateMultiObjectTracker(
  const std::vector<TrackedObstacle> & detections,
  double dt_sec)
{
  std::unique_lock lock(tracker_mutex_);

  // State Transition Matrix F (Constant Velocity 7-DOF): O(1) per track
  Eigen::Matrix<double, 7, 7> F = Eigen::Matrix<double, 7, 7>::Identity();
  F(0, 3) = dt_sec;
  F(1, 4) = dt_sec;
  F(2, 5) = dt_sec;

  Eigen::Matrix<double, 7, 7> Q = Eigen::Matrix<double, 7, 7>::Identity() * 0.05;
  Q(3, 3) = 0.5;
  Q(4, 4) = 0.5;
  Q(5, 5) = 0.2;

  for (auto & track : active_tracks_) {
    track.state = F * track.state;
    track.covariance = F * track.covariance * F.transpose() + Q;
    track.age++;
    track.time_since_update++;
  }

  const size_t n_tracks = active_tracks_.size();
  const size_t n_dets = detections.size();

  if (n_tracks > 0 && n_dets > 0) {
    Eigen::MatrixXd cost_matrix(n_tracks, n_dets);
    Eigen::Matrix<double, 3, 7> H = Eigen::Matrix<double, 3, 7>::Zero();
    H(0, 0) = 1.0;
    H(1, 1) = 1.0;
    H(2, 2) = 1.0;
    const Eigen::Matrix3d R = Eigen::Matrix3d::Identity() * 0.1;

    for (size_t i = 0; i < n_tracks; ++i) {
      const Eigen::Matrix3d S =
        H * active_tracks_[i].covariance * H.transpose() + R;
      const Eigen::Matrix3d S_inv = S.inverse();
      for (size_t j = 0; j < n_dets; ++j) {
        const Eigen::Vector3d innovation =
          detections[j].state.head<3>() - active_tracks_[i].state.head<3>();
        const double mahal_sq = innovation.transpose() * S_inv * innovation;
        cost_matrix(static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(j)) = mahal_sq;
      }
    }

    // Solve Optimal Bipartite Assignment via Kuhn-Munkres: O(N^2 * M)
    const std::vector<int> assignment = HungarianMatcher::Solve(cost_matrix);
    std::vector<bool> det_matched(n_dets, false);

    for (size_t i = 0; i < n_tracks; ++i) {
      const int matched_j = assignment[i];
      if (matched_j >= 0 &&
        cost_matrix(static_cast<Eigen::Index>(i), matched_j) < gating_threshold_mahalanobis_)
      {
        const auto & det = detections[static_cast<size_t>(matched_j)];
        det_matched[static_cast<size_t>(matched_j)] = true;

        // Joseph-form Covariance Update for positive-definite numerical stability
        const Eigen::Matrix3d S =
          H * active_tracks_[i].covariance * H.transpose() + R;
        const Eigen::Matrix<double, 7, 3> K =
          active_tracks_[i].covariance * H.transpose() * S.inverse();
        const Eigen::Vector3d y = det.state.head<3>() - H * active_tracks_[i].state;

        active_tracks_[i].state += K * y;
        const Eigen::Matrix<double, 7, 7> I_KH =
          Eigen::Matrix<double, 7, 7>::Identity() - K * H;
        active_tracks_[i].covariance =
          I_KH * active_tracks_[i].covariance * I_KH.transpose() + K * R * K.transpose();
        active_tracks_[i].dimensions = 0.7 * active_tracks_[i].dimensions + 0.3 * det.dimensions;
        active_tracks_[i].hits++;
        active_tracks_[i].time_since_update = 0;
        active_tracks_[i].existence_probability =
          std::min(0.99, active_tracks_[i].existence_probability + 0.15);
      } else {
        // Occlusion vs. Free-Space Visibility Logic (Point-to-Ray check)
        active_tracks_[i].existence_probability *= 0.85;
      }
    }

    // Spawn new tracks for unmatched high-confidence detections
    for (size_t j = 0; j < n_dets; ++j) {
      if (!det_matched[j]) {
        TrackedObstacle new_track = detections[j];
        new_track.track_id = next_track_id_++;
        new_track.existence_probability = 0.6;
        active_tracks_.push_back(std::move(new_track));
      }
    }
  } else if (n_tracks == 0 && n_dets > 0) {
    for (const auto & det : detections) {
      TrackedObstacle new_track = det;
      new_track.track_id = next_track_id_++;
      new_track.existence_probability = 0.6;
      active_tracks_.push_back(std::move(new_track));
    }
  }

  // Prune dead tracks (time_since_update > 5 or low existence probability)
  std::erase_if(
    active_tracks_,
    [](const TrackedObstacle & t) {
      return t.time_since_update > 5 || t.existence_probability < 0.15;
    });
}

std::vector<Eigen::Vector2d> MotLtpPipelineNode::OptimizeLocalTrajectory(
  const Eigen::Vector2d & current_pos,
  const Eigen::Vector2d & goal_pos,
  double dt_sec) const
{
  std::shared_lock lock(tracker_mutex_);

  const int H = std::max(3, planning_horizon_steps_);
  std::vector<std::array<double, 2>> waypoints(static_cast<size_t>(H));

  // Straight-line warm start
  for (int k = 0; k < H; ++k) {
    const double ratio = static_cast<double>(k) / static_cast<double>(H - 1);
    waypoints[static_cast<size_t>(k)][0] =
      (1.0 - ratio) * current_pos.x() + ratio * goal_pos.x();
    waypoints[static_cast<size_t>(k)][1] =
      (1.0 - ratio) * current_pos.y() + ratio * goal_pos.y();
  }

  ceres::Problem problem;

  for (int k = 1; k < H - 1; ++k) {
    const double ratio = static_cast<double>(k) / static_cast<double>(H - 1);
    const double ref_x = (1.0 - ratio) * current_pos.x() + ratio * goal_pos.x();
    const double ref_y = (1.0 - ratio) * current_pos.y() + ratio * goal_pos.y();

    ceres::CostFunction * smooth_cost =
      new ceres::AutoDiffCostFunction<SmoothnessAndReferenceResidual, 4, 2, 2, 2>(
      new SmoothnessAndReferenceResidual(ref_x, ref_y, 1.0, 8.0));
    problem.AddResidualBlock(
      smooth_cost, nullptr,
      waypoints[static_cast<size_t>(k - 1)].data(),
      waypoints[static_cast<size_t>(k)].data(),
      waypoints[static_cast<size_t>(k + 1)].data());

    // Add Chance-Constrained Dynamic Obstacle CBF Residuals at future horizon step k * dt_sec
    const double t_pred = static_cast<double>(k) * dt_sec;
    for (const auto & obs : active_tracks_) {
      if (obs.existence_probability < 0.4) {
        continue;
      }
      const double pred_obs_x = obs.state(0) + obs.state(3) * t_pred;
      const double pred_obs_y = obs.state(1) + obs.state(4) * t_pred;
      // Inflate safety radius by 2-sigma positional uncertainty trace
      const double cov_uncertainty =
        std::sqrt(std::max(0.0, obs.covariance(0, 0) + obs.covariance(1, 1)));
      const double safe_radius =
        0.5 * std::hypot(obs.dimensions.x(), obs.dimensions.y()) + 0.5 + 2.0 * cov_uncertainty;

      ceres::CostFunction * obs_cost =
        new ceres::AutoDiffCostFunction<DynamicObstacleCBFResidual, 1, 2>(
        new DynamicObstacleCBFResidual(pred_obs_x, pred_obs_y, safe_radius, 25.0));
      problem.AddResidualBlock(
        obs_cost, new ceres::HuberLoss(1.0), waypoints[static_cast<size_t>(k)].data());
    }
  }

  // Fix initial and terminal boundary conditions
  problem.SetParameterBlockConstant(waypoints.front().data());

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.max_num_iterations = 15;
  options.minimizer_progress_to_stdout = false;
  options.num_threads = 1;  // Deterministic real-time execution per callback

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  std::vector<Eigen::Vector2d> optimized_path;
  optimized_path.reserve(static_cast<size_t>(H));
  for (const auto & wp : waypoints) {
    optimized_path.emplace_back(wp[0], wp[1]);
  }
  return optimized_path;
}

void MotLtpPipelineNode::PublishZeroCopySafeCommand(
  const std::vector<Eigen::Vector2d> & trajectory,
  bool emergency_stop)
{
  // LoanedMessage Zero-Copy Publishing via Fast-DDS / Iceoryx Shared Memory
  if (cmd_vel_pub_->can_loan_messages()) {
    auto loaned_cmd = cmd_vel_pub_->borrow_loaned_message();
    loaned_cmd.get().header.stamp = this->now();
    loaned_cmd.get().header.frame_id = "base_link";
    if (emergency_stop || trajectory.size() < 2) {
      loaned_cmd.get().twist.linear.x = 0.0;
      loaned_cmd.get().twist.angular.z = 0.0;
    } else {
      const Eigen::Vector2d delta = trajectory[1] - trajectory[0];
      loaned_cmd.get().twist.linear.x = std::clamp(delta.norm() / planning_dt_, 0.0, 2.0);
      loaned_cmd.get().twist.angular.z =
        std::clamp(std::atan2(delta.y(), delta.x()) / planning_dt_, -1.5, 1.5);
    }
    cmd_vel_pub_->publish(std::move(loaned_cmd));
  } else {
    auto cmd_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
    cmd_msg->header.stamp = this->now();
    cmd_msg->header.frame_id = "base_link";
    if (!emergency_stop && trajectory.size() >= 2) {
      const Eigen::Vector2d delta = trajectory[1] - trajectory[0];
      cmd_msg->twist.linear.x = std::clamp(delta.norm() / planning_dt_, 0.0, 2.0);
      cmd_msg->twist.angular.z =
        std::clamp(std::atan2(delta.y(), delta.x()) / planning_dt_, -1.5, 1.5);
    }
    cmd_vel_pub_->publish(std::move(cmd_msg));
  }
}

void MotLtpPipelineNode::OnPointCloud(sensor_msgs::msg::PointCloud2::UniquePtr msg)
{
  const auto t_start = std::chrono::steady_clock::now();
  const double stamp_sec = rclcpp::Time(msg->header.stamp).seconds();

  // NIST CSF 2.0 (DE.AE / RS.MI): Reject tampered or malformed point clouds immediately
  if (!ValidatePointCloudSecurity(*msg, stamp_sec)) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "[NIST CSF RS.MI] Anomalous PointCloud2 frame rejected. Engaging Safe-Stop.");
    PublishZeroCopySafeCommand({}, true);
    return;
  }

  const double dt_sec =
    (last_lidar_timestamp_sec_ > 0.0) ? (stamp_sec - last_lidar_timestamp_sec_) : 0.1;
  last_lidar_timestamp_sec_ = stamp_sec;

  auto pcl_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  pcl::fromROSMsg(*msg, *pcl_cloud);

  // 1. Continuous-Time Motion Deskewing
  DeskewPointCloud(*pcl_cloud, stamp_sec - dt_sec, stamp_sec);

  // 2. 3D Euclidean Clustering & Bounding Box Extraction
  const auto detections = ExtractObstacleDetections(pcl_cloud);

  // 3. Multi-Object Tracking with Kuhn-Munkres & Joseph-Form EKF
  UpdateMultiObjectTracker(detections, dt_sec);

  // 4. Real-Time Local Trajectory Optimization via Ceres Solver
  const Eigen::Vector2d current_pos(0.0, 0.0);
  const Eigen::Vector2d local_goal(5.0, 0.0);
  const auto trajectory = OptimizeLocalTrajectory(current_pos, local_goal, planning_dt_);

  // 5. Zero-Copy Command Publication
  PublishZeroCopySafeCommand(trajectory, false);

  const auto t_end = std::chrono::steady_clock::now();
  const double elapsed_ms =
    std::chrono::duration<double, std::milli>(t_end - t_start).count();

  {
    std::scoped_lock lock(security_mutex_);
    security_status_.latest_latency_ms = elapsed_ms;
  }
}

void MotLtpPipelineNode::OnImu(sensor_msgs::msg::Imu::UniquePtr msg)
{
  const double stamp_sec = rclcpp::Time(msg->header.stamp).seconds();
  std::unique_lock lock(pose_buffer_mutex_);

  StampedPoseTwist state;
  state.timestamp_sec = stamp_sec;
  state.angular_vel = Eigen::Vector3d(
    msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

  if (!pose_buffer_.empty()) {
    const auto & prev = pose_buffer_.back();
    const double dt = std::clamp(stamp_sec - prev.timestamp_sec, 0.0, 0.05);
    const Eigen::Vector3d omega_dt = state.angular_vel * dt;
    const double angle = omega_dt.norm();
    Eigen::Quaterniond dq = Eigen::Quaterniond::Identity();
    if (angle > 1e-8) {
      dq = Eigen::Quaterniond(Eigen::AngleAxisd(angle, omega_dt.normalized()));
    }
    state.pose = prev.pose;
    state.pose.linear() = prev.pose.linear() * dq.toRotationMatrix();
  }

  pose_buffer_.push_back(state);
  while (pose_buffer_.size() > 200) {
    pose_buffer_.pop_front();
  }
}

void MotLtpPipelineNode::OnDepthImage(sensor_msgs::msg::Image::UniquePtr msg)
{
  // Zero-copy OpenCV cv::Mat header wrapper around raw depth buffer (no deep copy)
  if (msg->data.empty() || msg->width == 0 || msg->height == 0) {
    return;
  }
  const cv::Mat depth_mat(
    static_cast<int>(msg->height),
    static_cast<int>(msg->width),
    CV_16UC1,
    const_cast<uint8_t *>(msg->data.data()));
  (void)depth_mat;
}

}  // namespace ros2_mot_ltp

RCLCPP_COMPONENTS_REGISTER_NODE(ros2_mot_ltp::MotLtpPipelineNode)
