#include "simple_pure_pursuit/simple_pure_pursuit.hpp"

#include <motion_utils/motion_utils.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include <tf2/utils.h>

#include <algorithm>

namespace simple_pure_pursuit
{

using motion_utils::findNearestIndex;
using tier4_autoware_utils::calcLateralDeviation;
using tier4_autoware_utils::calcYawDeviation;

SimplePurePursuit::SimplePurePursuit()
: Node("simple_pure_pursuit"),
  // initialize parameters
  wheel_base_(declare_parameter<float>("wheel_base", 2.14)),
  lookahead_gain_(declare_parameter<float>("lookahead_gain", 1.0)),
  lookahead_min_distance_(declare_parameter<float>("lookahead_min_distance", 1.0)),
  speed_proportional_gain_(declare_parameter<float>("speed_proportional_gain", 1.0)),
  use_external_target_vel_(declare_parameter<bool>("use_external_target_vel", false)),
  external_target_vel_(declare_parameter<float>("external_target_vel", 0.0)),
  steering_tire_angle_gain_(declare_parameter<float>("steering_tire_angle_gain", 1.0)),

  // 曲率コンストラクタの追加
  curvature_gain_(declare_parameter<float>("curvature_gain", 0.5)),
  // 最大先行距離の追加
  lookahead_max_distance_(declare_parameter<float>("lookahead_max_distance", 10.0))
{
  pub_cmd_ = create_publisher<AckermannControlCommand>("output/control_cmd", 1);
  pub_raw_cmd_ = create_publisher<AckermannControlCommand>("output/raw_control_cmd", 1);
  pub_lookahead_point_ = create_publisher<PointStamped>("/control/debug/lookahead_point", 1);

  const auto bv_qos = rclcpp::QoS(rclcpp::KeepLast(1)).durability_volatile().best_effort();
  sub_kinematics_ = create_subscription<Odometry>(
    "input/kinematics", bv_qos, [this](const Odometry::SharedPtr msg) { odometry_ = msg; });
  sub_trajectory_ = create_subscription<Trajectory>(
    "input/trajectory", bv_qos, [this](const Trajectory::SharedPtr msg) { trajectory_ = msg; });

  using namespace std::literals::chrono_literals;
  timer_ =
    rclcpp::create_timer(this, get_clock(), 10ms, std::bind(&SimplePurePursuit::onTimer, this));
}

AckermannControlCommand zeroAckermannControlCommand(rclcpp::Time stamp)
{
  AckermannControlCommand cmd;
  cmd.stamp = stamp;
  cmd.longitudinal.stamp = stamp;
  cmd.longitudinal.speed = 0.0;
  cmd.longitudinal.acceleration = 0.0;
  cmd.lateral.stamp = stamp;
  cmd.lateral.steering_tire_angle = 0.0;
  return cmd;
}

// 3点から曲率を計算するヘルパ関数
double calcCurvatureFrom3Points(
  const geometry_msgs::msg::Point & p_prev, const geometry_msgs::msg::Point & p_curr,
  const geometry_msgs::msg::Point & p_next)
{
  const double a = std::hypot(p_curr.x - p_prev.x, p_curr.y - p_prev.y);
  const double b = std::hypot(p_next.x - p_curr.x, p_next.y - p_curr.y);
  const double c = std::hypot(p_prev.x - p_next.x, p_prev.y - p_next.y);
  const double s = (a + b + c) / 2.0;
  const double area_squared = s * (s - a) * (s - b) * (s - c);

  // 3点がほぼ一直線上にある場合 (面積が非常に小さい) は曲率を0とする
  if (area_squared < 1e-6) {
    return 0.0;
  }
  const double area = std::sqrt(area_squared);

  // 外接円の半径 R = abc / (4 * Area)
  // 曲率 kappa = 1/R = 4 * Area / (abc)
  if (a * b * c < 1e-6) {
    return 0.0;
  }
  return 4.0 * area / (a * b * c);
}


void SimplePurePursuit::onTimer()
{
  // check data
  if (!subscribeMessageAvailable()) {
    return;
  }

  size_t closet_traj_point_idx =
    findNearestIndex(trajectory_->points, odometry_->pose.pose.position);

  // publish zero command
  AckermannControlCommand cmd = zeroAckermannControlCommand(get_clock()->now());

  // get closest trajectory point from current position
  TrajectoryPoint closet_traj_point = trajectory_->points.at(closet_traj_point_idx);

  // calc longitudinal speed and acceleration
  double target_longitudinal_vel =
    use_external_target_vel_ ? external_target_vel_ : closet_traj_point.longitudinal_velocity_mps;
  double current_longitudinal_vel = odometry_->twist.twist.linear.x;

  cmd.longitudinal.speed = target_longitudinal_vel;
  cmd.longitudinal.acceleration =
    speed_proportional_gain_ * (target_longitudinal_vel - current_longitudinal_vel);

  // calc lateral control
  //// calc lookahead distance
  // double lookahead_distance = lookahead_gain_ * target_longitudinal_vel + lookahead_min_distance_;
  // 新しい先行距離計算ロジック
  const int curvature_calc_point_dist = 5; // 5点先の点を使う
  double kappa = calcTrajectoryCurvature(closet_traj_point_idx, curvature_calc_point_dist);
  
    // 2. 曲率と速度に基づいて先行距離を計算
  double lookahead_distance =
    (lookahead_gain_ / (curvature_gain_ * std::abs(kappa) + 1.0)) * target_longitudinal_vel +
    lookahead_min_distance_;
  
  // 3. 先行距離に上限を設定
  lookahead_distance = std::min(lookahead_distance, lookahead_max_distance_);
  //// calc center coordinate of rear wheel
  double rear_x = odometry_->pose.pose.position.x -
                  wheel_base_ / 2.0 * std::cos(odometry_->pose.pose.orientation.z);
  double rear_y = odometry_->pose.pose.position.y -
                  wheel_base_ / 2.0 * std::sin(odometry_->pose.pose.orientation.z);
  //// search lookahead point
  auto lookahead_point_itr = std::find_if(
    trajectory_->points.begin() + closet_traj_point_idx, trajectory_->points.end(),
    [&](const TrajectoryPoint & point) {
      return std::hypot(point.pose.position.x - rear_x, point.pose.position.y - rear_y) >=
             lookahead_distance;
    });
  double lookahead_point_x = lookahead_point_itr->pose.position.x;
  double lookahead_point_y = lookahead_point_itr->pose.position.y;

  geometry_msgs::msg::PointStamped lookahead_point_msg;
  lookahead_point_msg.header.stamp = get_clock()->now();
  lookahead_point_msg.header.frame_id = "map";
  lookahead_point_msg.point.x = lookahead_point_x;
  lookahead_point_msg.point.y = lookahead_point_y;
  lookahead_point_msg.point.z = closet_traj_point.pose.position.z;
  pub_lookahead_point_->publish(lookahead_point_msg);

  // calc steering angle for lateral control
  double alpha = std::atan2(lookahead_point_y - rear_y, lookahead_point_x - rear_x) -
                 tf2::getYaw(odometry_->pose.pose.orientation);
  cmd.lateral.steering_tire_angle =
    steering_tire_angle_gain_ * std::atan2(2.0 * wheel_base_ * std::sin(alpha), lookahead_distance);

  pub_cmd_->publish(cmd);
  cmd.lateral.steering_tire_angle /=  steering_tire_angle_gain_;
  pub_raw_cmd_->publish(cmd);
}

// 軌道上の指定されたインデックス周辺の曲率を計算する
double SimplePurePursuit::calcTrajectoryCurvature(
  const size_t point_idx, const int curvature_calc_point_dist)
{
  if (trajectory_->points.size() < 3) {
    return 0.0;
  }

  const size_t prev_idx =
    (point_idx < static_cast<size_t>(curvature_calc_point_dist))
     ? 0
      : point_idx - curvature_calc_point_dist;
  const size_t next_idx = (point_idx + curvature_calc_point_dist >= trajectory_->points.size())
                           ? trajectory_->points.size() - 1
                            : point_idx + curvature_calc_point_dist;

  // 3点が同じでないことを確認
  if (prev_idx == point_idx || point_idx == next_idx) {
    return 0.0;
  }

  return calcCurvatureFrom3Points(
    trajectory_->points.at(prev_idx).pose.position, trajectory_->points.at(point_idx).pose.position,
    trajectory_->points.at(next_idx).pose.position);
}

bool SimplePurePursuit::subscribeMessageAvailable()
{
  if (!odometry_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000 /*ms*/, "odometry is not available");
    return false;
  }
  if (!trajectory_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000 /*ms*/, "trajectory is not available");
    return false;
  }
  if (trajectory_->points.empty()) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000 /*ms*/,  "trajectory points is empty");
      return false;
    }
  return true;
}

}  // namespace simple_pure_pursuit




int main(int argc, char const * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<simple_pure_pursuit::SimplePurePursuit>());
  rclcpp::shutdown();
  return 0;
}
