#include "nav2_mpc_controller/mpc_controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_core/goal_checker.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <nav2_core/exceptions.hpp>

namespace nav2_mpc_controller
{

void MPCController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{

  auto node = parent.lock();
  node_ = parent;
  if (!node) {
    throw nav2_core::PlannerException("Unable to lock node!");
  }

  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  tf_buffer_ = tf;
  plugin_name_ = name;
  logger_ = node->get_logger();
  clock_ = node->get_clock();
  RCLCPP_INFO(rclcpp::get_logger("nav2_mpc_controller"), "MPCController configured (skeleton)");
}

void MPCController::activate()
{
  RCLCPP_INFO(rclcpp::get_logger("nav2_mpc_controller"), "MPCController activated (skeleton)");
}

void MPCController::deactivate()
{
  RCLCPP_INFO(rclcpp::get_logger("nav2_mpc_controller"), "MPCController deactivated (skeleton)");
}

void MPCController::cleanup()
{
  RCLCPP_INFO(rclcpp::get_logger("nav2_mpc_controller"), "MPCController cleanup (skeleton)");
}

void MPCController::setPlan(const nav_msgs::msg::Path & path)
{
  (void)path;
  RCLCPP_INFO(rclcpp::get_logger("nav2_mpc_controller"), "MPCController received plan (skeleton)");
}

geometry_msgs::msg::TwistStamped MPCController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  (void)pose;
  (void)velocity;
  (void)goal_checker;
  geometry_msgs::msg::TwistStamped cmd;
  // Default: zero command — replace with MPC computation
  cmd.twist.linear.x = 0.0;
  cmd.twist.angular.z = 0.0;
  RCLCPP_INFO(logger_,"mpc_local_planner");
  return cmd;
}


void MPCController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  (void)speed_limit;
  (void)percentage;
}

}  // namespace nav2_mpc_controller

// Export for pluginlib
PLUGINLIB_EXPORT_CLASS(nav2_mpc_controller::MPCController, nav2_core::Controller)
