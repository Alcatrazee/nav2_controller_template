#include "nav2_lqr_controller/lqr_controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_core/goal_checker.hpp"
#include <pluginlib/class_list_macros.hpp>
#include "nav2_core/controller_exceptions.hpp"

namespace nav2_lqr_controller
{

void LQRController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{

  auto node = parent.lock();
  node_ = parent;
  if (!node) {
    throw nav2_core::ControllerException("Unable to lock node!");
  }

  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  tf_buffer_ = tf;
  plugin_name_ = name;
  logger_ = node->get_logger();
  clock_ = node->get_clock();
  RCLCPP_INFO(rclcpp::get_logger("nav2_lqr_controller"), "LQRController configured (skeleton)");
}

void LQRController::activate()
{
  RCLCPP_INFO(rclcpp::get_logger("nav2_lqr_controller"), "LQRController activated (skeleton)");
}

void LQRController::deactivate()
{
  RCLCPP_INFO(rclcpp::get_logger("nav2_lqr_controller"), "LQRController deactivated (skeleton)");
}

void LQRController::cleanup()
{
  RCLCPP_INFO(rclcpp::get_logger("nav2_lqr_controller"), "LQRController cleanup (skeleton)");
}

void LQRController::setPlan(const nav_msgs::msg::Path & path)
{
  (void)path;
  RCLCPP_INFO(rclcpp::get_logger("nav2_lqr_controller"), "LQRController received plan (skeleton)");
}

geometry_msgs::msg::TwistStamped LQRController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  (void)pose;
  (void)velocity;
  (void)goal_checker;
  geometry_msgs::msg::TwistStamped cmd;
  // Default: zero command — replace with LQR computation
  cmd.twist.linear.x = 0.0;
  cmd.twist.angular.z = 0.0;
  return cmd;
}


void LQRController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  (void)speed_limit;
  (void)percentage;
}

}  // namespace nav2_lqr_controller

// Export for pluginlib
PLUGINLIB_EXPORT_CLASS(nav2_lqr_controller::LQRController, nav2_core::Controller)
