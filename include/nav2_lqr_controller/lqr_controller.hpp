// Lightweight LQR controller skeleton for nav2
#pragma once

#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_core/goal_checker.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_core/controller.hpp"

namespace nav2_lqr_controller
{

class LQRController : public nav2_core::Controller
{
public:
  LQRController() = default;
  ~LQRController() override = default;

  // nav2_core::Controller interface
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  void setPlan(const nav_msgs::msg::Path & path) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::string plugin_name_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_;
  rclcpp::Logger logger_ {rclcpp::get_logger("LqrController")};
  rclcpp::Clock::SharedPtr clock_;

};

}  // namespace nav2_lqr_controller
