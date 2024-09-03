#include <chrono>
#include <functional>
#include <memory>
#include <thread>

#include "isaac_moveit_msgs/action/move_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <Eigen/Dense>
#include <map>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <rclcpp/rclcpp.hpp>
#include <string_view>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::placeholders;

using moveit::planning_interface::MoveGroupInterface;
using MoveToPose = isaac_moveit_msgs::action::MoveToPose;
using GoalHandleMoveToPose = rclcpp_action::ServerGoalHandle<MoveToPose>;

class MotionPlanner : public rclcpp::Node {
public:
  MotionPlanner(std::shared_ptr<rclcpp::Node> move_group_node);

private:
  // Action Server callback methods
  rclcpp_action::GoalResponse moveToPoseHandleGoal(const rclcpp_action::GoalUUID &uuid,
                                                   std::shared_ptr<const MoveToPose::Goal> goal);
  rclcpp_action::CancelResponse moveToPoseHandleCancel(const std::shared_ptr<GoalHandleMoveToPose> goal_handle);
  void moveToPoseHandleAccepted(const std::shared_ptr<GoalHandleMoveToPose> goal_handle);
  void moveToPoseExecute(const std::shared_ptr<GoalHandleMoveToPose> goal_handle);

  std::unique_ptr<tf2_ros::Buffer> tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener;
  std::shared_ptr<rclcpp::Node> move_group_node;
  std::unique_ptr<MoveGroupInterface> move_group_interface;
  std::unique_ptr<rclcpp::Logger> logger;
  rclcpp_action::Server<MoveToPose>::SharedPtr move_to_pose_action_server;
  std::string action_endpoint, move_group_name, target_link_name, base_link_name;
};