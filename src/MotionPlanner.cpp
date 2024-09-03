#include "isaac_moveit_sample/MotionPlanner.hpp"

MotionPlanner::MotionPlanner(std::shared_ptr<rclcpp::Node> move_group_node)
    : Node("motion_planner"), move_group_node(move_group_node) {
  // Create a ROS logger
  logger = std::make_unique<rclcpp::Logger>(rclcpp::get_logger("motion_planner"));

  // Declare and read node parameters
  this->declare_parameter("move_group_name", rclcpp::PARAMETER_STRING);
  this->declare_parameter("target_link_name", rclcpp::PARAMETER_STRING);
  this->declare_parameter("base_link_name", rclcpp::PARAMETER_STRING);
  rclcpp::Parameter move_group_name_param = this->get_parameter("move_group_name");
  rclcpp::Parameter target_link_name_param = this->get_parameter("target_link_name");
  rclcpp::Parameter base_link_name_param = this->get_parameter("base_link_name");
  this->action_endpoint = std::string("move_to_pose");
  this->move_group_name = move_group_name_param.as_string();
  this->target_link_name = target_link_name_param.as_string();
  this->base_link_name = base_link_name_param.as_string();

  // Create move group interface
  moveit::planning_interface::MoveGroupInterface::Options options(move_group_name);
  move_group_interface = std::make_unique<MoveGroupInterface>(move_group_node, options);
  // Configure interface velocity and acceleration
  move_group_interface->setMaxVelocityScalingFactor(1.0);
  move_group_interface->setMaxAccelerationScalingFactor(1.0);
  move_group_interface->setPlanningPipelineId("ompl");
  move_group_interface->setPlanningTime(2.0);
  move_group_interface->setNumPlanningAttempts(10);

  // Initialize TF Listener
  tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

  // Initialize action servers
  this->move_to_pose_action_server = rclcpp_action::create_server<MoveToPose>(
      this, this->action_endpoint, std::bind(&MotionPlanner::moveToPoseHandleGoal, this, _1, _2),
      std::bind(&MotionPlanner::moveToPoseHandleCancel, this, _1),
      std::bind(&MotionPlanner::moveToPoseHandleAccepted, this, _1));
}

rclcpp_action::GoalResponse MotionPlanner::moveToPoseHandleGoal(const rclcpp_action::GoalUUID &uuid,
                                                                std::shared_ptr<const MoveToPose::Goal> goal) {
  RCLCPP_INFO(*logger, "Received goal request for action endpoint %s", action_endpoint.c_str());
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}
rclcpp_action::CancelResponse
MotionPlanner::moveToPoseHandleCancel(const std::shared_ptr<GoalHandleMoveToPose> goal_handle) {
  RCLCPP_INFO(*logger, "Received request to cancel goal for action endpoint %s", action_endpoint.c_str());
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void MotionPlanner::moveToPoseHandleAccepted(const std::shared_ptr<GoalHandleMoveToPose> goal_handle) {
  // this needs to return quickly to avoid blocking the executor, so spin up a new thread
  std::thread{std::bind(&MotionPlanner::moveToPoseExecute, this, _1), goal_handle}.detach();
}

void MotionPlanner::moveToPoseExecute(const std::shared_ptr<GoalHandleMoveToPose> goal_handle) {
  RCLCPP_INFO(*logger, "Executing goal for action endpoint %s", action_endpoint.c_str());
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<MoveToPose::Feedback>();
  auto result = std::make_shared<MoveToPose::Result>();

  RCLCPP_INFO(*logger, "Moving arm to pose");
  RCLCPP_INFO(*logger, "base link name: %s", base_link_name.c_str());
  RCLCPP_INFO(*logger, "target link name: %s", target_link_name.c_str());
  RCLCPP_INFO(*logger, "position %f %f %f", goal->goal.pose.position.x, goal->goal.pose.position.y,
              goal->goal.pose.position.z);

  geometry_msgs::msg::TransformStamped transform_world_to_robot_base;
  std::string toFrameRel(base_link_name);
  std::string fromFrameRel("World");
  try {
    // Wait for 1 seconds for transform to be received by tf2
    std::this_thread::sleep_for(std::chrono::seconds(3));

    transform_world_to_robot_base = tf_buffer->lookupTransform(toFrameRel, fromFrameRel, tf2::TimePointZero);
    RCLCPP_INFO(*logger, "Transform %s to %s: %f %f %f", toFrameRel.c_str(), fromFrameRel.c_str(),
                transform_world_to_robot_base.transform.translation.x,
                transform_world_to_robot_base.transform.translation.y,
                transform_world_to_robot_base.transform.translation.z);
  } catch (const tf2::TransformException &ex) {
    RCLCPP_INFO(*logger, "Could not transform %s to %s: %s", toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
    goal_handle->abort(result);
    return;
  }

  Eigen::Vector3d goal_position_world, goal_position_robot_base;
  goal_position_world << goal->goal.pose.position.x, goal->goal.pose.position.y, goal->goal.pose.position.z;
  tf2::doTransform(goal_position_world, goal_position_robot_base, transform_world_to_robot_base);

  RCLCPP_INFO(*logger, "Goal position in robot base frame %f %f %f", goal_position_robot_base.x(),
              goal_position_robot_base.y(), goal_position_robot_base.z());

  geometry_msgs::msg::TransformStamped transform_goal;
  transform_goal.transform.translation.x = goal_position_robot_base.x();
  transform_goal.transform.translation.y = goal_position_robot_base.y();
  transform_goal.transform.translation.z = goal_position_robot_base.z();

  // Move robot to pose
  geometry_msgs::msg::Pose goal_pose;
  tf2::Transform intermediate_tf_transform;
  tf2::fromMsg(transform_goal.transform, intermediate_tf_transform);
  tf2::toMsg(intermediate_tf_transform, goal_pose);

  auto goal_msg = geometry_msgs::msg::PoseStamped();
  goal_msg.header.frame_id = base_link_name;
  goal_msg.pose = goal_pose;
  goal_msg.pose.orientation.x = goal->goal.pose.orientation.x;
  goal_msg.pose.orientation.y = goal->goal.pose.orientation.y;
  goal_msg.pose.orientation.z = goal->goal.pose.orientation.z;
  goal_msg.pose.orientation.w = goal->goal.pose.orientation.w;
  move_group_interface->setStartStateToCurrentState();
  move_group_interface->setPoseReferenceFrame(base_link_name);
  move_group_interface->setPoseTarget(goal_msg, target_link_name);

  // Create a plan to that target pose
  moveit::planning_interface::MoveGroupInterface::Plan msg;
  auto const success = static_cast<bool>(move_group_interface->plan(msg));

  // Execute the plan
  if (success) {
    result->success = move_group_interface->execute(msg) == moveit::core::MoveItErrorCode::SUCCESS;
  } else {
    RCLCPP_ERROR(*logger, "Planning failed!");
    result->success = false;
  }

  // Check if goal is done
  if (rclcpp::ok()) {
    goal_handle->succeed(result);
    RCLCPP_INFO(*logger, "Goal succeeded for action endpoint %s", action_endpoint.c_str());
  }
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("move_group_interface_custom", node_options);
  auto motion_planner_node = std::make_shared<MotionPlanner>(move_group_node);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  executor.add_node(motion_planner_node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}