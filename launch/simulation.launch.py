from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    OpaqueFunction,
    GroupAction,
)
from moveit_configs_utils import MoveItConfigsBuilder
from srdfdom.srdf import SRDF
from ament_index_python import get_package_share_directory
import os
import yaml

robot_data = {
    "package_name": "franka_moveit_config",
    "robot_name": "panda",
    "move_group_name": "panda_arm",
    "target_link_name": "panda_tip",
    "base_link_name": "panda_link0",
}


def launch_robot(context):
    actions = []

    allow_trajectory_execution = True
    should_publish = True

    move_group_configuration = {
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": allow_trajectory_execution,
        "capabilities": "",
        "disable_capabilities": "",
        "publish_planning_scene": should_publish,
        "publish_geometry_updates": should_publish,
        "publish_state_updates": should_publish,
        "publish_transforms_updates": should_publish,
        "monitor_dynamics": False,
    }

    moveit_config = MoveItConfigsBuilder(
        robot_data["robot_name"],
        package_name=robot_data["package_name"],
    ).to_moveit_configs()
    move_group_params = [
        moveit_config.to_dict(),
        move_group_configuration,
        {"use_sim_time": True},
        {"planning_scene_monitor_options.joint_state_topic": "/joint_states"},
    ]

    rviz_arguments = ["-d", str(moveit_config.package_path / "config/moveit.rviz")]
    rviz_parameters = [
        moveit_config.planning_pipelines,
        moveit_config.robot_description_kinematics,
        {"use_sim_time": True},
    ]

    actions_robot_namespace = []

    name_counter = 0
    for key, xml_contents in moveit_config.robot_description_semantic.items():
        srdf = SRDF.from_xml_string(xml_contents)
        for vj in srdf.virtual_joints:
            actions_robot_namespace.append(
                Node(
                    package="tf2_ros",
                    executable="static_transform_publisher",
                    name=f"static_transform_publisher{name_counter}",
                    output="log",
                    arguments=[
                        "--frame-id",
                        vj.parent_frame,
                        "--child-frame-id",
                        vj.child_link,
                    ],
                )
            )
            name_counter += 1

    actions_robot_namespace.append(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            respawn=True,
            output="screen",
            parameters=[
                {
                    "robot_description": moveit_config.robot_description[
                        "robot_description"
                    ],
                    "publish_frequency": 15.0,
                },
                {"use_sim_time": True},
            ],
        )
    )
    actions_robot_namespace.append(
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                moveit_config.robot_description,
                str(moveit_config.package_path / "config/ros2_controllers.yaml"),
                {"use_sim_time": True},
            ],
        )
    )

    controller_names = moveit_config.trajectory_execution.get(
        "moveit_simple_controller_manager", {}
    ).get("controller_names", [])
    for controller in controller_names + ["joint_state_broadcaster"]:
        actions_robot_namespace.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller],
                output="screen",
            )
        )

    actions_robot_namespace.append(
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=move_group_params,
        )
    )
    actions_robot_namespace.append(
        Node(
            package="rviz2",
            executable="rviz2",
            output="screen",
            arguments=rviz_arguments,
            parameters=rviz_parameters,
        )
    )

    actions_robot_namespace.append(
        Node(
            package="isaac_moveit_sample",
            executable="clock",
            name="clock",
        )
    )

    kinematics_yaml = load_yaml(robot_data["package_name"], "config/kinematics.yaml")
    actions_robot_namespace.append(
        Node(
            package="isaac_moveit_sample",
            executable="motion_planner",
            parameters=[
                kinematics_yaml,
                {
                    "move_group_name": robot_data["move_group_name"],
                    "target_link_name": robot_data["target_link_name"],
                    "base_link_name": robot_data["base_link_name"],
                },
                {"use_sim_time": True},
            ],
        )
    )

    actions.append(GroupAction(actions=actions_robot_namespace))

    return actions


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except (
        EnvironmentError
    ):  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    ld = LaunchDescription()

    opfunc = OpaqueFunction(function=launch_robot)
    ld.add_action(opfunc)

    return ld
