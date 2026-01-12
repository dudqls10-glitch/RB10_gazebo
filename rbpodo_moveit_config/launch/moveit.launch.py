import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


# Launch arguments
robot_ip = LaunchConfiguration("robot_ip")
use_fake_hardware = LaunchConfiguration("use_fake_hardware")
fake_sensor_commands = LaunchConfiguration("fake_sensor_commands")
model_id = LaunchConfiguration("model_id")
cb_simulation = LaunchConfiguration("cb_simulation")
use_sim_time = LaunchConfiguration("use_sim_time")


def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config",
            default_value="moveit.rviz",
            description="RViz configuration file",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            default_value="10.0.2.7",
            description="RB Cobot Control Box IP Address",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="true",
            description="True if there's no real control box",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "fake_sensor_commands",
            default_value="false",
            description="Use fake sensor commands",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "cb_simulation",
            default_value="Simulation",
            description="Simulation or Real",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "model_id",
            default_value="rb10_1300e",
            description="RB model id",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation time",
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )


def launch_setup(context, *args, **kwargs):

    mappings = {
        "robot_ip": robot_ip,
        "use_fake_hardware": use_fake_hardware,
        "fake_sensor_commands": fake_sensor_commands,
        "model_id": model_id,
        "cb_simulation": cb_simulation,
    }

    moveit_config = (
        MoveItConfigsBuilder("rbpodo")
        .robot_description(
            file_path="config/rbpodo.urdf.xacro",
            mappings=mappings,
        )
        .trajectory_execution(
            file_path="config/moveit_controllers.yaml"
        )
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"]
        )
        .to_moveit_configs()
    )

    # Move group node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            moveit_config.to_dict(),
        ],
    )

    # RViz
    rviz_config = PathJoinSubstitution(
        [
            FindPackageShare("rbpodo_moveit_config"),
            "config",
            LaunchConfiguration("rviz_config"),
        ]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            {"use_sim_time": use_sim_time},
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    # Static TF: world -> link0
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=[
            "0.0", "0.0", "0.0",
            "0.0", "0.0", "0.0",
            "world", "link0",
        ],
    )

    # Robot state publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            {"use_sim_time": use_sim_time},
            moveit_config.robot_description,
        ],
    )

    return [
        static_tf_node,
        robot_state_publisher_node,
        move_group_node,
        rviz_node,
    ]

