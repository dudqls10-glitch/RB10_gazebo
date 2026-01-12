import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, AppendEnvironmentVariable
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg_name = 'rbpodo_description'
    pkg_share = get_package_share_directory(pkg_name)

    # Gazebo resource path
    model_path = os.path.join(pkg_share, '..')
    set_env = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=model_path
    )

    # URDF 생성
    xacro_file = os.path.join(pkg_share, 'robots', 'rb10_1300e.urdf.xacro')
    doc = xacro.process_file(xacro_file)
    robot_desc = doc.toxml()

    # Ignition plugin path
    env_plugin_path = AppendEnvironmentVariable(
        name='IGN_GAZEBO_SYSTEM_PLUGIN_PATH',
        value='/opt/ros/humble/lib:/opt/ros/humble/lib/x86_64-linux-gnu'
    )

    # LD library path
    env_ld_lib_path = AppendEnvironmentVariable(
        name='LD_LIBRARY_PATH',
        value='/opt/ros/humble/lib:/opt/ros/humble/lib/controller_manager'
    )

    # Robot State Publisher
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_desc},
            {'use_sim_time': True}
        ]
    )

    # World
    world_file = os.path.join(
        pkg_share,
        'worlds',
        'empty_fortress.world'
    )

    # Gazebo 실행
    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', world_file],
        output='screen'
    )

    # Robot spawn
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'rb10',
            '-z', '0'
        ],
        output='screen'
    )

    # Controllers
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager'
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_trajectory_controller',
            '--controller-manager', '/controller_manager'
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # Clock bridge (🔥 수정 핵심)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'
        ],
        output='screen'
    )

    return LaunchDescription([
        set_env,
        env_plugin_path,
        env_ld_lib_path,
        gazebo,
        rsp,
        spawn,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        bridge
    ])
11
