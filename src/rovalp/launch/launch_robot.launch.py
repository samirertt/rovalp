import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'rovalp'
    pkg_share = get_package_share_directory(package_name)

    # Include robot_state_publisher (handles /robot_description)
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_share, 'launch', 'rsp.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': 'false',
            'use_ros2_control': 'true'
        }.items(),
    )

    # ros2_control_node (do NOT duplicate robot_description here)
    controller_params_file = os.path.join(pkg_share, 'config', 'controllers.yaml')

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_params_file],
        output="screen",
    )

    # Spawners
    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # Delay spawners until ros2_control_node is up
    delayed_spawners = TimerAction(
        period=5.0,
        actions=[joint_broad_spawner, diff_drive_spawner],
    )

    return LaunchDescription([
        rsp,
        ros2_control_node,
        delayed_spawners,
    ])
