import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def generate_launch_description():

    # ------------------------------------------------------------------
    # Launch arguments
    # ------------------------------------------------------------------
    ur_type_arg = DeclareLaunchArgument(
        "ur_type", default_value="ur5",
        description="Type/series of used UR robot.",
        choices=["ur3","ur3e","ur5","ur5e","ur7e","ur10","ur10e",
                 "ur12e","ur16e","ur15","ur20","ur30"],
    )

    # ------------------------------------------------------------------
    # Paths
    # ------------------------------------------------------------------
    pkg_share       = get_package_share_directory("g27_assignment_2")
    apriltag_params = os.path.join(pkg_share, "config", "cube_tags.yaml")

    # ------------------------------------------------------------------
    # 1. Base simulation launch file
    # ------------------------------------------------------------------
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ir_launch"),
                "launch",
                "assignment_2.launch.py",
            )
        )
    )

    # ------------------------------------------------------------------
    # 2. AprilTag detection node — delayed 5s for Gazebo + bridge
    # ------------------------------------------------------------------
    apriltag_node = Node(
        package="apriltag_ros",
        executable="apriltag_node",
        name="apriltag",
        remappings=[
            ("image_rect",  "/rgb_camera/image"),
            ("camera_info", "/rgb_camera/camera_info"),
        ],
        parameters=[apriltag_params],
        output="screen",
    )

    # ------------------------------------------------------------------
    # 3. Cube color detector node — delayed 6s, needs apriltag up first
    # ------------------------------------------------------------------
    color_detector_node = Node(
        package="g27_assignment_2",
        executable="cube_color_detector",
        name="cube_color_detector",
        output="screen",
    )

    return LaunchDescription([
        ur_type_arg,
        SetParameter(name="use_sim_time", value=True),
        sim_launch,
        TimerAction(period=5.0, actions=[apriltag_node]),
        TimerAction(period=6.0, actions=[color_detector_node]),
    ])