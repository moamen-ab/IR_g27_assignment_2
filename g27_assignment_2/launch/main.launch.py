import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import SetLaunchConfiguration
from launch_ros.actions import SetParameter
from launch.actions import ExecuteProcess
from moveit_configs_utils import MoveItConfigsBuilder


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
    ur_type = LaunchConfiguration("ur_type")

    # ------------------------------------------------------------------
    # MoveIt configuration — exactly as in ir_movit.launch.py
    # ------------------------------------------------------------------
    moveit_config = (
        MoveItConfigsBuilder(robot_name="ur", package_name="ir_movit_config")
        .robot_description(
            Path("config") / "robot_with_environment.urdf.xacro",
            {"name": ur_type})
        .robot_description_semantic(
            Path("config") / "ur_gripper.srdf",
            {"name": ur_type})
        .robot_description_kinematics(Path("config") / "kinematics.yaml")
        .to_moveit_configs()
    )

    # ------------------------------------------------------------------
    # Paths
    # ------------------------------------------------------------------
    pkg_share       = get_package_share_directory("g27_assignment_2")
    apriltag_params = os.path.join(pkg_share, "config", "cube_tags.yaml")

    # Controllers yaml from ir_movit_config — needed by the MTC
    controllers_yaml = os.path.join(
        get_package_share_directory("ir_movit_config"),
        "config",
        "ros2_controllers.yaml",
    )

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
    # 3. AprilTag detection node — delayed 5s for Gazebo + bridge
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
    # 4. Pose estimator node — delayed 6s, needs apriltag up first
    # ------------------------------------------------------------------
    pose_estimator_node = Node(
        package="g27_assignment_2",
        executable="pose_estimator_node",
        name="pose_estimator_node",
        output="screen",
    )


    # ------------------------------------------------------------------
    # 5. Cube swap node (MTC-based)
    # ------------------------------------------------------------------
    cube_swap_node = Node(
        package="g27_assignment_2",
        executable="cube_swap_node",
        name="cube_swap_node",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    # ------------------------------------------------------------------
    # 6. cmd durind runtime
    # ------------------------------------------------------------------
    EP = ExecuteProcess(
        cmd=['ros2', 'param', 'set', '/joint_trajectory_controller', 'allow_nonzero_velocity_at_trajectory_end', 'true'],
                output='screen'
    )

    return LaunchDescription([
        ur_type_arg,
        SetParameter(name="use_sim_time", value=True),
        SetParameter(name="capabilities", value="move_group/ExecuteTaskSolutionCapability"),
        SetParameter(name="allow_nonzero_velocity_at_trajectory_end", value="true"),   
        sim_launch,
        TimerAction(period=5.0,  actions=[apriltag_node]),
        TimerAction(period=6.0,  actions=[pose_estimator_node]),
        TimerAction(period=10.0, actions=[cube_swap_node]),
        TimerAction(period=30.0, actions=[EP]),
    ])
