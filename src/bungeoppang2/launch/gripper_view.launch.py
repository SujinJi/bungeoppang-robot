"""그리퍼 + 작업대·물체 표시 + 전용 RViz(설정 포함)
  ros2 launch bungeoppang2 gripper_view.launch.py
로봇·그리퍼·물체가 전부 미리 추가된 RViz가 떠요 (Add/저장 불필요).
우리 RViz를 끄려면: rviz:=false
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from bungeoppang2_yammy import workcell as W


def generate_launch_description():
    xacro_file = PathJoinSubstitution(
        [FindPackageShare("bungeoppang2"), "urdf", "gripper_on_e0509.urdf.xacro"])
    description = ParameterValue(Command([
        "xacro ", xacro_file,
        f" yaw_deg:={W.GRIPPER_YAW_DEG}",
        f" mount_z:={W.GRIPPER_MOUNT_Z / 1000.0}",
    ]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true"),
        # 그리퍼 전용 robot_state_publisher → /gripper/robot_description, /gripper/joint_states
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace="gripper",
            parameters=[{"robot_description": description}],
            output="screen",
        ),
        Node(package="bungeoppang2", executable="gripper_viz", output="screen"),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", PathJoinSubstitution(
                [FindPackageShare("bungeoppang2"), "rviz", "bungeoppang.rviz"])],
            condition=IfCondition(LaunchConfiguration("rviz")),
            output="screen",
        ),
    ])
