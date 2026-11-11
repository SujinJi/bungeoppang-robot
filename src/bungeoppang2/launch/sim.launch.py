"""시뮬레이션 올인원 실행 — 터미널 하나로 전부:
  ros2 launch bungeoppang2 sim.launch.py
= 두산 에뮬레이터 + 전용 RViz(로봇·그리퍼·물체) + 20초 뒤 전체 공정 자동 시작

옵션:
  bake:=false                 공정은 빼고 에뮬레이터+RViz만 (공정은 나중에 ros2 run으로)
  f1:=슈크림 f2:=팥           틀 1·2의 속재료 (기본: 팥, 슈크림)
  bake_delay:=25.0            에뮬레이터 준비를 기다리는 시간(초)
종료는 이 터미널에서 Ctrl+C 한 번 (전부 같이 꺼져요).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.substitutions import PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bake = LaunchConfiguration("bake")
    delay = LaunchConfiguration("bake_delay")

    emulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare("dsr_bringup2"), "launch", "dsr_bringup2_rviz.launch.py"])),
        launch_arguments={
            "mode": "virtual", "host": "127.0.0.1", "port": "12345",
            "model": "e0509", "gui": "false",
        }.items(),
    )

    viewer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare("bungeoppang2"), "launch", "gripper_view.launch.py"])),
    )

    bake_node = TimerAction(
        period=PythonExpression(delay),
        actions=[Node(
            package="bungeoppang2",
            executable="bake_sequence",
            arguments=[LaunchConfiguration("f1"), LaunchConfiguration("f2")],
            output="screen",
            condition=IfCondition(bake),
        )],
    )

    return LaunchDescription([
        DeclareLaunchArgument("bake", default_value="true"),
        DeclareLaunchArgument("bake_delay", default_value="20.0"),
        DeclareLaunchArgument("f1", default_value="팥"),
        DeclareLaunchArgument("f2", default_value="슈크림"),
        emulator,
        viewer,
        bake_node,
    ])
