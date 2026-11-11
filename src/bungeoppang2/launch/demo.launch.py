"""인터랙션 데모 올인원 — 터미널 하나, 명령 하나:
  ros2 launch bungeoppang2 demo.launch.py
= 에뮬레이터 + RViz + 공정(--serve) + 총괄 + 음성인식(마이크)

손님 등장(사람 인식 대신 키보드)은 다른 터미널에서:
  ros2 topic pub --once /bungeoppang/person std_msgs/msg/Bool "{data: true}"
→ ㄱ자 인사 → 마이크에 "팥 주세요" → 끄덕 → 공정 자동 시작

옵션:
  voice:=false    음성인식 끄기 → 다른 터미널에서 텍스트로 주문:
                  ros2 run bungeoppang2 text_order   (문장 입력, 예: "팥 하나 슈크림 하나 주세요")
                  (또는 ros2 topic pub --once /bungeoppang/order std_msgs/msg/String "{data: 팥x1}")
종료: 이 터미널에서 Ctrl+C 한 번.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare("bungeoppang2"), "launch", "sim.launch.py"])),
        launch_arguments={"bake": "false"}.items(),
    )

    later = TimerAction(period=35.0, actions=[
        Node(package="bungeoppang2", executable="bake_sequence",
             arguments=["--serve"], output="screen"),
        Node(package="bungeoppang2", executable="orchestrator", output="screen"),
        Node(package="bungeoppang2", executable="voice_node", output="screen",
             condition=IfCondition(LaunchConfiguration("voice"))),
    ])

    return LaunchDescription([
        DeclareLaunchArgument("voice", default_value="true"),
        sim,
        later,
    ])
