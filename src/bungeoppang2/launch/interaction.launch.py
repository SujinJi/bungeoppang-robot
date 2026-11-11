"""손님 응대 묶음 실행: 사람 인식 + 음성인식 + 총괄
  ros2 launch bungeoppang2 interaction.launch.py
(bake_sequence --serve 는 로봇 쪽 터미널에서 따로 실행)
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package="bungeoppang2", executable="vision_node", output="screen"),
        Node(package="bungeoppang2", executable="voice_node", output="screen"),
        Node(package="bungeoppang2", executable="orchestrator", output="screen"),
    ])
