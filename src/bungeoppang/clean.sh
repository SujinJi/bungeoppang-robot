#!/bin/bash
echo "ROS 프로세스 정리 중..."
pkill -f "lib/bungeoppang/"; pkill -f "ros2 run bungeoppang"
pkill -f rviz2; pkill -f virtual; pkill -f move_group
pkill -f ros2_control_node; pkill -f dsr_; pkill -f "ros2 launch"; pkill -f robot_state_publisher
docker stop $(docker ps -q) 2>/dev/null
sleep 5
ros2 daemon stop > /dev/null 2>&1; ros2 daemon start > /dev/null 2>&1
LEFT=$(ros2 node list 2>/dev/null)
if [ -z "$LEFT" ]; then echo "✅ 정리 완료"; else echo "❌ 아직 남은 노드:"; echo "$LEFT"; fi
