#!/bin/bash
cd ~/ros2_ws
source install/setup.bash

echo "1) move_group 준비 대기..."
for i in $(seq 1 30); do
  ros2 node list 2>/dev/null | grep -qx "/move_group" && break; sleep 2
done

echo "2) 컨트롤러 active 대기..."
for i in $(seq 1 30); do
  timeout 5 ros2 control list_controllers -c /controller_manager 2>/dev/null | grep -w "dsr_controller2" | grep -qw "active" && break; sleep 2
done

echo "3) 중복 노드 확인..."
DUP=$(ros2 node list 2>/dev/null | sort | uniq -d | grep -v rviz2_moveit)
if [ -n "$DUP" ]; then
  echo "❌ 중복 노드가 있습니다. clean.sh 후 런치를 다시 켜세요:"; echo "$DUP"
fi

echo "4) 설정 적용..."
ros2 param set /move_group trajectory_execution.allowed_start_tolerance 0.0
ros2 param set /move_group trajectory_execution.execution_duration_monitoring false

echo "5) 빌드 후 실행"
colcon build --packages-select bungeoppang && source install/setup.bash
ros2 run bungeoppang my_node
