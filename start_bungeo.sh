#!/bin/bash
# 붕어빵 데모 올인원 실행 (bungeoppang2 전용) — 탭 5개
S="source ~/ros2_ws/install/setup.bash"

tab() {  # tab "제목" "명령"
  gnome-terminal --tab -t "$1" -- bash -c "$2; exec bash"
  sleep 0.7
}

tab "1.에뮬+RViz" "$S; ros2 launch bungeoppang2 sim.launch.py bake:=false"
tab "2.공정"      "sleep 18; $S; ros2 run bungeoppang2 bake_sequence --serve"
tab "3.총괄"      "sleep 18; $S; ros2 run bungeoppang2 orchestrator"
tab "4.음성"      "sleep 18; $S; ros2 run bungeoppang2 voice_node"
tab "5.사람(Enter)" "$S; echo '=== Enter 를 누르면 손님 등장 신호 ==='; while read -r _; do ros2 topic pub --once /bungeoppang/person std_msgs/msg/Bool '{data: true}' > /dev/null; echo '👤 손님 등장 전송!'; done"
