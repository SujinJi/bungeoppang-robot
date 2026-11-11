#!/bin/bash
# 사용법:
#   ~/ros2_ws/start_all.sh                  → pour_demo, 테스트 모드 (키보드로 손님·주문)
#   ~/ros2_ws/start_all.sh my_node          → my_node, 테스트 모드
#   ~/ros2_ws/start_all.sh pour_demo real   → pour_demo, 실제 카메라·마이크
ROBOT=${1:-pour_demo}
MODE=${2:-test}
WS=~/ros2_ws
SRC="source /opt/ros/jazzy/setup.bash && cd $WS && source install/setup.bash"

open_term() {   # $1 = 창 제목, $2 = 실행할 명령 (끝나도 창은 남겨둠)
  if command -v gnome-terminal > /dev/null; then
    gnome-terminal --title="$1" -- bash -c "$2; exec bash"
  elif command -v terminator > /dev/null; then
    terminator -T "$1" -e "bash -c '$2; exec bash'" &
  else
    xterm -T "$1" -e bash -c "$2; exec bash" &
  fi
}

echo "1) 정리"
$WS/clean.sh
echo "2) 빌드"
(source /opt/ros/jazzy/setup.bash && cd $WS && colcon build --packages-select bungeoppang) || { echo "❌ 빌드 실패"; exit 1; }

case "$MODE" in
  real)  V=""; C="" ;;                                                  # 마이크 + 카메라
  voice) V=""; C="--ros-args -p mode:=key" ;;                           # 마이크 + 키보드(손님 도착)
  *)     V="--ros-args -p mode:=text"; C="--ros-args -p mode:=key" ;;   # 키보드 + 키보드
esac

echo "3) 창 4개 열기"
open_term "1 MoveIt + RViz2" "$SRC && ros2 launch dsr_bringup2 dsr_bringup2_moveit.launch.py mode:=virtual model:=m1013 host:=127.0.0.1 2>&1 | tee ~/moveit.log"
sleep 3
open_term "2 로봇 ($ROBOT)" "$SRC && SKIP_BUILD=1 $WS/run_demo.sh $ROBOT"
open_term "3 음성 주문" "$SRC && ros2 run bungeoppang voice_order $V"
open_term "4 손님 감지" "$SRC && ros2 run bungeoppang customer $C"
open_term "5 상황판" "$SRC && ros2 run bungeoppang hud"
echo "✅ 창 4개를 열었어요. '2 로봇' 창에 '손님을 기다리는 중'이 뜨면 '4 손님 감지' 창에서 Enter!"
