# 붕어빵 로봇팔 (Bungeoppang Robot)

두산 협동로봇(E0509 / M1013)으로 붕어빵을 굽는 ROS 2 프로젝트입니다.
손님 감지 → 음성 주문 → 반죽 붓기 → 속재료(팥/슈크림) → 틀 닫기·뒤집기 → 굽기 → 접시에 담기.

## 환경
- Ubuntu, ROS 2 Jazzy
- [doosan-robot2](https://github.com/DoosanRobotics/doosan-robot2) (jazzy), MoveIt 2, RViz2
- 그리퍼: ROBOTIS RH-P12-RN

## 구성
| 경로 | 설명 |
|---|---|
| `src/bungeoppang` | 전체 공정 (pour_demo.py, layout.py, 음성 주문, 손님 감지, HUD) |
| `src/bungeoppang2` | E0509용: 굽기부터 꼬챙이로 꺼내 컵에 담기까지 |
| `src/bungeoppang_description` | 붕어빵 기계·작업대 URDF/메시 |
| `*.sh` | 실행·정리·백업 스크립트 |

자세한 공정과 배치 값은 [CLAUDE.md](CLAUDE.md)를 참고하세요.

## 빌드 & 실행
```bash
cd ~/ros2_ws/src && git clone https://github.com/SujinJi/bungeoppang-robot.git
cd ~/ros2_ws && colcon build --symlink-install && source install/setup.bash
ROBOT_MODEL=e0509 ./start_all.sh pour_demo voice   # 가상
```
