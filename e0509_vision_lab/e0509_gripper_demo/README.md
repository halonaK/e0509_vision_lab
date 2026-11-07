# e0509_gripper_demo

Doosan E0509 + RH-P12-RN-A 실습 예제 모음 (ROS 2 Jazzy).
`e0509_gripper_description` 의 bringup 위에서 동작하며, **virtual 모드(에뮬레이터)** 에서 모두 실행됩니다.

| 예제 | 실행 이름 | 배우는 것 |
| --- | --- | --- |
| 1 | `ex01_gripper_sequence` | 그리퍼 서비스(open/close)와 토픽(position_cmd) |
| 2 | `ex02_move_joint` | 관절 이동, degree 단위, 관절별 역할, 속도 비교 |
| 3 | `ex03_move_line` | TCP 직선 이동, 현재 위치 읽기, 상대/절대 이동 |
| 4 | `ex04_pick_and_place` | 팔 + 그리퍼 조합 시퀀스 (비전 연동의 뼈대) |
| 5 | `ex05_state_monitor` | joint_states 구독, 콜백 안의 비동기 서비스 호출 |

## 설치

```bash
cd ~/doosan_ws/src
tar xzf ~/Downloads/e0509_gripper_demo.tar.gz
cd ~/doosan_ws
colcon build --symlink-install --packages-select e0509_gripper_demo
source install/setup.bash
```

## 실행

터미널 1 (켜둔 채로):

```bash
ros2 launch e0509_gripper_description bringup.launch.py mode:=virtual
```

터미널 2 (예제 하나씩):

```bash
ros2 run e0509_gripper_demo ex01_gripper_sequence
ros2 run e0509_gripper_demo ex02_move_joint
ros2 run e0509_gripper_demo ex03_move_line
ros2 run e0509_gripper_demo ex04_pick_and_place
```

터미널 3 (다른 예제를 돌리는 동안 켜 두기):

```bash
ros2 run e0509_gripper_demo ex05_state_monitor
```

## 파일 구조

```
e0509_gripper_demo/
├── robot_client.py        # 공통 헬퍼: move_joint / move_line / get_tcp / gripper_*
├── gripper_demo_node.py   # 예제 1
├── ex02_move_joint.py
├── ex03_move_line.py
├── ex04_pick_and_place.py
└── ex05_state_monitor.py
```

## 주의

- virtual 모드에서는 그리퍼 서비스가 `success: false` 를 줄 수 있습니다. RViz 의 그리퍼는 정상적으로 움직입니다.
- `stroke` 는 그리퍼에서 읽은 값이 아니라 보낸 명령값입니다.
- 실로봇에서는 ex03·ex04 의 이동 거리와 물체 위치를 실제 작업 공간에 맞게 줄이고, 속도를 낮춰서 시작하세요.
