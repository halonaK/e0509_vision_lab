# e0509_vision_lab

Doosan E0509 + RH-P12-RN-A 그리퍼, eye-to-hand 카메라 Pick & Place 실습 코드 (ROS 2 Jazzy, Ubuntu 24.04)

| 패키지 | 내용 |
| --- | --- |
| `e0509_gripper_description` | 팔+그리퍼 결합 모델, bringup, 그리퍼 통신 노드 (Jazzy 수정 반영) |
| `e0509_gripper_demo` | 그리퍼 / 관절 이동 / 직선 이동 / Pick & Place / 상태 모니터 예제 |
| `e0509_camera_lab` | 가상 RealSense, 클릭 → base 좌표, eye-to-hand 캘리브레이션 |
| `scripts/apply_jazzy_patch.sh` | doosan-robot2(jazzy)에 Flange Serial 서비스 추가 |

함께 받는 공식 패키지: [DoosanRobotics/doosan-robot2](https://github.com/DoosanRobotics/doosan-robot2) (jazzy), [ROBOTIS-GIT/RH-P12-RN-A](https://github.com/ROBOTIS-GIT/RH-P12-RN-A) (jazzy)

## 설치

```bash
# 0) 다른 ROS 워크스페이스가 ~/.bashrc 에서 source 되고 있지 않은지 확인
grep -n "source" ~/.bashrc

# 1) 소스
mkdir -p ~/doosan_ws/src && cd ~/doosan_ws/src
git clone <이 저장소 주소> e0509_vision_lab
git clone -b jazzy https://github.com/DoosanRobotics/doosan-robot2.git
git clone -b jazzy https://github.com/ROBOTIS-GIT/RH-P12-RN-A.git

# 2) 두산 패키지에 Flange Serial 서비스 추가
bash e0509_vision_lab/scripts/apply_jazzy_patch.sh ~/doosan_ws/src

# 3) 빌드 (dsr_msgs2 먼저, 항상 --symlink-install)
cd ~/doosan_ws
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy
colcon build --symlink-install --packages-select dsr_msgs2 && source install/setup.bash
colcon build --symlink-install && source install/setup.bash

# 4) 에뮬레이터 (Docker 필요, 처음 한 번)
cd ~/doosan_ws/src/doosan-robot2 && sudo ./install_emulator.sh
```

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "source ~/doosan_ws/install/setup.bash" >> ~/.bashrc
```

## 실행

```bash
# 로봇 (가상)
ros2 launch e0509_gripper_description bringup.launch.py mode:=virtual
# 로봇 (실제)
ros2 launch e0509_gripper_description bringup.launch.py mode:=real host:=<로봇IP> rt_host:=<로봇IP>
# Gazebo
ros2 launch e0509_gripper_description gazebo.launch.py

# 예제
ros2 run e0509_gripper_demo ex01_gripper_sequence
ros2 run e0509_gripper_demo ex02_move_joint
ros2 run e0509_gripper_demo ex03_move_line
ros2 run e0509_gripper_demo ex04_pick_and_place
ros2 run e0509_gripper_demo ex05_state_monitor

# 카메라 (bringup 실행 중)
ros2 launch e0509_camera_lab camera_lab.launch.py                    # 가상 RealSense
ros2 launch e0509_camera_lab camera_lab.launch.py camera:=realsense  # 실제 RealSense
ros2 run e0509_camera_lab click_to_base
ros2 run e0509_camera_lab eye_to_hand_calib                          # 가상: 자동 14자세
ros2 run e0509_camera_lab eye_to_hand_calib --ros-args -p mode:=manual -p marker_size:=0.10   # 실로봇
```

캘리브레이션 결과: `~/doosan_ws/eye_to_hand.yaml` (TF), `~/doosan_ws/calibration_eye_to_hand.npz` (`T_cam_to_base`)

## Launch 파일

| launch | 용도 |
| --- | --- |
| `display.launch.py` | 모델 + 슬라이더 + RViz |
| `bringup.launch.py` | 에뮬레이터/실로봇 + 제어기 + 그리퍼 노드 + RViz |
| `gazebo.launch.py` | Gazebo 단독 시뮬레이션 |
| `rviz_gazebo.launch.py` | RViz 슬라이더 → Gazebo |
| `bringup_real_gazebo.launch.py` | 실로봇 ↔ Gazebo 디지털 트윈 (실로봇 필요) |
| `curobo_vision.launch.py` | RealSense + Grounding DINO + cuRobo (원본 참고용, Humble/CUDA 경로 하드코딩) |

## 출처

`e0509_gripper_description` 은 [fhekwn549/e0509_gripper_description](https://github.com/fhekwn549/e0509_gripper_description) (commit `ccee97a`, Apache-2.0) 를 Jazzy 에 맞게 수정한 것입니다. 수정 내용은 `docs/JAZZY_CHANGES.md` 에 있습니다.
