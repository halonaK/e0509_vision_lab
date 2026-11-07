# Jazzy 에서 바꿔야 하는 것 (재설치 체크리스트)

이 저장소에는 아래 수정이 모두 반영되어 있습니다. 원본 저장소로 처음부터 다시 설치할 때 이 목록만 확인하면 됩니다.

## 1. doosan-robot2

| 항목 | 내용 |
| --- | --- |
| 브랜치 | 공식 `jazzy` 사용 (원본 README 의 humble 포크 사용하지 않음) |
| Flange Serial 서비스 | `scripts/apply_jazzy_patch.sh` 실행. `dsr_msgs2/srv/FlangeSerial{Open,Close,Write,Read}.srv`, `dsr_msgs2/CMakeLists.txt`, `dsr_controller2` (hpp, cpp) 에 추가됨 |
| 확인 | `grep -c FlangeSerial doosan-robot2/dsr_msgs2/CMakeLists.txt` → 4 |

`git pull` / `git checkout` 으로 doosan-robot2 를 갱신하면 패치가 사라지므로 다시 실행합니다.

## 2. e0509_gripper_description (원본 대비 변경)

| 파일 | 변경 | 이유 |
| --- | --- | --- |
| `urdf/e0509_with_gripper.urdf.xacro` | `ign_ros2_control/IgnitionSystem` → `gz_ros2_control/GazeboSimSystem` | Jazzy Gazebo = Harmonic |
| 같은 파일 | 그리퍼 ros2_control 의 `gripper_rh_r2`, `gripper_rh_l1`, `gripper_rh_l2` 에 `mimic="false"` | Jazzy 는 mimic 관절에 명령 인터페이스가 있으면 하드웨어 전체 로드 실패 → Gazebo 에서 로봇이 쓰러짐 |
| `config/gz_controllers.yaml` | 각 컨트롤러에 `type:` 추가 | Jazzy spawner 는 `--param-file` 에서 타입을 읽음 |
| `launch/gazebo.launch.py`, `rviz_gazebo.launch.py`, `bringup_real_gazebo.launch.py` | spawner 에 `--param-file gz_controllers.yaml`, 환경변수 `GZ_SIM_RESOURCE_PATH` | Jazzy 두산 Gazebo 플러그인은 두산 컨트롤러 yaml 만 읽음 |
| `launch/gazebo.launch.py` | 컨트롤러 시작 후 홈 자세 명령 | 시작 직후 처진 팔을 세움 |
| `launch/rviz_gazebo.launch.py`, `bringup_real_gazebo.launch.py` | `gazebo_bridge.py` → C++ `gazebo_bridge`, `gripper_*.py` → C++ 이름 | 원본이 C++ 로 바꾸면서 .py 는 설치되지 않음 |
| `launch/bringup_dsr.launch.py`, `bringup_gazebo.launch.py` | 삭제 | 단독 사용 불가 / 브릿지 없음 |

## 3. 환경

| 항목 | 내용 |
| --- | --- |
| `~/.bashrc` | `/opt/ros/jazzy` 와 `~/doosan_ws/install` 만 source. 두산 패키지가 든 다른 워크스페이스가 함께 로드되면 Flange 없는 `dsr_msgs2` 를 읽음 |
| 빌드 옵션 | 항상 `--symlink-install` (섞어 쓰면 설치 단계 충돌) |
| 빌드 순서 | `dsr_msgs2` 먼저 빌드 → `source` → 전체 |
| Docker | 에뮬레이터용. `sudo usermod -aG docker $USER` 후 재로그인 |
