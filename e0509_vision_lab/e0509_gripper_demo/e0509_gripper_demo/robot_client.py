"""
예제 공통 헬퍼: 두산 E0509 + RH-P12-RN-A 를 '한 줄 명령'으로 쓰기 위한 클래스

    robot = RobotClient()                      # ROS 노드 생성 + 서비스 연결
    robot.move_joint([0, 0, 90, 0, 90, 0])     # 관절 이동 [deg]
    robot.move_line([0, 0, -100, 0, 0, 0], relative=True)   # TCP 직선 이동 [mm, deg]
    robot.gripper_close()                      # 그리퍼 닫기
    print(robot.get_tcp())                     # 현재 TCP [x, y, z, rx, ry, rz]

모든 함수는 동작이 끝날 때까지 기다렸다가(blocking) 돌아옵니다.
그래서 예제 코드는 위에서 아래로 '순서대로' 읽으면 됩니다.

사용하는 인터페이스 (bringup.launch.py 가 띄워 줌)
  /dsr01/motion/move_joint        dsr_msgs2/srv/MoveJoint
  /dsr01/motion/move_line         dsr_msgs2/srv/MoveLine
  /dsr01/aux_control/get_current_posx  dsr_msgs2/srv/GetCurrentPosx
  /dsr01/gripper/open, close      std_srvs/srv/Trigger
  /dsr01/gripper/position_cmd     std_msgs/msg/Int32  (0=열림 ~ 700=닫힘)
"""
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from std_srvs.srv import Trigger
from dsr_msgs2.srv import MoveJoint, MoveLine, GetCurrentPosx

# 자주 쓰는 자세 [deg]
HOME = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]          # 똑바로 선 자세
READY = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]       # 그리퍼가 아래를 보는 작업 자세

STROKE_OPEN = 0
STROKE_CLOSE = 700


class RobotClient(Node):
    def __init__(self, name='robot_client', namespace='dsr01', timeout=60.0):
        super().__init__(name)
        self.declare_parameter('namespace', namespace)
        ns = self.get_parameter('namespace').value
        self.timeout = timeout

        self.cli_movej = self.create_client(MoveJoint, f'/{ns}/motion/move_joint')
        self.cli_movel = self.create_client(MoveLine, f'/{ns}/motion/move_line')
        self.cli_posx = self.create_client(GetCurrentPosx, f'/{ns}/aux_control/get_current_posx')
        self.cli_open = self.create_client(Trigger, f'/{ns}/gripper/open')
        self.cli_close = self.create_client(Trigger, f'/{ns}/gripper/close')
        self.pub_grip = self.create_publisher(Int32, f'/{ns}/gripper/position_cmd', 10)

        for cli in (self.cli_movej, self.cli_movel, self.cli_posx, self.cli_open, self.cli_close):
            while not cli.wait_for_service(timeout_sec=2.0):
                self.get_logger().warn(
                    f'{cli.srv_name} 대기 중... (bringup.launch.py 실행 확인)')
        self.get_logger().info('로봇 서비스 연결 완료')

    # ------------------------------------------------------------ 내부
    def _call(self, client, request, what):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.timeout)
        res = future.result()
        if res is None:
            raise RuntimeError(f'{what}: 응답 없음 ({self.timeout}s 초과)')
        return res

    # ------------------------------------------------------------ 팔
    def move_joint(self, pos_deg, vel=30.0, acc=30.0):
        """관절 6개 각도로 이동 [deg]. 끝날 때까지 기다림."""
        req = MoveJoint.Request()
        req.pos = [float(v) for v in pos_deg]
        req.vel, req.acc = float(vel), float(acc)
        req.mode = 0        # 절대 위치
        req.sync_type = 0   # 동작 완료 후 응답
        self.get_logger().info(f'move_joint {req.pos}')
        res = self._call(self.cli_movej, req, 'move_joint')
        if not res.success:
            raise RuntimeError('move_joint 실패 (도달 불가 자세이거나 로봇 상태 확인)')

    def move_line(self, pos, vel=100.0, acc=100.0, relative=False):
        """TCP 직선 이동. pos = [x, y, z, rx, ry, rz] (mm, deg), 로봇 베이스 기준.
        relative=True 면 현재 위치에서 그만큼 더 이동."""
        req = MoveLine.Request()
        req.pos = [float(v) for v in pos]
        req.vel = [float(vel), 30.0]      # [mm/s, deg/s]
        req.acc = [float(acc), 60.0]      # [mm/s^2, deg/s^2]
        req.ref = 0                       # DR_BASE
        req.mode = 1 if relative else 0   # 0=절대, 1=상대
        req.sync_type = 0
        self.get_logger().info(f'move_line {"상대" if relative else "절대"} {req.pos}')
        res = self._call(self.cli_movel, req, 'move_line')
        if not res.success:
            raise RuntimeError('move_line 실패 (작업 영역 밖이거나 특이점 근처)')

    def get_tcp(self):
        """현재 TCP 위치 [x, y, z, rx, ry, rz] (mm, deg)"""
        req = GetCurrentPosx.Request()
        req.ref = 0
        res = self._call(self.cli_posx, req, 'get_current_posx')
        if not res.success or not res.task_pos_info:
            raise RuntimeError('get_current_posx 실패')
        return list(res.task_pos_info[0].data[:6])

    # ------------------------------------------------------------ 그리퍼
    def gripper_open(self):
        self.get_logger().info('gripper open')
        self._call(self.cli_open, Trigger.Request(), 'gripper/open')

    def gripper_close(self):
        self.get_logger().info('gripper close')
        self._call(self.cli_close, Trigger.Request(), 'gripper/close')

    def gripper_set(self, stroke, wait=1.0):
        """0(열림) ~ 700(닫힘). 토픽이라 응답이 없으므로 wait 초 기다림."""
        stroke = max(STROKE_OPEN, min(STROKE_CLOSE, int(stroke)))
        self.get_logger().info(f'gripper position {stroke}')
        self.pub_grip.publish(Int32(data=stroke))
        self.sleep(wait)

    # ------------------------------------------------------------ 기타
    def sleep(self, sec):
        """콜백을 처리하면서 기다리기 (time.sleep 대신 사용)"""
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)


def run(example_main, node_name):
    """예제 공통 실행 틀: init → 예제 함수 → 정리"""
    rclpy.init()
    robot = RobotClient(node_name)
    try:
        example_main(robot)
        robot.get_logger().info('예제 완료')
    except KeyboardInterrupt:
        pass
    except RuntimeError as e:
        robot.get_logger().error(str(e))
    finally:
        robot.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
