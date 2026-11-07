#!/usr/bin/env python3
"""
예제 5. 로봇 상태 모니터 (토픽 구독 + 주기적 서비스 호출)

배울 것
  - /dsr01/joint_states 에는 팔 6축 + 그리퍼 4축이 함께 들어온다
    (gripper_joint_publisher 가 둘을 합쳐서 50 Hz 로 발행)
  - 그리퍼 관절각 [rad] ↔ stroke 관계: angle = stroke / 700
  - 콜백 안에서는 서비스를 call_async 로 '기다리지 않고' 호출해야 한다
  - 주의: stroke 는 '명령값'이다. 실제 그리퍼 위치를 읽은 값이 아님

다른 예제(ex02~ex04)를 실행하면서 옆 터미널에서 켜 두면 값이 바뀌는 게 보인다.

실행:  ros2 run e0509_gripper_demo ex05_state_monitor
"""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from dsr_msgs2.srv import GetCurrentPosx

ARM = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
GRIPPER = 'gripper_rh_r1'


class StateMonitor(Node):
    def __init__(self):
        super().__init__('ex05_state_monitor')
        self.declare_parameter('namespace', 'dsr01')
        self.declare_parameter('rate', 2.0)   # 화면 출력 주기 [Hz]
        ns = self.get_parameter('namespace').value

        self.arm_deg = None
        self.stroke = None
        self.tcp = None
        self.pending = None

        self.create_subscription(JointState, f'/{ns}/joint_states', self.on_joints, 10)
        self.cli_posx = self.create_client(GetCurrentPosx, f'/{ns}/aux_control/get_current_posx')
        self.create_timer(1.0 / float(self.get_parameter('rate').value), self.on_timer)
        self.get_logger().info(f'/{ns}/joint_states 구독 시작 (Ctrl+C 로 종료)')

    def on_joints(self, msg):
        pos = dict(zip(msg.name, msg.position))
        if all(j in pos for j in ARM):
            self.arm_deg = [math.degrees(pos[j]) for j in ARM]
        if GRIPPER in pos:                         # 그리퍼가 들어 있는 메시지만
            self.stroke = round(pos[GRIPPER] * 700)

    def on_timer(self):
        # TCP 는 서비스로 읽는다. 콜백 안이므로 응답을 기다리지 않고 다음 주기에 반영
        if self.cli_posx.service_is_ready() and self.pending is None:
            req = GetCurrentPosx.Request()
            req.ref = 0
            self.pending = self.cli_posx.call_async(req)
            self.pending.add_done_callback(self.on_tcp)

        if self.arm_deg is None:
            self.get_logger().info('joint_states 대기 중...')
            return
        joints = ' '.join(f'{v:7.1f}' for v in self.arm_deg)
        tcp = ' '.join(f'{v:7.1f}' for v in self.tcp[:3]) if self.tcp else '   (읽는 중)'
        grip = f'{self.stroke:3d}' if self.stroke is not None else '  ?'
        self.get_logger().info(f'J[deg] {joints} | TCP xyz[mm] {tcp} | stroke {grip}')

    def on_tcp(self, future):
        self.pending = None
        res = future.result()
        if res is not None and res.success and res.task_pos_info:
            self.tcp = list(res.task_pos_info[0].data[:6])


def main():
    rclpy.init()
    node = StateMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
