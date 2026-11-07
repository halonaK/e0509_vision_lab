#!/usr/bin/env python3
"""
RH-P12-RN-A 그리퍼 데모 노드

steps 파라미터에 적힌 동작을 interval 초 간격으로 순서대로 실행합니다.
  - "open"  : /<ns>/gripper/open  서비스 호출 (stroke 0)
  - "close" : /<ns>/gripper/close 서비스 호출 (stroke 700)
  - 숫자    : /<ns>/gripper/position_cmd 토픽 발행 (0 = 열림 ~ 700 = 닫힘)

/<ns>/gripper/stroke 를 구독해서 현재 값을 로그로 보여줍니다.
virtual / real 모드 모두 같은 코드로 동작합니다.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from std_srvs.srv import Trigger

STROKE_MIN = 0
STROKE_MAX = 700


class GripperDemo(Node):
    def __init__(self):
        super().__init__('gripper_demo')

        # ---------- 파라미터 ----------
        self.declare_parameter('namespace', 'dsr01')
        self.declare_parameter('steps', ['open', '350', 'close', 'open'])
        self.declare_parameter('interval', 2.0)   # 동작 사이 간격 [s]
        self.declare_parameter('loop', False)     # True면 steps 반복

        ns = self.get_parameter('namespace').value
        self.steps = list(self.get_parameter('steps').value)
        self.interval = float(self.get_parameter('interval').value)
        self.loop = bool(self.get_parameter('loop').value)
        prefix = f'/{ns}/gripper'

        # ---------- 통신 ----------
        self.pos_pub = self.create_publisher(Int32, f'{prefix}/position_cmd', 10)
        self.cli_open = self.create_client(Trigger, f'{prefix}/open')
        self.cli_close = self.create_client(Trigger, f'{prefix}/close')
        self.create_subscription(Int32, f'{prefix}/stroke', self.on_stroke, 10)

        self.index = 0
        self.busy = False       # 서비스 응답 대기 중이면 다음 단계로 넘어가지 않음
        self.last_stroke = None

        self.get_logger().info(f'그리퍼 서비스 대기: {prefix}/open, {prefix}/close')
        for cli in (self.cli_open, self.cli_close):
            while not cli.wait_for_service(timeout_sec=2.0):
                self.get_logger().warn(
                    f'{cli.srv_name} 없음 - bringup.launch.py 가 실행 중인지 확인하세요')

        self.get_logger().info(
            f'시작: steps={self.steps}, interval={self.interval}s, loop={self.loop}')
        self.run_step()                                   # 첫 동작은 바로
        self.timer = self.create_timer(self.interval, self.run_step)

    # ---------------------------------------------------------------
    def run_step(self):
        if self.busy:
            return
        if self.index >= len(self.steps):
            if self.loop:
                self.index = 0
            else:
                self.get_logger().info('모든 동작 완료 - 노드를 종료합니다')
                self.timer.cancel()
                raise SystemExit
        step = str(self.steps[self.index]).strip().lower()
        self.index += 1

        if step == 'open':
            self.call(self.cli_open, 'open')
        elif step == 'close':
            self.call(self.cli_close, 'close')
        else:
            try:
                value = int(step)
            except ValueError:
                self.get_logger().error(f'알 수 없는 단계 "{step}" - 건너뜀')
                return
            value = max(STROKE_MIN, min(STROKE_MAX, value))
            self.pos_pub.publish(Int32(data=value))
            self.get_logger().info(f'[{self.index}/{len(self.steps)}] position_cmd = {value}')

    def call(self, client, name):
        self.busy = True
        self.get_logger().info(f'[{self.index}/{len(self.steps)}] {name} 요청')
        future = client.call_async(Trigger.Request())
        future.add_done_callback(lambda f: self.on_response(f, name))

    def on_response(self, future, name):
        self.busy = False
        res = future.result()
        if res is None:
            self.get_logger().error(f'{name} 응답 없음')
        else:
            # virtual 모드에서는 하드웨어 통신이 없어 success=False 가 올 수 있음 (RViz는 정상 동작)
            self.get_logger().info(f'{name} 결과: success={res.success}, {res.message}')

    def on_stroke(self, msg):
        if msg.data != self.last_stroke:
            self.last_stroke = msg.data
            self.get_logger().info(f'현재 stroke = {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    node = GripperDemo()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
