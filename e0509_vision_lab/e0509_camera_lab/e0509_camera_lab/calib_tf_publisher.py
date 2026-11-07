#!/usr/bin/env python3
"""
base_link -> camera_link 정적 TF 발행기 (eye-to-hand 결과를 TF 트리에 넣는 노드)

  1) 캘리브레이션 파일(~/doosan_ws/eye_to_hand.yaml)이 있으면 그 값을 사용
  2) 없으면 패키지의 초기 추정값(config/eye_to_hand_initial.yaml, 줄자로 잰 대충 값) 사용
  3) 파일이 바뀌면(캘리브레이션 완료) 자동으로 다시 읽어서 TF 갱신  ← 재실행 불필요

yaml 형식:  x, y, z [m]  +  (qx, qy, qz, qw)  또는  (roll_deg, pitch_deg, yaw_deg)
"""
import math
import os

import yaml
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
from ament_index_python.packages import get_package_share_directory

from e0509_camera_lab import geometry as g
from e0509_camera_lab import rosutil as ru


def load_calib(path):
    with open(path) as f:
        d = yaml.safe_load(f)
    xyz = [float(d['x']), float(d['y']), float(d['z'])]
    if all(k in d for k in ('qx', 'qy', 'qz', 'qw')):
        q = [float(d[k]) for k in ('qx', 'qy', 'qz', 'qw')]
    else:
        R = g.rpy_to_R(*[math.radians(float(d[k])) for k in ('roll_deg', 'pitch_deg', 'yaw_deg')])
        q = list(g.R_to_quat(R))
    return xyz, q, d


class CalibTF(Node):
    def __init__(self):
        super().__init__('calib_tf_publisher')
        self.declare_parameter('calib_file', '~/doosan_ws/eye_to_hand.yaml')
        self.declare_parameter('initial_file', os.path.join(
            get_package_share_directory('e0509_camera_lab'), 'config', 'eye_to_hand_initial.yaml'))
        self.calib = os.path.expanduser(self.get_parameter('calib_file').value)
        self.initial = os.path.expanduser(self.get_parameter('initial_file').value)
        self.br = StaticTransformBroadcaster(self)
        self.loaded = None
        self.create_timer(1.0, self.check)
        self.check()

    def check(self):
        path = self.calib if os.path.exists(self.calib) else self.initial
        key = (path, os.path.getmtime(path))
        if key == self.loaded:
            return
        try:
            xyz, q, d = load_calib(path)
        except Exception as e:
            self.get_logger().error(f'{path} 읽기 실패: {e}')
            return
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = ru.BASE_FRAME
        t.child_frame_id = ru.CAMERA_LINK
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = xyz
        (t.transform.rotation.x, t.transform.rotation.y,
         t.transform.rotation.z, t.transform.rotation.w) = q
        self.br.sendTransform(t)
        self.loaded = key
        tag = '캘리브레이션 결과' if path == self.calib else '초기 추정값 (캘리브레이션 전)'
        self.get_logger().info(
            f'[{tag}] base_link -> camera_link  xyz=({xyz[0]:.3f}, {xyz[1]:.3f}, {xyz[2]:.3f}) m   ← {path}')


def main():
    rclpy.init()
    node = CalibTF()
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
