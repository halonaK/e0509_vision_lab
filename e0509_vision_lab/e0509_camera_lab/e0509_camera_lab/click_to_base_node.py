#!/usr/bin/env python3
"""
실습 1~3단계: 클릭한 픽셀 -> 카메라 3D 좌표 -> 로봇 base 좌표

  카메라 영상 창에서 아무 곳이나 클릭하면 터미널에 단계별 계산이 출력됩니다.
    ① 픽셀 (u, v) 와 depth Z
    ② 역투영:  X = (u - cx)·Z/fx,  Y = (v - cy)·Z/fy        (camera_color_optical_frame)
    ③ TF 변환:  p_base = T_base_cam · p_cam                 (base_link)
    ④ (가상 카메라일 때) 정답과의 오차

  창에서 'q' 를 누르면 종료.  RViz 에는 /clicked_point_base 마커로 표시됩니다.
"""
import json

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker
from tf2_ros import Buffer, TransformListener

from e0509_camera_lab import geometry as g
from e0509_camera_lab import rosutil as ru

WIN = 'click_to_base  (click = point, q = quit)'


class ClickToBase(Node):
    def __init__(self):
        super().__init__('click_to_base')
        self.color = self.depth = self.info = None
        self.T_true_opt = None
        self.clicks = []
        self.create_subscription(Image, ru.COLOR_TOPIC, lambda m: setattr(self, 'color', m), 2)
        self.create_subscription(Image, ru.DEPTH_TOPIC, lambda m: setattr(self, 'depth', m), 2)
        self.create_subscription(CameraInfo, ru.COLOR_INFO_TOPIC, self.on_info, 2)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/virtual_scene/truth', self.on_truth, latched)
        self.pub_pt = self.create_publisher(PointStamped, '/clicked_point_base', 5)
        self.pub_mk = self.create_publisher(Marker, '/clicked_point_marker', 5)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        cv2.namedWindow(WIN)
        cv2.setMouseCallback(WIN, self.on_mouse)
        self.create_timer(0.05, self.on_timer)
        self.get_logger().info('카메라 영상 창에서 점을 클릭하세요 (q: 종료)')

    def on_info(self, msg):
        if self.info is None:
            K, _, _ = ru.info_to_K(msg)
            self.get_logger().info(
                f'카메라 내부 파라미터 K (camera_info):\n'
                f'   fx={K[0, 0]:.1f}  fy={K[1, 1]:.1f}  cx={K[0, 2]:.1f}  cy={K[1, 2]:.1f}  '
                f'({msg.width}x{msg.height})')
        self.info = msg

    def on_truth(self, msg):
        T_link = np.array(json.loads(msg.data)['camera_link_in_base'])
        self.T_true_opt = T_link @ g.T_LINK_OPTICAL

    def on_mouse(self, event, u, v, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.compute(u, v)

    # ------------------------------------------------------------ 핵심 계산
    def compute(self, u, v):
        if self.depth is None or self.info is None:
            self.get_logger().warn('depth / camera_info 를 아직 받지 못했습니다')
            return
        depth = ru.image_to_np(self.depth)
        _, _, K = ru.info_to_K(self.info)
        win = depth[max(0, v - 2):v + 3, max(0, u - 2):u + 3]
        valid = win[win > 0]
        if valid.size == 0:
            self.get_logger().warn(f'({u}, {v}) 에 depth 값이 없습니다 (0 = 측정 실패)')
            return
        Z = float(np.median(valid)) / 1000.0            # mm -> m, 5x5 중앙값으로 노이즈 완화

        p_cam = g.deproject(u, v, Z, K['fx'], K['fy'], K['cx'], K['cy'])
        lines = [
            '',
            f'① 픽셀 (u, v) = ({u}, {v})     depth Z = {Z * 1000:.0f} mm  (5x5 중앙값)',
            f'② 역투영  X = (u - cx)·Z/fx = ({u} - {K["cx"]:.1f})·{Z:.3f}/{K["fx"]:.1f} = {p_cam[0]:+.4f} m',
            f'          Y = (v - cy)·Z/fy = ({v} - {K["cy"]:.1f})·{Z:.3f}/{K["fy"]:.1f} = {p_cam[1]:+.4f} m',
            f'   p_cam (camera_color_optical_frame) = [{p_cam[0]:+.4f}, {p_cam[1]:+.4f}, {p_cam[2]:+.4f}] m',
        ]
        try:
            tf = self.tf_buffer.lookup_transform(ru.BASE_FRAME, ru.OPTICAL_FRAME, rclpy.time.Time())
        except Exception as e:
            lines.append(f'③ TF base_link <- {ru.OPTICAL_FRAME} 없음: {e}')
            self.get_logger().info('\n'.join(lines))
            return
        T = g.transform_to_T(tf.transform)
        p_base = g.apply_T(T, p_cam)
        lines.append(f'③ p_base = T_base_cam · p_cam = [{p_base[0]:+.4f}, {p_base[1]:+.4f}, {p_base[2]:+.4f}] m'
                     f'   (= [{p_base[0] * 1000:.0f}, {p_base[1] * 1000:.0f}, {p_base[2] * 1000:.0f}] mm)')
        if self.T_true_opt is not None:
            p_true = g.apply_T(self.T_true_opt, p_cam)
            err = np.linalg.norm(p_base - p_true) * 1000
            lines.append(f'④ 정답 p_true = [{p_true[0]:+.4f}, {p_true[1]:+.4f}, {p_true[2]:+.4f}] m'
                         f'   →  오차 {err:.1f} mm  (카메라 TF 가 정확할수록 0 에 가까움)')
        self.get_logger().info('\n'.join(lines))
        self.clicks = (self.clicks + [(u, v)])[-5:]
        self.publish(p_base)

    def publish(self, p):
        pt = PointStamped()
        pt.header.frame_id = ru.BASE_FRAME
        pt.header.stamp = self.get_clock().now().to_msg()
        pt.point.x, pt.point.y, pt.point.z = map(float, p)
        self.pub_pt.publish(pt)
        m = Marker()
        m.header = pt.header
        m.ns, m.id, m.type, m.action = 'click', 0, Marker.SPHERE, Marker.ADD
        m.pose.position = pt.point
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.02
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.0, 1.0, 1.0
        self.pub_mk.publish(m)

    def on_timer(self):
        if self.color is not None:
            img = ru.image_to_np(self.color)
            for (u, v) in self.clicks:
                cv2.drawMarker(img, (u, v), (255, 0, 255), cv2.MARKER_CROSS, 14, 2)
            cv2.imshow(WIN, img)
        if (cv2.waitKey(1) & 0xFF) == ord('q'):
            raise KeyboardInterrupt


def main():
    rclpy.init()
    node = ClickToBase()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
