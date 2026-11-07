#!/usr/bin/env python3
"""
가상 RealSense (eye-to-hand, 로봇 앞에 고정된 RGB-D 카메라)

실제 realsense2_camera 와 같은 토픽을 내보내므로, 나중에 실제 카메라로 바꿔도
나머지 노드는 그대로 동작합니다.

  발행  /camera/camera/color/image_raw                     bgr8
        /camera/camera/color/camera_info
        /camera/camera/aligned_depth_to_color/image_raw     16UC1 [mm]
        /camera/camera/aligned_depth_to_color/camera_info
        TF  camera_link -> camera_color_optical_frame      (카메라 내부 구조, 고정)
        /virtual_scene/markers   정답 물체 위치 (RViz 확인용)
        /virtual_scene/truth     정답 JSON (카메라 실제 위치, 물체 위치) - 채점용
  구독  /virtual_scene/cmd       "attach red" / "detach red" / "reset"

주의: base_link -> camera_link TF 는 이 노드가 내보내지 않습니다.
      그 값은 '학생이 측정/캘리브레이션해서 알아내야 하는 값'입니다 (calib_tf_publisher).
"""
import json

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener, StaticTransformBroadcaster
from sensor_msgs.msg import Image, CameraInfo

from e0509_camera_lab import geometry as g
from e0509_camera_lab import scene as sc
from e0509_camera_lab import aruco_compat as ac
from e0509_camera_lab import rosutil as ru

COLORS = {'red': (40, 40, 210), 'green': (60, 180, 60), 'yellow': (40, 210, 230),
          'blue': (200, 110, 40), 'orange': (30, 130, 240)}

# 그리퍼 마커 장착 위치 (link_6 기준). 카메라 쪽(-x_link6)을 향하고 45도 위로 기울어진 판
_n = np.array([-1.0, 0.0, -1.0]) / np.sqrt(2.0)
_x = np.array([0.0, 1.0, 0.0])
T_FLANGE_MARKER = np.eye(4)
T_FLANGE_MARKER[:3, 0], T_FLANGE_MARKER[:3, 1], T_FLANGE_MARKER[:3, 2] = _x, np.cross(_n, _x), _n
T_FLANGE_MARKER[:3, 3] = [-0.07, 0.0, 0.06]


class VirtualCamera(Node):
    def __init__(self):
        super().__init__('virtual_camera')
        P = self.declare_parameter
        P('camera_xyz', [1.0, 0.35, 0.75])            # 정답 카메라 위치 (base_link 기준)
        P('camera_look_at', [0.42, 0.0, 0.08])        # 카메라가 바라보는 점
        P('objects', ['red 0.45 -0.15', 'green 0.38 0.00', 'yellow 0.52 0.12'])
        P('object_size', 0.04)                        # 정육면체 한 변 [m]
        P('pad', 'blue 0.30 0.24')                    # 놓을 자리 (얇은 판)
        P('pad_size', [0.10, 0.18])
        P('marker_enabled', True)
        P('marker_id', 0)
        P('marker_size', 0.10)                        # 검은 마커 한 변 [m]
        P('tcp_offset', 0.13)                         # link_6 -> 손가락 끝 [m]
        P('fps', 10.0)
        P('depth_noise', True)
        gp = lambda k: self.get_parameter(k).value

        cam = np.array(gp('camera_xyz'), float)
        r, p, y = g.look_at_rpy(cam, gp('camera_look_at'))
        self.T_true = g.make_T(g.rpy_to_R(r, p, y), cam)      # base_link -> camera_link (정답)
        self.K = sc.intrinsics()
        self.depth_noise = gp('depth_noise')
        self.tcp_offset = gp('tcp_offset')

        s = gp('object_size')
        self.init_objects = []
        for spec in gp('objects'):
            name, x, yy = spec.split()[:3]
            self.init_objects.append((name, [float(x), float(yy), s / 2], [s, s, s]))
        pname, px, py = gp('pad').split()[:3]
        pw = gp('pad_size')
        self.init_pad = (pname + '_pad', [float(px), float(py), 0.0025], [pw[0], pw[1], 0.005])
        self.reset_scene()

        self.board = None
        if gp('marker_enabled'):
            mk = ac.marker_image(gp('marker_id'), 200)
            tex = np.full((280, 280), 255, np.uint8)
            tex[40:240, 40:240] = mk
            ms = gp('marker_size')
            self.board = sc.MarkerBoard(tex, ms, ms * 1.4)

        # ---------- ROS
        self.pub_color = self.create_publisher(Image, ru.COLOR_TOPIC, 5)
        self.pub_cinfo = self.create_publisher(CameraInfo, ru.COLOR_INFO_TOPIC, 5)
        self.pub_depth = self.create_publisher(Image, ru.DEPTH_TOPIC, 5)
        self.pub_dinfo = self.create_publisher(CameraInfo, ru.DEPTH_INFO_TOPIC, 5)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_truth = self.create_publisher(String, '/virtual_scene/truth', latched)
        self.pub_markers = self.create_publisher(MarkerArray, '/virtual_scene/markers', 5)
        self.create_subscription(String, '/virtual_scene/cmd', self.on_cmd, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.static_tf = StaticTransformBroadcaster(self)
        self.publish_optical_tf()

        self.cache = None
        self.last_key = None
        self.create_timer(1.0 / gp('fps'), self.on_timer)
        self.create_timer(1.0, self.publish_truth)
        self.get_logger().info(
            f'가상 RealSense 시작: {self.K["width"]}x{self.K["height"]}, fx={self.K["fx"]}, '
            f'물체 {[o.name for o in self.boxes]}')
        self.get_logger().info('base_link -> camera_link TF 는 calib_tf_publisher 가 담당합니다')

    # ------------------------------------------------------------ 장면
    def reset_scene(self):
        self.boxes = [sc.Box(n, c, sz, COLORS.get(n, (180, 180, 180))) for n, c, sz in self.init_objects]
        n, c, sz = self.init_pad
        self.pad = sc.Box(n, c, sz, COLORS.get(n.replace('_pad', ''), (200, 110, 40)))
        self.attached = {}          # name -> link_6 기준 오프셋
        self.cache = None

    def find_box(self, name):
        return next((b for b in self.boxes if b.name == name), None)

    def on_cmd(self, msg):
        parts = msg.data.split()
        if not parts:
            return
        cmd, name = parts[0], (parts[1] if len(parts) > 1 else '')
        if cmd == 'reset':
            self.reset_scene()
            self.get_logger().info('장면 초기화')
            return
        b = self.find_box(name)
        T6 = self.flange_T()
        if b is None or T6 is None:
            self.get_logger().warn(f'명령 무시: {msg.data} (물체 없음 또는 link_6 TF 없음)')
            return
        if cmd == 'attach':
            self.attached[name] = g.apply_T(g.inv_T(T6), b.center)
            self.get_logger().info(f'{name} 를 그리퍼에 붙임')
        elif cmd == 'detach' and name in self.attached:
            c = g.apply_T(T6, self.attached.pop(name))
            on_pad = (abs(c[0] - self.pad.center[0]) < self.pad.size[0] / 2 and
                      abs(c[1] - self.pad.center[1]) < self.pad.size[1] / 2)
            support = self.pad.top_z if on_pad else 0.0
            b.center = np.array([c[0], c[1], support + b.size[2] / 2])
            self.get_logger().info(f'{name} 내려놓음 ({c[0]:.3f}, {c[1]:.3f})')
        self.cache = None

    def flange_T(self):
        try:
            tf = self.tf_buffer.lookup_transform(ru.BASE_FRAME, ru.FLANGE_FRAME, rclpy.time.Time())
            return g.transform_to_T(tf.transform)
        except Exception:
            return None

    # ------------------------------------------------------------ 발행
    def publish_optical_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = ru.CAMERA_LINK
        t.child_frame_id = ru.OPTICAL_FRAME
        qx, qy, qz, qw = g.R_to_quat(g.R_LINK_OPTICAL)
        t.transform.rotation.x, t.transform.rotation.y = qx, qy
        t.transform.rotation.z, t.transform.rotation.w = qz, qw
        self.static_tf.sendTransform(t)

    def on_timer(self):
        T6 = self.flange_T()
        for name, off in self.attached.items():
            b = self.find_box(name)
            if b is not None and T6 is not None:
                b.center = g.apply_T(T6, off)
        if self.board is not None:
            self.board.T_base_board = None if T6 is None else T6 @ T_FLANGE_MARKER

        key = None if T6 is None else tuple(np.round(T6[:3, :].ravel(), 4))
        if self.cache is None or key != self.last_key:     # 로봇이 움직였을 때만 다시 그림
            color, depth = sc.render(self.T_true, self.K, self.boxes + [self.pad], self.board,
                                     depth_noise=self.depth_noise)
            self.cache, self.last_key = (color, depth), key

        stamp = self.get_clock().now().to_msg()
        color, depth = self.cache
        self.pub_color.publish(ru.np_to_image(color, 'bgr8', ru.OPTICAL_FRAME, stamp))
        self.pub_depth.publish(ru.np_to_image(depth, '16UC1', ru.OPTICAL_FRAME, stamp))
        info = ru.make_camera_info(self.K, ru.OPTICAL_FRAME, stamp)
        self.pub_cinfo.publish(info)
        self.pub_dinfo.publish(info)
        self.publish_markers()

    def publish_truth(self):
        data = {'camera_link_in_base': self.T_true.tolist(),
                'objects': {b.name: [float(b.center[0]), float(b.center[1]), b.top_z]
                            for b in self.boxes + [self.pad]}}
        self.pub_truth.publish(String(data=json.dumps(data)))

    def publish_markers(self):
        arr = MarkerArray()
        for i, b in enumerate(self.boxes + [self.pad]):
            m = Marker()
            m.header.frame_id = ru.BASE_FRAME
            m.ns, m.id, m.type, m.action = 'truth', i, Marker.CUBE, Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, b.center)
            m.pose.orientation.w = 1.0
            m.scale.x, m.scale.y, m.scale.z = map(float, b.size)
            bgr = b.bgr / 255.0
            m.color.r, m.color.g, m.color.b, m.color.a = float(bgr[2]), float(bgr[1]), float(bgr[0]), 1.0
            arr.markers.append(m)
        cam = Marker()                       # 정답 카메라 위치 (회색 화살표)
        cam.header.frame_id = ru.BASE_FRAME
        cam.ns, cam.id, cam.type = 'truth_camera', 100, Marker.ARROW
        cam.pose.position.x, cam.pose.position.y, cam.pose.position.z = map(float, self.T_true[:3, 3])
        qx, qy, qz, qw = g.R_to_quat(self.T_true[:3, :3])
        cam.pose.orientation.x, cam.pose.orientation.y = qx, qy
        cam.pose.orientation.z, cam.pose.orientation.w = qz, qw
        cam.scale.x, cam.scale.y, cam.scale.z = 0.15, 0.02, 0.02
        cam.color.r = cam.color.g = cam.color.b = 0.6
        cam.color.a = 0.8
        arr.markers.append(cam)
        self.pub_markers.publish(arr)


def main():
    rclpy.init()
    node = VirtualCamera()
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
