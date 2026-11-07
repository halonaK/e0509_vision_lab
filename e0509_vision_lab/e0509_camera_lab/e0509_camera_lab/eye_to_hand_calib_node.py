#!/usr/bin/env python3
"""
실습 4단계: Eye-to-hand 캘리브레이션

  준비: 그리퍼에 ArUco 마커(DICT_6X6_250, id 0, 한 변 10 cm)를 붙임
        (가상 카메라는 이미 붙어 있는 상태로 그려 줌)

  1) 로봇을 여러 자세로 움직이며 매 자세마다
       G = T_base_link6   (로봇이 아는 값: TF)
       C = T_cam_marker   (카메라가 본 값: ArUco + solvePnP)
     를 한 쌍씩 모음
  2) G·Y = X·C  (AX = XB) 를 풀어서  X = T_base_cam  계산 (여러 방법 비교)
  3) 결과 저장
       ~/doosan_ws/eye_to_hand.yaml                → calib_tf_publisher 가 자동으로 읽어 TF 갱신
       ~/doosan_ws/calibration_eye_to_hand.npz     → 원본 레포(object_tracking_node.py)와 같은 형식
                                                     키 'T_cam_to_base' (optical frame, 4x4)

  mode:=auto   정해진 관절 자세 목록으로 자동 수집 (가상/에뮬레이터용)
  mode:=manual 티치펜던트나 슬라이더로 로봇을 옮기고 Enter 로 한 장씩 수집 (실로봇용)

실행:  ros2 run e0509_camera_lab eye_to_hand_calib
"""
import datetime
import json
import math
import os
import time

import cv2
import numpy as np
import rclpy
import yaml
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from tf2_ros import Buffer, TransformListener

from e0509_gripper_demo.robot_client import RobotClient, READY
from e0509_camera_lab import geometry as g
from e0509_camera_lab import aruco_compat as ac
from e0509_camera_lab import handeye as he
from e0509_camera_lab import rosutil as ru

# READY 주변에서 마커가 카메라에 잘 보이고, 회전축이 골고루 섞인 14자세 [deg]
DEFAULT_POSES = [
    [0, 0, 90, 0, 90, 0], [20, 0, 90, 0, 90, 0], [0, -10, 100, 0, 80, 0], [0, 10, 80, 0, 100, 0],
    [10, 5, 85, 20, 90, 0], [0, 0, 90, 0, 70, 30], [0, 0, 90, 0, 110, -30], [15, -5, 95, -15, 80, 20],
    [-15, -5, 95, 15, 100, -20], [5, 10, 75, 10, 95, 40], [-5, 10, 75, -10, 85, -40],
    [25, 0, 90, 25, 90, 0], [-10, 0, 90, 0, 90, 25], [10, -8, 98, 0, 85, -25]]
WIN = 'eye_to_hand_calib'


class EyeToHandCalib(RobotClient):
    def __init__(self):
        super().__init__('eye_to_hand_calib')
        P = self.declare_parameter
        P('mode', 'auto')
        P('marker_id', 0)
        P('marker_size', 0.10)
        P('poses', [float(v) for q in DEFAULT_POSES for v in q])
        P('settle', 1.5)
        P('method', 'auto')            # auto | numpy-POINTS | numpy-PARK | opencv-TSAI | ...
        P('output_yaml', '~/doosan_ws/eye_to_hand.yaml')
        P('output_npz', '~/doosan_ws/calibration_eye_to_hand.npz')
        P('show', True)
        gp = lambda k: self.get_parameter(k).value
        self.mode, self.marker_id, self.marker_size = gp('mode'), gp('marker_id'), gp('marker_size')
        flat = gp('poses')
        self.poses = [flat[i:i + 6] for i in range(0, len(flat), 6)]
        self.settle, self.method, self.show = gp('settle'), gp('method'), gp('show')
        self.out_yaml = os.path.expanduser(gp('output_yaml'))
        self.out_npz = os.path.expanduser(gp('output_npz'))

        self.color = None
        self.color_time = 0.0
        self.info = None
        self.T_true_opt = None
        self.create_subscription(Image, ru.COLOR_TOPIC, self.on_color, 2)
        self.create_subscription(CameraInfo, ru.COLOR_INFO_TOPIC, lambda m: setattr(self, 'info', m), 2)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/virtual_scene/truth', self.on_truth, latched)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.G, self.C = [], []

    def on_color(self, msg):
        self.color, self.color_time = msg, time.time()

    def on_truth(self, msg):
        self.T_true_opt = np.array(json.loads(msg.data)['camera_link_in_base']) @ g.T_LINK_OPTICAL

    # ------------------------------------------------------------ 한 자세에서 한 쌍 수집
    def capture(self, label):
        t0 = time.time()
        while self.color_time < t0 or self.info is None:         # 멈춘 뒤 새로 들어온 영상
            rclpy.spin_once(self, timeout_sec=0.05)
            if time.time() - t0 > 5.0:
                self.get_logger().warn(f'{label}: 카메라 영상이 없습니다')
                return False
        img = ru.image_to_np(self.color)
        Km, dist, _ = ru.info_to_K(self.info)
        corners, ids = ac.detect(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY))
        idx = None if ids is None else next((i for i, v in enumerate(ids.ravel()) if v == self.marker_id), None)
        if idx is None:
            self.get_logger().warn(f'{label}: 마커(id {self.marker_id})가 안 보여서 건너뜀')
            self.preview(img)
            return False
        pose = ac.marker_pose(corners[idx], self.marker_size, Km, dist)
        if pose is None:
            return False
        try:
            tf = self.tf_buffer.lookup_transform(ru.BASE_FRAME, ru.FLANGE_FRAME, rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(f'{label}: TF base_link -> link_6 없음 ({e})')
            return False
        Gi, Ci = g.transform_to_T(tf.transform), g.make_T(*pose)
        self.G.append(Gi)
        self.C.append(Ci)
        cv2.aruco.drawDetectedMarkers(img, [corners[idx]])
        cv2.drawFrameAxes(img, Km, dist, cv2.Rodrigues(pose[0])[0], pose[1], self.marker_size * 0.6)
        self.preview(img)
        self.get_logger().info(
            f'{label}: 수집 #{len(self.G)}   flange xyz=({Gi[0, 3]:.3f}, {Gi[1, 3]:.3f}, {Gi[2, 3]:.3f}) m   '
            f'카메라→마커 거리 {np.linalg.norm(Ci[:3, 3]):.3f} m')
        return True

    def preview(self, img):
        if self.show:
            cv2.imshow(WIN, img)
            cv2.waitKey(1)

    # ------------------------------------------------------------ 수집
    def collect_auto(self):
        self.get_logger().info(f'자동 수집: {len(self.poses)}자세')
        for i, q in enumerate(self.poses):
            self.move_joint(q, vel=40, acc=40)
            self.sleep(self.settle)
            self.capture(f'[{i + 1}/{len(self.poses)}] {q}')
        self.move_joint(READY, vel=40, acc=40)

    def collect_manual(self):
        self.get_logger().info('수동 수집: 로봇을 옮긴 뒤 Enter = 수집, c = 계산 시작, q = 취소')
        while True:
            key = input(f'  [{len(self.G)}개 수집됨] Enter / c / q > ').strip().lower()
            if key == 'q':
                return False
            if key == 'c':
                return True
            self.capture(f'manual #{len(self.G) + 1}')

    # ------------------------------------------------------------ 계산 + 저장
    def solve_and_save(self):
        n = len(self.G)
        if n < 5:
            self.get_logger().error(f'수집 {n}개: 최소 5개(권장 10개 이상)가 필요합니다')
            return
        results = he.solve_all(self.G, self.C)
        lines = ['', f'캘리브레이션 결과 ({n}자세)',
                 f'  {"방법":18s} {"일관성(mm, deg)":>18s}' + ('   정답과 오차(mm, deg)' if self.T_true_opt is not None else '')]
        scores = {}
        for name, X in results.items():
            cpos, cang = he.consistency(X, self.G, self.C)
            scores[name] = cpos
            row = f'  {name:18s} {cpos:8.2f} {cang:8.2f}'
            if self.T_true_opt is not None:
                e = he.compare(X, self.T_true_opt)
                row += f'   {e[0]:8.2f} {e[1]:6.2f}'
            lines.append(row)
        best = min(scores, key=scores.get) if self.method == 'auto' else self.method
        if best not in results:
            self.get_logger().error(f'method={best} 를 쓸 수 없습니다. 가능한 값: {list(results)}')
            return
        X = results[best]                                   # base -> optical
        lines.append(f'  → 선택: {best}   (일관성: 자세마다 계산한 마커 장착 위치의 흩어짐, 작을수록 좋음)')
        self.get_logger().info('\n'.join(lines))

        T_link = X @ g.inv_T(g.T_LINK_OPTICAL)               # base -> camera_link (TF 용)
        qx, qy, qz, qw = g.R_to_quat(T_link[:3, :3])
        r, p, y = (math.degrees(a) for a in g.R_to_rpy(T_link[:3, :3]))
        data = {'parent_frame': ru.BASE_FRAME, 'child_frame': ru.CAMERA_LINK,
                'x': float(T_link[0, 3]), 'y': float(T_link[1, 3]), 'z': float(T_link[2, 3]),
                'qx': float(qx), 'qy': float(qy), 'qz': float(qz), 'qw': float(qw),
                'roll_deg': round(r, 3), 'pitch_deg': round(p, 3), 'yaw_deg': round(y, 3),
                'method': best, 'samples': n, 'consistency_mm': round(scores[best], 3),
                'date': datetime.datetime.now().isoformat(timespec='seconds')}
        os.makedirs(os.path.dirname(self.out_yaml), exist_ok=True)
        with open(self.out_yaml, 'w') as f:
            f.write('# eye-to-hand 캘리브레이션 결과: base_link -> camera_link\n')
            yaml.safe_dump(data, f, sort_keys=False, allow_unicode=True)
        Km = np.array(self.info.k).reshape(3, 3) if self.info else np.eye(3)
        np.savez(self.out_npz, T_cam_to_base=X, G=np.array(self.G), C=np.array(self.C), K=Km)
        self.get_logger().info(
            f'저장 완료\n  {self.out_yaml}  (calib_tf_publisher 가 자동으로 읽어 TF 갱신)\n'
            f'  {self.out_npz}  (원본 레포 object_tracking_node.py 형식: T_cam_to_base)\n'
            f'  camera_link xyz=({T_link[0, 3]:.3f}, {T_link[1, 3]:.3f}, {T_link[2, 3]:.3f}) m  '
            f'rpy=({r:.1f}, {p:.1f}, {y:.1f}) deg\n'
            f'→ click_to_base 로 다시 클릭해서 오차가 줄었는지 확인하세요')


def main():
    rclpy.init()
    node = EyeToHandCalib()
    try:
        ok = True
        if node.mode == 'manual':
            ok = node.collect_manual()
        else:
            node.collect_auto()
        if ok:
            node.solve_and_save()
    except KeyboardInterrupt:
        pass
    except RuntimeError as e:
        node.get_logger().error(str(e))
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
