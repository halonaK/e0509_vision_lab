"""
좌표 변환 공통 함수 (ROS 없이 numpy 만 사용)

강의에서 손으로 따라 써 볼 핵심 식이 모두 여기 있습니다.
  - 역투영 (pixel + depth -> 카메라 3D 점)
  - 4x4 동차변환 만들기 / 적용하기
  - camera_link <-> camera_color_optical_frame 관계
"""
import math
import numpy as np


# ----------------------------------------------------------------- 회전 표현
def rpy_to_R(roll, pitch, yaw):
    """ROS 규약 roll-pitch-yaw (고정축 X->Y->Z) -> 3x3 회전행렬"""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr]])


def R_to_rpy(R):
    pitch = math.asin(max(-1.0, min(1.0, -R[2, 0])))
    roll = math.atan2(R[2, 1], R[2, 2])
    yaw = math.atan2(R[1, 0], R[0, 0])
    return roll, pitch, yaw


def quat_to_R(x, y, z, w):
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)]])


def R_to_quat(R):
    """3x3 -> (x, y, z, w)"""
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        w, x = 0.25 * s, (R[2, 1] - R[1, 2]) / s
        y, z = (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w, x = (R[2, 1] - R[1, 2]) / s, 0.25 * s
        y, z = (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w, x = (R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s
        y, z = 0.25 * s, (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w, x = (R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s
        y, z = (R[1, 2] + R[2, 1]) / s, 0.25 * s
    return x, y, z, w


def make_T(R, t):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def inv_T(T):
    R, t = T[:3, :3], T[:3, 3]
    return make_T(R.T, -R.T @ t)


def apply_T(T, pts):
    """pts: (3,) 또는 (N,3)  ->  같은 모양으로 변환된 점"""
    pts = np.asarray(pts, dtype=float)
    return pts @ T[:3, :3].T + T[:3, 3]


def rot_angle_deg(Ra, Rb):
    """두 회전 사이의 각도 차이 [deg]"""
    c = (np.trace(Ra.T @ Rb) - 1.0) / 2.0
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


def transform_to_T(tf):
    """geometry_msgs/Transform -> 4x4"""
    q, p = tf.rotation, tf.translation
    return make_T(quat_to_R(q.x, q.y, q.z, q.w), [p.x, p.y, p.z])


# ----------------------------------------------------------------- 카메라 프레임
# camera_link (ROS 규약: x 앞, y 왼쪽, z 위)
# camera_color_optical_frame (영상 규약: z 앞, x 오른쪽, y 아래)
R_LINK_OPTICAL = np.array([[0.0, 0.0, 1.0],
                           [-1.0, 0.0, 0.0],
                           [0.0, -1.0, 0.0]])
T_LINK_OPTICAL = make_T(R_LINK_OPTICAL, [0.0, 0.0, 0.0])


def look_at_rpy(cam_xyz, target_xyz):
    """카메라(camera_link)가 target 을 바라보도록 하는 roll, pitch, yaw"""
    d = np.asarray(target_xyz, float) - np.asarray(cam_xyz, float)
    yaw = math.atan2(d[1], d[0])
    pitch = math.atan2(-d[2], math.hypot(d[0], d[1]))   # 아래를 보면 +
    return 0.0, pitch, yaw


# ----------------------------------------------------------------- 역투영 / 투영
def deproject(u, v, z, fx, fy, cx, cy):
    """픽셀 (u,v) + depth z [m]  ->  카메라 optical frame 의 3D 점 [m]

        X = (u - cx) * Z / fx
        Y = (v - cy) * Z / fy
        Z = depth
    u, v, z 는 스칼라 또는 같은 길이의 배열"""
    u = np.asarray(u, float)
    v = np.asarray(v, float)
    z = np.asarray(z, float)
    return np.stack([(u - cx) * z / fx, (v - cy) * z / fy, z], axis=-1)


def project(p, fx, fy, cx, cy):
    """optical frame 3D 점 -> 픽셀 (u, v)"""
    p = np.asarray(p, float)
    return np.stack([fx * p[..., 0] / p[..., 2] + cx,
                     fy * p[..., 1] / p[..., 2] + cy], axis=-1)


# ----------------------------------------------------------------- 두산 posx
def zyz_to_R(a_deg, b_deg, c_deg):
    """두산 posx 의 자세 [rx, ry, rz] 는 ZYZ 오일러각: R = Rz(a) Ry(b) Rz(c)"""
    a, b, c = map(math.radians, (a_deg, b_deg, c_deg))

    def rz(t):
        return np.array([[math.cos(t), -math.sin(t), 0], [math.sin(t), math.cos(t), 0], [0, 0, 1]])

    def ry(t):
        return np.array([[math.cos(t), 0, math.sin(t)], [0, 1, 0], [-math.sin(t), 0, math.cos(t)]])
    return rz(a) @ ry(b) @ rz(c)
