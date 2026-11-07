"""ROS 메시지 <-> numpy 변환 (cv_bridge 없이)"""
import numpy as np
from sensor_msgs.msg import Image, CameraInfo

# realsense2_camera(ROS 2) 기본 토픽 이름과 동일하게 맞춤
COLOR_TOPIC = '/camera/camera/color/image_raw'
COLOR_INFO_TOPIC = '/camera/camera/color/camera_info'
DEPTH_TOPIC = '/camera/camera/aligned_depth_to_color/image_raw'
DEPTH_INFO_TOPIC = '/camera/camera/aligned_depth_to_color/camera_info'
OPTICAL_FRAME = 'camera_color_optical_frame'
CAMERA_LINK = 'camera_link'
BASE_FRAME = 'base_link'
FLANGE_FRAME = 'link_6'


def image_to_np(msg):
    if msg.encoding in ('bgr8', 'rgb8'):
        a = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, :msg.width * 3]
        a = a.reshape(msg.height, msg.width, 3)
        return a[:, :, ::-1].copy() if msg.encoding == 'rgb8' else a.copy()
    if msg.encoding in ('16UC1', 'mono16'):
        a = np.frombuffer(msg.data, np.uint16).reshape(msg.height, msg.step // 2)[:, :msg.width]
        return a.copy()
    if msg.encoding == '32FC1':                     # m -> mm 로 통일
        a = np.frombuffer(msg.data, np.float32).reshape(msg.height, msg.step // 4)[:, :msg.width]
        return np.nan_to_num(a * 1000.0).astype(np.uint16)
    if msg.encoding == 'mono8':
        return np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, :msg.width].copy()
    raise ValueError(f'지원하지 않는 encoding: {msg.encoding}')


def np_to_image(arr, encoding, frame_id, stamp):
    msg = Image()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    msg.height, msg.width = arr.shape[:2]
    msg.encoding = encoding
    msg.is_bigendian = 0
    msg.step = arr.strides[0]
    msg.data = np.ascontiguousarray(arr).tobytes()
    return msg


def make_camera_info(K, frame_id, stamp):
    m = CameraInfo()
    m.header.frame_id = frame_id
    m.header.stamp = stamp
    m.width, m.height = K['width'], K['height']
    m.distortion_model = 'plumb_bob'
    m.d = [0.0] * 5
    m.k = [K['fx'], 0.0, K['cx'], 0.0, K['fy'], K['cy'], 0.0, 0.0, 1.0]
    m.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    m.p = [K['fx'], 0.0, K['cx'], 0.0, 0.0, K['fy'], K['cy'], 0.0, 0.0, 0.0, 1.0, 0.0]
    return m


def info_to_K(info):
    """CameraInfo -> (3x3 K, dist, dict)"""
    Km = np.array(info.k, dtype=np.float64).reshape(3, 3)
    dist = np.array(info.d if len(info.d) else [0.0] * 5, dtype=np.float64)
    return Km, dist, dict(fx=Km[0, 0], fy=Km[1, 1], cx=Km[0, 2], cy=Km[1, 2],
                          width=info.width, height=info.height)
