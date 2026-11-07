"""
OpenCV ArUco 버전 호환 래퍼

Ubuntu 24.04 의 python3-opencv(4.6) 와 pip 의 최신 OpenCV(4.7+) 는
ArUco 함수 이름이 다릅니다. 이 파일이 그 차이를 감춰 줍니다.
"""
import cv2
import numpy as np

DICT_NAME = 'DICT_6X6_250'


def get_dictionary(name=DICT_NAME):
    if not hasattr(cv2, 'aruco'):
        raise RuntimeError('cv2.aruco 가 없습니다:  sudo apt install python3-opencv  (contrib 포함)')
    d = getattr(cv2.aruco, name)
    if hasattr(cv2.aruco, 'getPredefinedDictionary'):
        return cv2.aruco.getPredefinedDictionary(d)
    return cv2.aruco.Dictionary_get(d)


def marker_image(marker_id, pixels, name=DICT_NAME):
    d = get_dictionary(name)
    if hasattr(cv2.aruco, 'generateImageMarker'):
        return cv2.aruco.generateImageMarker(d, marker_id, pixels)
    return cv2.aruco.drawMarker(d, marker_id, pixels)


def detect(gray, name=DICT_NAME):
    """-> corners(list of (1,4,2)), ids(Nx1) 또는 (None, None)"""
    d = get_dictionary(name)
    if hasattr(cv2.aruco, 'ArucoDetector'):
        params = cv2.aruco.DetectorParameters()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX   # 꼭짓점 서브픽셀 보정
        corners, ids, _ = cv2.aruco.ArucoDetector(d, params).detectMarkers(gray)
    else:
        params = cv2.aruco.DetectorParameters_create()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        corners, ids, _ = cv2.aruco.detectMarkers(gray, d, parameters=params)
    return corners, ids


def marker_pose(corner, marker_size, K, dist):
    """마커 한 개의 4 꼭짓점 -> (R_cam_marker 3x3, t_cam_marker 3) [m]
    마커 좌표계: 중심 원점, x 오른쪽, y 위, z 마커 앞면(카메라 쪽)"""
    s = marker_size / 2.0
    obj = np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]], dtype=np.float64)
    img = np.asarray(corner, dtype=np.float64).reshape(4, 2)
    ok, rvec, tvec = cv2.solvePnP(obj, img, K, dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok:
        return None
    R, _ = cv2.Rodrigues(rvec)
    return R, tvec.reshape(3)
