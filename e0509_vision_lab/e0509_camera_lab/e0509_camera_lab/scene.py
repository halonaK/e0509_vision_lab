"""
가상 RGB-D 장면 렌더러 (numpy ray-casting)

실제 카메라가 없을 때 RealSense 대신 color + aligned depth 영상을 만듭니다.
  - 테이블(z = 0 평면), 색깔 박스들, 그리퍼에 붙인 ArUco 마커 보드를 그림
  - color 와 depth 를 같은 내부 파라미터로 그리므로 처음부터 정렬(aligned) 상태
  - 모든 위치는 로봇 베이스(base_link) 기준 [m]
"""
import cv2
import numpy as np

from e0509_camera_lab.geometry import T_LINK_OPTICAL

LIGHT = np.array([0.35, 0.25, 1.0]) / np.linalg.norm([0.35, 0.25, 1.0])


class Box:
    def __init__(self, name, center, size, bgr):
        self.name = name
        self.center = np.asarray(center, float)   # 박스 중심 [m]
        self.size = np.asarray(size, float)       # [sx, sy, sz] [m]
        self.bgr = np.asarray(bgr, float)
        self.visible = True

    @property
    def top_z(self):
        return float(self.center[2] + self.size[2] / 2)


class MarkerBoard:
    """흰 테두리가 있는 ArUco 마커 판. T_base_board 의 z 축 = 마커 앞면"""
    def __init__(self, texture, marker_size, board_size):
        self.tex = texture                # (H, W) uint8, 테두리 포함
        k = max(1, texture.shape[0] // 140) | 1   # 화면 한 픽셀이 덮는 텍스처 크기만큼 미리 흐리게
        self.tex_smooth = cv2.GaussianBlur(texture.astype(np.float32), (k, k), 0)
        self.marker_size = marker_size    # 검은 마커 한 변 [m]
        self.board_size = board_size      # 흰 테두리 포함 한 변 [m]
        self.T_base_board = None          # 4x4, None 이면 안 그림


def intrinsics(width=640, height=480, fx=615.0, fy=615.0, cx=None, cy=None):
    """RealSense D435 color 640x480 과 비슷한 값"""
    cx = (width - 1) / 2.0 if cx is None else cx
    cy = (height - 1) / 2.0 if cy is None else cy
    return dict(width=width, height=height, fx=fx, fy=fy, cx=cx, cy=cy)


_RAY_CACHE = {}


def _ray_dirs(K):
    key = tuple(sorted(K.items()))
    if key in _RAY_CACHE:
        return _RAY_CACHE[key]
    u, v = np.meshgrid(np.arange(K['width']), np.arange(K['height']))
    d = np.stack([(u - K['cx']) / K['fx'], (v - K['cy']) / K['fy'], np.ones_like(u, float)], axis=-1)
    _RAY_CACHE[key] = d.reshape(-1, 3)              # optical frame, z = 1
    return _RAY_CACHE[key]


def render(T_base_camlink, K, boxes, board=None, depth_noise=True, rng=None):
    """-> color (H,W,3) uint8 BGR,  depth (H,W) uint16 [mm, 0 = 측정 없음]"""
    rng = rng or np.random.default_rng()
    H, W = K['height'], K['width']
    T_base_opt = T_base_camlink @ T_LINK_OPTICAL
    R, o = T_base_opt[:3, :3], T_base_opt[:3, 3]
    d = _ray_dirs(K) @ R.T                          # base 기준 광선 방향 (optical z 성분 = 1)
    n = d.shape[0]

    t_best = np.full(n, np.inf)                     # 광선 파라미터 t = optical depth Z
    color = np.zeros((n, 3))
    color[:] = (40, 40, 40)                         # 배경

    # ---------------- 테이블 (z = 0, 1.6 m x 1.6 m)
    with np.errstate(divide='ignore', invalid='ignore'):
        t = -o[2] / d[:, 2]
    hit = (t > 0) & np.isfinite(t)
    p = o + d * t[:, None]
    hit &= (np.abs(p[:, 0] - 0.3) < 0.8) & (np.abs(p[:, 1]) < 0.8)
    grid = ((np.abs((p[:, 0] + 10) % 0.05 - 0.025) > 0.0235) |
            (np.abs((p[:, 1] + 10) % 0.05 - 0.025) > 0.0235))
    shade = np.where(grid, 150.0, 175.0)
    sel = hit & (t < t_best)
    t_best[sel] = t[sel]
    color[sel] = shade[sel, None]

    # ---------------- 박스 (축 정렬, slab 방법)
    dd = np.where(np.abs(d) < 1e-12, 1e-12, d)
    for b in boxes:
        if not b.visible:
            continue
        mn, mx = b.center - b.size / 2, b.center + b.size / 2
        t1, t2 = (mn - o) / dd, (mx - o) / dd
        tlo, thi = np.minimum(t1, t2), np.maximum(t1, t2)
        tmin, tmax = tlo.max(axis=1), thi.min(axis=1)
        sel = (tmax >= np.maximum(tmin, 0)) & (tmin > 0) & (tmin < t_best)
        if not sel.any():
            continue
        axis = tlo[sel].argmax(axis=1)
        normal = np.zeros((sel.sum(), 3))
        normal[np.arange(len(axis)), axis] = -np.sign(d[sel, axis])
        lam = 0.45 + 0.55 * np.clip(normal @ LIGHT, 0, 1)
        t_best[sel] = tmin[sel]
        color[sel] = b.bgr[None, :] * lam[:, None]

    # ---------------- ArUco 마커 보드
    if board is not None and board.T_base_board is not None:
        Tb = board.T_base_board
        xm, ym, nm, p0 = Tb[:3, 0], Tb[:3, 1], Tb[:3, 2], Tb[:3, 3]
        denom = d @ nm
        with np.errstate(divide='ignore', invalid='ignore'):
            t = ((p0 - o) @ nm) / denom
        p = o + d * t[:, None]
        a, bb = (p - p0) @ xm, (p - p0) @ ym
        half = board.board_size / 2
        sel = (t > 0) & (np.abs(a) < half) & (np.abs(bb) < half) & (t < t_best)
        front = sel & (denom < 0)
        back = sel & (denom >= 0)
        tex = board.tex_smooth
        th, tw = tex.shape
        # 양선형(bilinear) 보간: 경계가 계단처럼 깨지지 않아 실제 카메라처럼 꼭짓점 검출이 정확해짐
        fc = np.clip((a[front] / board.board_size + 0.5) * tw - 0.5, 0, tw - 1.001)
        fr = np.clip((0.5 - bb[front] / board.board_size) * th - 0.5, 0, th - 1.001)
        c0, r0 = fc.astype(int), fr.astype(int)
        dc, dr = fc - c0, fr - r0
        val = (tex[r0, c0] * (1 - dc) * (1 - dr) + tex[r0, c0 + 1] * dc * (1 - dr) +
               tex[r0 + 1, c0] * (1 - dc) * dr + tex[r0 + 1, c0 + 1] * dc * dr)
        color[front] = val[:, None]
        color[back] = (90, 90, 90)
        t_best[sel] = t[sel]

    # ---------------- 결과
    color = np.clip(color + rng.normal(0, 2.0, color.shape), 0, 255).astype(np.uint8).reshape(H, W, 3)
    z = np.where(np.isfinite(t_best), t_best, 0.0)
    if depth_noise:
        z = np.where(z > 0, z + rng.normal(0, 1, n) * (0.001 + 0.0015 * z * z), 0.0)
    depth = np.clip(np.round(z * 1000.0), 0, 65535).astype(np.uint16).reshape(H, W)
    return color, depth
