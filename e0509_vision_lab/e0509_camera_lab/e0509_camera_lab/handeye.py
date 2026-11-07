"""
Eye-to-hand 캘리브레이션 계산

카메라는 바닥에 고정, 마커는 그리퍼에 고정.
  G_i = T_base_gripper  (로봇이 알려줌: TF 또는 posx)
  C_i = T_cam_marker    (카메라가 알려줌: ArUco + solvePnP)
  X   = T_base_cam      (구하려는 값, 상수)
  Y   = T_gripper_marker(마커 장착 위치, 상수, 몰라도 됨)

모든 자세에서  G_i · Y = X · C_i  가 성립.
두 자세 i, j 를 묶으면 Y 가 사라지고   A · X = X · B
  A = G_j · G_i⁻¹ ,  B = C_j · C_i⁻¹      ← 유명한 AX = XB 문제
"""
import numpy as np

from e0509_camera_lab.geometry import make_T, inv_T, rot_angle_deg

try:
    import cv2
    _HAS_CV_HANDEYE = hasattr(cv2, 'calibrateHandEye')
except ImportError:
    _HAS_CV_HANDEYE = False


def _log_R(R):
    """회전행렬 -> 회전벡터 (축 * 각도)"""
    c = np.clip((np.trace(R) - 1) / 2, -1.0, 1.0)
    th = np.arccos(c)
    if th < 1e-9:
        return np.zeros(3)
    w = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]) / (2 * np.sin(th))
    return w * th


def solve_park(G, C):
    """Park & Martin (1994) 방법을 numpy 로 직접 구현 (강의용).
    G, C: 4x4 리스트 (같은 길이, 3개 이상)"""
    n = len(G)
    A, B = [], []
    for i in range(n):
        for j in range(i + 1, n):
            A.append(G[j] @ inv_T(G[i]))
            B.append(C[j] @ inv_T(C[i]))
    # 1) 회전:  log(R_A) = R_X · log(R_B)
    M = np.zeros((3, 3))
    for a, b in zip(A, B):
        M += np.outer(_log_R(b[:3, :3]), _log_R(a[:3, :3]))
    w, V = np.linalg.eigh(M.T @ M)
    Rx = V @ np.diag(1.0 / np.sqrt(np.maximum(w, 1e-12))) @ V.T @ M.T
    U, _, Vt = np.linalg.svd(Rx)                    # 가장 가까운 회전행렬로 보정
    Rx = U @ np.diag([1, 1, np.linalg.det(U @ Vt)]) @ Vt
    # 2) 이동:  (R_A - I) t_X = R_X t_B - t_A   (최소제곱)
    lhs = np.vstack([a[:3, :3] - np.eye(3) for a in A])
    rhs = np.concatenate([Rx @ b[:3, 3] - a[:3, 3] for a, b in zip(A, B)])
    tx = np.linalg.lstsq(lhs, rhs, rcond=None)[0]
    return make_T(Rx, tx)


OPENCV_METHODS = ['TSAI', 'PARK', 'HORAUD', 'ANDREFF', 'DANIILIDIS']


def solve_opencv(G, C, method='TSAI'):
    """cv2.calibrateHandEye 를 eye-to-hand 로 사용하는 요령:
    'gripper2base' 자리에 base2gripper(= G⁻¹)를 넣으면 결과가 cam2base(= X)"""
    Gi = [inv_T(g) for g in G]
    R, t = cv2.calibrateHandEye(
        [g[:3, :3] for g in Gi], [g[:3, 3] for g in Gi],
        [c[:3, :3] for c in C], [c[:3, 3] for c in C],
        method=getattr(cv2, 'CALIB_HAND_EYE_' + method))
    return make_T(R, np.asarray(t).reshape(3))


def solve_all(G, C):
    """가능한 모든 방법으로 풀어서 {이름: X} 반환"""
    out = {'numpy-POINTS': solve_points(G, C), 'numpy-PARK': solve_park(G, C)}
    if _HAS_CV_HANDEYE:
        for m in OPENCV_METHODS:
            try:
                out['opencv-' + m] = solve_opencv(G, C, m)
            except cv2.error:
                pass
    return out


def consistency(X, G, C):
    """정답이 없어도 품질을 볼 수 있는 지표.
    각 자세에서 Y_i = G_i⁻¹ · X · C_i 를 계산하면 원래는 모두 같아야 함.
    -> 퍼진 정도(위치 mm, 각도 deg)가 작을수록 좋은 캘리브레이션"""
    Ys = [inv_T(g) @ X @ c for g, c in zip(G, C)]
    p = np.array([y[:3, 3] for y in Ys])
    pos_mm = float(np.linalg.norm(p - p.mean(axis=0), axis=1).mean() * 1000)
    ang = float(np.mean([rot_angle_deg(Ys[0][:3, :3], y[:3, :3]) for y in Ys[1:]])) if len(Ys) > 1 else 0.0
    return pos_mm, ang


def compare(X, X_true):
    """-> (위치 오차 mm, 각도 오차 deg)"""
    return float(np.linalg.norm(X[:3, 3] - X_true[:3, 3]) * 1000), rot_angle_deg(X[:3, :3], X_true[:3, :3])


def solve_points(G, C):
    """마커 '중심점'만 쓰는 방법 (강의용, 직관적이고 회전 노이즈에 강함).

    마커 중심 y 는 그리퍼에 고정된 한 점 → base 기준으로는  G_i · y
    카메라가 본 마커 중심 p_i (= C_i 의 이동 성분) → base 기준으로는  X · p_i
        X · p_i = G_i · y        (X: 3x4 미지수 12개, y: 미지수 3개)
    한 자세당 식 3개가 나오고 미지수에 대해 '선형'이므로 최소제곱으로 바로 풀림.
    (최소 5자세, 실제로는 10자세 이상 권장. 자세가 넓게 퍼져 있어야 정확함)"""
    rows, rhs = [], []
    for g, c in zip(G, C):
        p = c[:3, 3]
        Rg, tg = g[:3, :3], g[:3, 3]
        for k in range(3):
            row = np.zeros(15)
            row[k * 3:(k + 1) * 3] = p          # R_X[k, :] · p
            row[9 + k] = 1.0                    # + t_X[k]
            row[12:15] = -Rg[k, :]              # - (R_g · y)[k]
            rows.append(row)
            rhs.append(tg[k])
    sol = np.linalg.lstsq(np.array(rows), np.array(rhs), rcond=None)[0]
    R = sol[:9].reshape(3, 3)
    U, _, Vt = np.linalg.svd(R)                 # 가장 가까운 회전행렬로 보정
    R = U @ np.diag([1, 1, np.linalg.det(U @ Vt)]) @ Vt
    # 회전을 고정하고 t_X, y 를 다시 최소제곱
    rows2, rhs2 = [], []
    for g, c in zip(G, C):
        rows2.append(np.hstack([np.eye(3), -g[:3, :3]]))
        rhs2.append(g[:3, 3] - R @ c[:3, 3])
    s2 = np.linalg.lstsq(np.vstack(rows2), np.concatenate(rhs2), rcond=None)[0]
    return make_T(R, s2[:3])
