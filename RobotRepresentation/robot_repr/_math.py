# -*- coding: utf-8 -*-
"""回転・同次変換・直線の計算（numpy だけに依存する。USD は使わない）。

座標の約束
  - 同次変換は 4x4 の numpy 配列で、列ベクトルに左から掛ける（p_world = T @ p_local）。
  - 角度はラジアン、長さはメートル。
"""

import math

import numpy as np

TOL = 1e-9  # 「0 とみなす」閾値。平行・交差・一致の判定と正準形の判定に使う


def vec3(v):
    a = np.asarray(v, dtype=float).reshape(3)
    if not np.all(np.isfinite(a)):
        raise ValueError(f"有限でない値が入っている: {v}")
    return a


def mat3(m):
    a = np.asarray(m, dtype=float).reshape(3, 3)
    if not np.all(np.isfinite(a)):
        raise ValueError(f"有限でない値が入っている: {m}")
    return a


def wrap_pi(angle):
    """角度を (-pi, pi] に入れる。"""
    a = math.fmod(angle, 2.0 * math.pi)
    if a <= -math.pi:
        a += 2.0 * math.pi
    elif a > math.pi:
        a -= 2.0 * math.pi
    return a


def clean(x, eps=1e-12):
    """数値誤差で 0 からずれた値を 0 に戻す（正準形の値を見やすく、比べやすくする）。"""
    return 0.0 if abs(x) < eps else float(x)


def clean_angle(a, eps=1e-12):
    """clean に加えて、pi の近く（-pi 側も）を pi にそろえる。"""
    a = clean(a, eps)
    return math.pi if abs(abs(a) - math.pi) < eps else a


def skew(w):
    x, y, z = w
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


def rot_axis_angle(axis, angle):
    """単位ベクトル axis 回りに angle 回す回転行列（Rodrigues）。"""
    k = skew(axis)
    return np.eye(3) + math.sin(angle) * k + (1.0 - math.cos(angle)) * (k @ k)


def rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def rotvec_to_matrix(rv):
    rv = vec3(rv)
    angle = float(np.linalg.norm(rv))
    if angle < 1e-15:
        return np.eye(3)
    return rot_axis_angle(rv / angle, angle)


def rotation_from_rpy(roll, pitch, yaw):
    """固定軸 X→Y→Z の順に回す回転行列（R = Rz(yaw) Ry(pitch) Rx(roll)）。"""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    return rot_z(yaw) @ ry @ rx


def is_rotation(r, tol=TOL * 100):
    r = mat3(r)
    return bool(np.allclose(r.T @ r, np.eye(3), atol=tol) and abs(np.linalg.det(r) - 1.0) < tol)


def tf(r=None, p=None):
    t = np.eye(4)
    if r is not None:
        t[:3, :3] = r
    if p is not None:
        t[:3, 3] = p
    return t


def tf_inv(t):
    r, p = t[:3, :3], t[:3, 3]
    return tf(r.T, -r.T @ p)


def trans(x=0.0, y=0.0, z=0.0):
    return tf(p=(x, y, z))


def exp_twist(w, v, theta):
    """ねじり（twist）S = (w, v) を theta だけ進めた同次変換 exp([S] theta)。

    w が単位ベクトルなら回転関節、w = 0 で v が単位ベクトルなら直動関節。
    """
    w = np.asarray(w, dtype=float)
    v = np.asarray(v, dtype=float)
    if np.linalg.norm(w) < 0.5:
        return tf(p=v * theta)
    k = skew(w)
    r = rot_axis_angle(w, theta)
    g = np.eye(3) * theta + (1.0 - math.cos(theta)) * k + (theta - math.sin(theta)) * (k @ k)
    return tf(r, g @ v)


def quat_wxyz(r):
    """回転行列 → 単位クォータニオン (w, x, y, z)。w >= 0 にそろえる。"""
    r = mat3(r)
    tr = r[0, 0] + r[1, 1] + r[2, 2]
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        q = (0.25 * s, (r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s)
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2.0
        q = ((r[2, 1] - r[1, 2]) / s, 0.25 * s, (r[0, 1] + r[1, 0]) / s, (r[0, 2] + r[2, 0]) / s)
    elif r[1, 1] > r[2, 2]:
        s = math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2.0
        q = ((r[0, 2] - r[2, 0]) / s, (r[0, 1] + r[1, 0]) / s, 0.25 * s, (r[1, 2] + r[2, 1]) / s)
    else:
        s = math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2.0
        q = ((r[1, 0] - r[0, 1]) / s, (r[0, 2] + r[2, 0]) / s, (r[1, 2] + r[2, 1]) / s, 0.25 * s)
    q = np.array(q)
    q /= np.linalg.norm(q)
    if q[0] < 0.0:
        q = -q
    return q


def min_rotation(a, b):
    """単位ベクトル a を b に重ねる最小の回転（回転軸が a と b の両方に垂直）。a = -b のときは None。"""
    c = np.cross(a, b)
    s = float(np.linalg.norm(c))
    d = float(np.dot(a, b))
    if s < 1e-12:
        return np.eye(3) if d > 0.0 else None
    return rot_axis_angle(c / s, math.atan2(s, d))


def project_point_to_line(x, point, direction):
    """点 x から直線（point を通り、単位ベクトル direction 向き）へ下ろした垂線の足。"""
    return point + np.dot(x - point, direction) * direction


def closest_points(p1, u1, p2, u2):
    """平行でない 2 直線の最近点の組（交わるなら同じ点）。"""
    w0 = p1 - p2
    b = float(np.dot(u1, u2))
    d = float(np.dot(u1, w0))
    e = float(np.dot(u2, w0))
    den = 1.0 - b * b
    s = (b * e - d) / den
    t = (e - b * d) / den
    return p1 + s * u1, p2 + t * u2


def angle_about(axis, x_from, x_to):
    """axis 回りに x_from を回して x_to に重ねる角度（-pi, pi]。"""
    return wrap_pi(math.atan2(float(np.dot(np.cross(x_from, x_to), axis)), float(np.dot(x_from, x_to))))
