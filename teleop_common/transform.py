"""Config-driven coordinate transform: XRoboToolkit frame -> AgiBot A3 PICO frame.

Pure stdlib, Python >= 3.8, used on both the PC (fake SDK inverse replay) and the robot.

Position:      p_out = scale * Rz(yaw) * M * p_in + translation
Orientation:   R_out = Rz(yaw) * (M * R_in * M^T) * R_local
               (M may be any orthogonal 3x3 incl. reflections, so a handedness flip
               is handled correctly: M R M^T is always a proper rotation.)

All quaternions are (x, y, z, w), Hamilton convention, active rotation
(v_world = q * v_local * q^-1).
"""
from __future__ import annotations

import json
import math
from typing import Iterable, Optional, Sequence, Tuple

Vec3 = Tuple[float, float, float]
Quat = Tuple[float, float, float, float]
Mat3 = Tuple[Vec3, Vec3, Vec3]

IDENTITY_M: Mat3 = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
IDENTITY_Q: Quat = (0.0, 0.0, 0.0, 1.0)


# ---- quaternion / matrix helpers -----------------------------------------------------
def q_mul(a: Quat, b: Quat) -> Quat:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def q_conj(q: Quat) -> Quat:
    return (-q[0], -q[1], -q[2], q[3])


def q_norm(q: Quat) -> float:
    return math.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3])


def q_normalize(q: Quat) -> Quat:
    n = q_norm(q)
    if not n > 0.0 or not math.isfinite(n):
        raise ValueError("cannot normalise quaternion %r" % (q,))
    return (q[0] / n, q[1] / n, q[2] / n, q[3] / n)


def q_from_axis_angle(axis: Vec3, angle_rad: float) -> Quat:
    n = math.sqrt(axis[0] ** 2 + axis[1] ** 2 + axis[2] ** 2)
    s = math.sin(angle_rad / 2.0) / n
    return (axis[0] * s, axis[1] * s, axis[2] * s, math.cos(angle_rad / 2.0))


def q_rotate(q: Quat, v: Vec3) -> Vec3:
    return mat_vec(q_to_mat(q), v)


def q_to_mat(q: Quat) -> Mat3:
    x, y, z, w = q
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return ((1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy)),
            (2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx)),
            (2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy)))


def q_from_mat(m: Mat3) -> Quat:
    """Rotation matrix -> unit quaternion with w >= 0 (Shepperd's method)."""
    m00, m01, m02 = m[0]
    m10, m11, m12 = m[1]
    m20, m21, m22 = m[2]
    tr = m00 + m11 + m22
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        q = ((m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s, 0.25 * s)
    elif m00 > m11 and m00 > m22:
        s = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        q = (0.25 * s, (m01 + m10) / s, (m02 + m20) / s, (m21 - m12) / s)
    elif m11 > m22:
        s = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        q = ((m01 + m10) / s, 0.25 * s, (m12 + m21) / s, (m02 - m20) / s)
    else:
        s = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        q = ((m02 + m20) / s, (m12 + m21) / s, 0.25 * s, (m10 - m01) / s)
    if q[3] < 0.0:
        q = (-q[0], -q[1], -q[2], -q[3])
    return q_normalize(q)


def mat_vec(m: Mat3, v: Vec3) -> Vec3:
    return (m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
            m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
            m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2])


def mat_mul(a: Mat3, b: Mat3) -> Mat3:
    return tuple(tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3))
                 for i in range(3))  # type: ignore[return-value]


def mat_T(m: Mat3) -> Mat3:
    return tuple(tuple(m[j][i] for j in range(3)) for i in range(3))  # type: ignore[return-value]


def mat_det(m: Mat3) -> float:
    return (m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
            - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
            + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]))


def is_orthogonal(m: Mat3, tol: float = 1e-6) -> bool:
    p = mat_mul(m, mat_T(m))
    return all(abs(p[i][j] - (1.0 if i == j else 0.0)) <= tol for i in range(3) for j in range(3))


def yaw_of(q: Quat) -> float:
    """Heading (rad) of the rotated local +x axis about world +z."""
    fx, fy, _ = q_rotate(q, (1.0, 0.0, 0.0))
    return math.atan2(fy, fx)


# ---- the transform -------------------------------------------------------------------
class FrameTransform:
    def __init__(self, axis_matrix: Sequence[Sequence[float]] = IDENTITY_M, scale: float = 1.0,
                 yaw_offset_deg: float = 0.0, translation: Sequence[float] = (0.0, 0.0, 0.0),
                 local_rotation_xyzw: Sequence[float] = IDENTITY_Q):
        m = tuple(tuple(float(v) for v in row) for row in axis_matrix)
        if len(m) != 3 or any(len(r) != 3 for r in m):
            raise ValueError("axis_matrix must be 3x3")
        if not is_orthogonal(m):  # type: ignore[arg-type]
            raise ValueError("axis_matrix must be orthogonal (signed axis permutation)")
        if not (math.isfinite(scale) and scale > 0.0):
            raise ValueError("scale must be a positive finite number")
        if len(translation) != 3 or not all(math.isfinite(float(v)) for v in translation):
            raise ValueError("translation must be 3 finite numbers")
        self.axis_matrix: Mat3 = m  # type: ignore[assignment]
        self.scale = float(scale)
        self.yaw_offset_deg = float(yaw_offset_deg)
        self.translation: Vec3 = tuple(float(v) for v in translation)  # type: ignore[assignment]
        self.local_rotation: Quat = q_normalize(tuple(float(v) for v in local_rotation_xyzw))  # type: ignore[arg-type]
        self._yaw_q = q_from_axis_angle((0.0, 0.0, 1.0), math.radians(self.yaw_offset_deg))
        self._yaw_m = q_to_mat(self._yaw_q)
        self._pos_m = mat_mul(self._yaw_m, self.axis_matrix)
        self._m_t = mat_T(self.axis_matrix)

    @classmethod
    def from_config(cls, cfg: dict) -> "FrameTransform":
        t = cfg.get("transform", cfg)
        return cls(axis_matrix=t.get("axis_matrix", IDENTITY_M), scale=t.get("scale", 1.0),
                   yaw_offset_deg=t.get("yaw_offset_deg", 0.0),
                   translation=t.get("translation", (0.0, 0.0, 0.0)),
                   local_rotation_xyzw=t.get("local_rotation_xyzw", IDENTITY_Q))

    def position(self, p: Sequence[float]) -> Vec3:
        v = mat_vec(self._pos_m, (p[0], p[1], p[2]))
        s, t = self.scale, self.translation
        return (s * v[0] + t[0], s * v[1] + t[1], s * v[2] + t[2])

    def orientation(self, q: Sequence[float]) -> Quat:
        qn = q_normalize((q[0], q[1], q[2], q[3]))
        r = mat_mul(mat_mul(self.axis_matrix, q_to_mat(qn)), self._m_t)
        return q_normalize(q_mul(q_mul(self._yaw_q, q_from_mat(r)), self.local_rotation))

    def pose(self, pose7: Sequence[float]) -> Tuple[Vec3, Quat]:
        return self.position(pose7[0:3]), self.orientation(pose7[3:7])

    # inverse (used by the fake SDK to turn recorded A3 frames back into SDK-frame data)
    def inverse_position(self, p: Sequence[float]) -> Vec3:
        s, t = self.scale, self.translation
        v = ((p[0] - t[0]) / s, (p[1] - t[1]) / s, (p[2] - t[2]) / s)
        return mat_vec(mat_T(self._pos_m), v)

    def inverse_orientation(self, q: Sequence[float]) -> Quat:
        qn = q_normalize((q[0], q[1], q[2], q[3]))
        core = q_mul(q_mul(q_conj(self._yaw_q), qn), q_conj(self.local_rotation))
        r = mat_mul(mat_mul(self._m_t, q_to_mat(core)), self.axis_matrix)
        return q_from_mat(r)


def load_config(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as fh:
        return json.load(fh)


def all_finite(values: Iterable[float]) -> bool:
    return all(math.isfinite(v) for v in values)
