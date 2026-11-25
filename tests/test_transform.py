import math
import random
import unittest

from teleop_common import transform as T
from tests.util import CONFIG

S = math.sqrt(0.5)


class TestQuaternionMaths(unittest.TestCase):
    def assertVec(self, a, b, places=9):
        for x, y in zip(a, b):
            self.assertAlmostEqual(x, y, places=places)

    def assertQuat(self, a, b, places=9):
        if sum(x * y for x, y in zip(a, b)) < 0:
            b = tuple(-v for v in b)
        self.assertVec(a, b, places)

    def test_rotate_known_cases(self):
        qz90 = T.q_from_axis_angle((0, 0, 1), math.pi / 2)
        self.assertVec(qz90, (0, 0, S, S))
        self.assertVec(T.q_rotate(qz90, (1, 0, 0)), (0, 1, 0))        # +90 deg about z: x -> y
        self.assertVec(T.q_rotate(T.q_from_axis_angle((1, 0, 0), math.pi / 2), (0, 1, 0)), (0, 0, 1))
        self.assertVec(T.q_rotate(T.q_from_axis_angle((0, 1, 0), math.pi / 2), (0, 0, 1)), (1, 0, 0))
        self.assertVec(T.q_rotate(T.IDENTITY_Q, (1, 2, 3)), (1, 2, 3))

    def test_multiplication_order(self):
        qz = T.q_from_axis_angle((0, 0, 1), math.pi / 2)
        qx = T.q_from_axis_angle((1, 0, 0), math.pi / 2)
        # (qz*qx) applies qx first: y -> z (by qx) -> z (unchanged by qz)
        self.assertVec(T.q_rotate(T.q_mul(qz, qx), (0, 1, 0)), (0, 0, 1))
        self.assertVec(T.q_rotate(T.q_mul(qx, qz), (0, 1, 0)), (-1, 0, 0))
        self.assertQuat(T.q_mul(qz, T.q_conj(qz)), T.IDENTITY_Q)

    def test_matrix_roundtrip(self):
        rnd = random.Random(1)
        for _ in range(200):
            q = T.q_normalize(tuple(rnd.uniform(-1, 1) for _ in range(4)))
            m = T.q_to_mat(q)
            self.assertTrue(T.is_orthogonal(m, 1e-9))
            self.assertAlmostEqual(T.mat_det(m), 1.0, places=9)
            self.assertQuat(T.q_from_mat(m), q, places=7)
            self.assertGreaterEqual(T.q_from_mat(m)[3], 0.0)

    def test_from_mat_180deg_branches(self):
        for axis in ((1, 0, 0), (0, 1, 0), (0, 0, 1)):
            q = T.q_from_axis_angle(axis, math.pi)
            self.assertQuat(T.q_from_mat(T.q_to_mat(q)), q, places=7)

    def test_yaw_of(self):
        self.assertAlmostEqual(T.yaw_of(T.q_from_axis_angle((0, 0, 1), 1.0)), 1.0)
        self.assertAlmostEqual(T.yaw_of((0.0, 0.0, -0.752, -0.652)), math.radians(98.2), places=1)


class TestFrameTransform(unittest.TestCase):
    assertVec = TestQuaternionMaths.assertVec
    assertQuat = TestQuaternionMaths.assertQuat

    def setUp(self):
        self.cfg = T.load_config(CONFIG)
        self.tf = T.FrameTransform.from_config(self.cfg)

    def test_default_config_is_marked_provisional(self):
        self.assertEqual(self.cfg["status"], "PROVISIONAL")

    def test_default_axes(self):
        # assumed XRT frame: +x right, +y up, -z forward  ->  robot: +x forward, +y left, +z up
        self.assertVec(self.tf.position((0, 0, -1)), (1, 0, 0))    # forward
        self.assertVec(self.tf.position((1, 0, 0)), (0, -1, 0))    # right -> -y
        self.assertVec(self.tf.position((0, 1, 0)), (0, 0, 1))     # up
        self.assertAlmostEqual(T.mat_det(self.tf.axis_matrix), 1.0)

    def test_default_orientations(self):
        self.assertQuat(self.tf.orientation((0, 0, 0, 1)), (0, 0, 0, 1))
        # turning left (about source +y up) must become a turn about robot +z
        src = T.q_from_axis_angle((0, 1, 0), math.radians(90))
        self.assertQuat(self.tf.orientation(src), T.q_from_axis_angle((0, 0, 1), math.radians(90)))
        # pitching up about source +x (right) = rotation about robot -y
        src = T.q_from_axis_angle((1, 0, 0), math.radians(30))
        self.assertQuat(self.tf.orientation(src), T.q_from_axis_angle((0, -1, 0), math.radians(30)))

    def test_rotation_consistency(self):
        """World change W = Rz(yaw)*M, local axes relabelled by M:  W*(R u) == R' * (M u)."""
        rnd = random.Random(2)
        tf = T.FrameTransform(axis_matrix=self.tf.axis_matrix, yaw_offset_deg=37.0)
        for _ in range(100):
            q = T.q_normalize(tuple(rnd.uniform(-1, 1) for _ in range(4)))
            u = tuple(rnd.uniform(-1, 1) for _ in range(3))
            self.assertVec(tf.position(T.q_rotate(q, u)),
                           T.q_rotate(tf.orientation(q), self.tf.position(u)), places=9)

    def test_left_handed_source(self):
        """Unity-style source (+x right, +y up, +z forward): reflection matrix, det = -1."""
        tf = T.FrameTransform(axis_matrix=[[0, 0, 1], [-1, 0, 0], [0, 1, 0]])
        self.assertAlmostEqual(T.mat_det(tf.axis_matrix), -1.0)
        self.assertVec(tf.position((0, 0, 1)), (1, 0, 0))
        # In a left-handed frame a +90 deg quaternion about +y (up) turns forward(+z) to right(+x):
        src = T.q_from_axis_angle((0, 1, 0), math.radians(90))
        self.assertVec(T.q_rotate(src, (0, 0, 1)), (1, 0, 0))
        out = tf.orientation(src)
        self.assertAlmostEqual(T.q_norm(out), 1.0)
        # ... which in the robot frame is forward(+x) -> right(-y), i.e. -90 deg about +z
        self.assertVec(T.q_rotate(out, (1, 0, 0)), (0, -1, 0))
        self.assertQuat(out, T.q_from_axis_angle((0, 0, 1), math.radians(-90)))

    def test_yaw_scale_translation(self):
        tf = T.FrameTransform(yaw_offset_deg=90.0, scale=2.0, translation=(1.0, 2.0, -1.5))
        self.assertVec(tf.position((1, 0, 0)), (1.0, 4.0, -1.5))
        self.assertQuat(tf.orientation((0, 0, 0, 1)), (0, 0, S, S))

    def test_local_rotation(self):
        loc = T.q_from_axis_angle((0, 0, 1), math.radians(90))
        tf = T.FrameTransform(local_rotation_xyzw=loc)
        self.assertQuat(tf.orientation((0, 0, 0, 1)), loc)
        q = T.q_from_axis_angle((1, 0, 0), 0.3)
        self.assertQuat(tf.orientation(q), T.q_mul(q, loc))

    def test_inverse(self):
        rnd = random.Random(3)
        tf = T.FrameTransform(axis_matrix=self.tf.axis_matrix, yaw_offset_deg=-63.0, scale=1.13,
                              translation=(0.2, -0.4, -1.5),
                              local_rotation_xyzw=T.q_from_axis_angle((0, 1, 0), 0.4))
        for _ in range(100):
            p = tuple(rnd.uniform(-2, 2) for _ in range(3))
            q = T.q_normalize(tuple(rnd.uniform(-1, 1) for _ in range(4)))
            self.assertVec(tf.position(tf.inverse_position(p)), p, places=9)
            self.assertQuat(tf.orientation(tf.inverse_orientation(q)), q, places=9)

    def test_non_unit_input_is_normalised(self):
        self.assertQuat(self.tf.orientation((0, 0, 0, 2.0)), (0, 0, 0, 1))

    def test_invalid_config(self):
        with self.assertRaises(ValueError):
            T.FrameTransform(axis_matrix=[[1, 0, 0], [0, 1, 0], [0, 1, 0]])
        with self.assertRaises(ValueError):
            T.FrameTransform(scale=0.0)
        with self.assertRaises(ValueError):
            T.FrameTransform(translation=(0, float("nan"), 0))
        with self.assertRaises(ValueError):
            T.FrameTransform(local_rotation_xyzw=(0, 0, 0, 0))
        with self.assertRaises(ValueError):
            self.tf.orientation((0, 0, 0, 0))


if __name__ == "__main__":
    unittest.main()
