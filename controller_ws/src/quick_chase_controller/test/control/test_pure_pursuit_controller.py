"""Tests for Pure Pursuit steering."""

import math
import unittest

from quick_chase_controller.control.pure_pursuit_controller import (
    PurePursuitController,
)
from quick_chase_controller.planning.reference_path import ReferencePath


class TestPurePursuitController(unittest.TestCase):
    def setUp(self):
        self.path = ReferencePath([
            (0, 0), (2, 0), (2, 2), (0, 2),
        ])
        self.controller = PurePursuitController(self.path)

    def test_straight_path_requires_zero_steering(self):
        steering = self.controller.calculate_steering(0.5, 0.0, 0.0)
        self.assertAlmostEqual(steering, 0.0)

    def test_left_corner_produces_expected_steering(self):
        # Target is (2, 0.1), relative to vehicle at (1.3, 0).
        # Curvature = 2 * 0.1 / (0.7² + 0.1²) = 0.4.
        steering = self.controller.calculate_steering(1.3, 0.0, 0.0)
        self.assertAlmostEqual(steering, math.atan(0.3302 * 0.4))

    def test_right_corner_produces_negative_steering(self):
        path = ReferencePath([
            (0, 0), (2, 0), (2, -2), (0, -2),
        ])
        controller = PurePursuitController(path)

        steering = controller.calculate_steering(1.3, 0.0, 0.0)
        self.assertAlmostEqual(steering, -math.atan(0.3302 * 0.4))

    def test_steering_is_limited(self):
        steering = self.controller.calculate_steering(1.8, 0.0, 0.0)
        self.assertAlmostEqual(steering, 0.4189)

    def test_rotated_vehicle_uses_vehicle_frame(self):
        steering = self.controller.calculate_steering(
            2.0, 0.5, math.pi / 2.0
        )
        self.assertAlmostEqual(steering, 0.0)

    def test_tracks_across_path_wraparound(self):
        steering = self.controller.calculate_steering(
            0.0, 0.3, -math.pi / 2.0
        )
        self.assertGreater(steering, 0.0)
        self.assertLessEqual(steering, 0.4189)

    def test_rejects_excessive_path_error(self):
        with self.assertRaises(ValueError):
            self.controller.calculate_steering(1.0, -1.0, 0.0)

    def test_rejects_target_behind_vehicle(self):
        with self.assertRaises(ValueError):
            self.controller.calculate_steering(0.5, 0.0, math.pi)

    def test_rejects_nonfinite_pose(self):
        with self.assertRaises(ValueError):
            self.controller.calculate_steering(0.0, 0.0, float("nan"))

    def test_rejects_invalid_parameters(self):
        for lookahead in (0.0, -1.0, float("inf")):
            with self.subTest(lookahead=lookahead):
                with self.assertRaises(ValueError):
                    PurePursuitController(
                        self.path, lookahead=lookahead
                    )

    def test_accepts_normal_forward_progress(self):
        self.controller.calculate_steering(0.2, 0.0, 0.0)
        steering = self.controller.calculate_steering(0.23, 0.0, 0.0)
        self.assertAlmostEqual(steering, 0.0)

    def test_accepts_progress_across_finish_line(self):
        self.controller.calculate_steering(
            0.0, 0.3, -math.pi / 2.0
        )
        steering = self.controller.calculate_steering(0.1, 0.0, 0.0)
        self.assertAlmostEqual(steering, 0.0)

    def test_progress_jump_requires_explicit_reset(self):
        self.controller.calculate_steering(0.2, 0.0, 0.0)

        with self.assertRaises(ValueError):
            self.controller.calculate_steering(1.4, 0.0, 0.0)

        # Returning to the original location must not restart tracking.
        with self.assertRaises(ValueError):
            self.controller.calculate_steering(0.23, 0.0, 0.0)

        self.controller.reset()
        steering = self.controller.calculate_steering(0.23, 0.0, 0.0)
        self.assertAlmostEqual(steering, 0.0)

    def test_excessive_path_error_requires_reset(self):
        self.controller.calculate_steering(0.5, 0.0, 0.0)

        with self.assertRaises(ValueError):
            self.controller.calculate_steering(0.5, -1.0, 0.0)

        # Returning to the path must not automatically resume tracking.
        with self.assertRaises(ValueError):
            self.controller.calculate_steering(0.5, 0.0, 0.0)

        self.controller.reset()
        steering = self.controller.calculate_steering(0.5, 0.0, 0.0)
        self.assertAlmostEqual(steering, 0.0)

    def test_backward_steps_cannot_accumulate(self):
        self.controller.calculate_steering(1.0, 0.0, 0.0)
        self.controller.calculate_steering(0.95, 0.0, 0.0)
        self.controller.calculate_steering(0.91, 0.0, 0.0)

        # Each step is small, but total backward progress exceeds 0.1 m.
        with self.assertRaises(ValueError):
            self.controller.calculate_steering(0.89, 0.0, 0.0)

        with self.assertRaises(ValueError):
            self.controller.calculate_steering(1.0, 0.0, 0.0)

    def test_small_backward_jitter_allows_forward_recovery(self):
        self.controller.calculate_steering(1.0, 0.0, 0.0)
        self.controller.calculate_steering(0.95, 0.0, 0.0)

        steering = self.controller.calculate_steering(1.03, 0.0, 0.0)
        self.assertAlmostEqual(steering, 0.0)

if __name__ == "__main__":
    unittest.main()