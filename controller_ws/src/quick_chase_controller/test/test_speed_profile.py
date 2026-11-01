"""Nominal speed responds to upcoming path geometry."""

import math
import unittest

from quick_chase_controller.reference_path import ReferencePath
from quick_chase_controller.speed_profile import SpeedProfile


class TestSpeedProfile(unittest.TestCase):
    def setUp(self):
        self.profile = SpeedProfile()
        self.square = ReferencePath([(0, 0), (10, 0), (10, 10), (0, 10)])

    def test_straight_preserves_requested_speed(self):
        self.assertEqual(self.profile.calculate(self.square, 2.0, 1.5, 2.0), 1.5)

    def test_preview_slows_before_corner(self):
        speed = self.profile.calculate(self.square, 8.0, 1.5, 2.5)
        self.assertGreater(speed, 0.0)
        self.assertLess(speed, 1.5)
        self.assertEqual(self.profile.calculate(self.square, 8.0, 1.5, 0.5), 1.5)

    def test_circle_matches_lateral_acceleration_bound(self):
        circle = ReferencePath([(2 * math.cos(i * math.tau / 1000),
                                    2 * math.sin(i * math.tau / 1000))
                                for i in range(1000)])
        speed = self.profile.calculate(circle, 1.0, 1.5, 2.0)
        self.assertAlmostEqual(speed, math.sqrt(0.6 * 2.0), places=3)

    def test_wraparound_includes_finish_line_corner(self):
        self.assertLess(self.profile.calculate(self.square, 39.5, 1.5, 1.0), 1.5)

    def test_zero_request_stays_zero(self):
        self.assertEqual(self.profile.calculate(self.square, 2.0, 0.0, 1.0), 0.0)

    def test_rejects_invalid_configuration(self):
        for key in ('lateral_acceleration', 'sample_step', 'curvature_span'):
            for value in (0.0, -1.0, math.nan, math.inf):
                with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                    SpeedProfile(**{key: value})

    def test_rejects_invalid_inputs(self):
        for values in ((math.nan, 1.0, 1.0), (0.0, math.inf, 1.0),
                        (0.0, -1.0, 1.0), (0.0, 1.0, -1.0)):
            with self.subTest(values=values), self.assertRaises(ValueError):
                self.profile.calculate(self.square, *values)