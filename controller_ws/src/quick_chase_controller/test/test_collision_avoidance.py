"""Tests for clearance-based speed limiting."""

import math
import unittest

from PIL import Image

from quick_chase_controller.collision_avoidance import CollisionAvoidance
from quick_chase_controller.occupancy_map import OccupancyMap


class TestCollisionAvoidance(unittest.TestCase):
    def setUp(self):
        self.avoidance = CollisionAvoidance()

    def test_preserves_requested_speed_when_clear(self):
        self.assertAlmostEqual(
            self.avoidance.limit_speed(0.6, 5.0), 0.6
        )

    def test_reduces_speed_to_fit_stopping_distance(self):
        # At 0.5 m/s: reaction distance = 0.125 m,
        # braking distance = 0.125 m, margin = 0.10 m.
        self.assertAlmostEqual(
            self.avoidance.limit_speed(0.6, 0.35), 0.5
        )

    def test_stops_when_clearance_is_within_margin(self):
        for clearance in (0.0, 0.05, 0.10):
            with self.subTest(clearance=clearance):
                self.assertEqual(
                    self.avoidance.limit_speed(0.6, clearance), 0.0
                )

    def test_never_increases_a_zero_speed_request(self):
        self.assertEqual(self.avoidance.limit_speed(0.0, 5.0), 0.0)

    def test_rejects_invalid_clearance(self):
        for clearance in (-1.0, float("nan"), float("inf")):
            with self.subTest(clearance=clearance):
                with self.assertRaises(ValueError):
                    self.avoidance.limit_speed(0.6, clearance)

    def test_rejects_invalid_requested_speed(self):
        for speed in (-0.1, float("nan"), float("inf")):
            with self.subTest(speed=speed):
                with self.assertRaises(ValueError):
                    self.avoidance.limit_speed(speed, 5.0)

    def test_rejects_invalid_configuration(self):
        for settings in (
            {"braking_deceleration": 0.0},
            {"reaction_time": -0.1},
            {"clearance_margin": float("nan")},
        ):
            with self.subTest(settings=settings):
                with self.assertRaises(ValueError):
                    CollisionAvoidance(**settings)

    def test_scan_conversion_accounts_for_sensor_offset(self):
        points = self.avoidance.scan_to_points(
            ranges=[1.0, 2.0],
            angle_min=0.0,
            angle_increment=math.pi / 2.0,
            range_min=0.05,
            range_max=25.0,
        )

        self.assertAlmostEqual(points[0][0], 1.275)
        self.assertAlmostEqual(points[0][1], 0.0)
        self.assertAlmostEqual(points[1][0], 0.275)
        self.assertAlmostEqual(points[1][1], 2.0)

    def test_scan_conversion_skips_no_returns(self):
        points = self.avoidance.scan_to_points(
            ranges=[math.inf, 1.0],
            angle_min=-math.pi / 2.0,
            angle_increment=math.pi / 2.0,
            range_min=0.05,
            range_max=25.0,
        )

        self.assertEqual(len(points), 1)
        self.assertAlmostEqual(points[0][0], 1.275)
        self.assertAlmostEqual(points[0][1], 0.0)

    def test_scan_conversion_rejects_invalid_measurements(self):
        for distance in (
            float("nan"), -math.inf, -1.0, 0.01, 26.0
        ):
            with self.subTest(distance=distance):
                with self.assertRaises(ValueError):
                    self.avoidance.scan_to_points(
                        [distance], 0.0, 0.01, 0.05, 25.0
                    )

    def test_scan_conversion_rejects_empty_scan(self):
        with self.assertRaises(ValueError):
            self.avoidance.scan_to_points(
                [], 0.0, 0.01, 0.05, 25.0
            )

    def test_straight_motion_preserves_heading(self):
        pose = self.avoidance.pose_on_arc(0.5, 0.0)
        self.assertEqual(pose, (0.5, 0.0, 0.0))

    def test_left_turn_follows_unit_radius_circle(self):
        # tan(steering) / wheelbase = 1, so radius = 1 metre.
        steering = math.atan(0.3302)
        x, y, yaw = self.avoidance.pose_on_arc(
            math.pi / 2.0, steering
        )

        self.assertAlmostEqual(x, 1.0)
        self.assertAlmostEqual(y, 1.0)
        self.assertAlmostEqual(yaw, math.pi / 2.0)

    def test_right_turn_mirrors_left_turn(self):
        steering = -math.atan(0.3302)
        x, y, yaw = self.avoidance.pose_on_arc(
            math.pi / 2.0, steering
        )

        self.assertAlmostEqual(x, 1.0)
        self.assertAlmostEqual(y, -1.0)
        self.assertAlmostEqual(yaw, -math.pi / 2.0)

    def test_zero_distance_keeps_initial_pose(self):
        self.assertEqual(
            self.avoidance.pose_on_arc(0.0, 0.3),
            (0.0, 0.0, 0.0),
        )

    def test_arc_rejects_invalid_inputs(self):
        for distance, steering, wheelbase in (
            (-1.0, 0.0, 0.3302),
            (1.0, float("nan"), 0.3302),
            (1.0, 0.0, 0.0),
            (1.0, math.pi / 2.0, 0.3302),
        ):
            with self.subTest(
                distance=distance,
                steering=steering,
                wheelbase=wheelbase,
            ):
                with self.assertRaises(ValueError):
                    self.avoidance.pose_on_arc(
                        distance, steering, wheelbase
                    )

    def test_footprint_accounts_for_rear_axle_offset(self):
        pose = (0.0, 0.0, 0.0)

        self.assertTrue(
            self.avoidance.footprint_contains((0.45, 0.0), pose)
        )
        self.assertFalse(
            self.avoidance.footprint_contains((0.48, 0.0), pose)
        )
        self.assertTrue(
            self.avoidance.footprint_contains((-0.10, 0.0), pose)
        )
        self.assertFalse(
            self.avoidance.footprint_contains((-0.15, 0.0), pose)
        )

    def test_footprint_checks_vehicle_width(self):
        pose = (0.0, 0.0, 0.0)

        for lateral in (-0.15, 0.15):
            self.assertTrue(
                self.avoidance.footprint_contains((0.2, lateral), pose)
            )

        for lateral in (-0.17, 0.17):
            self.assertFalse(
                self.avoidance.footprint_contains((0.2, lateral), pose)
            )

    def test_footprint_handles_translation_and_rotation(self):
        pose = (2.0, 3.0, math.pi / 2.0)

        self.assertTrue(
            self.avoidance.footprint_contains((2.0, 3.45), pose)
        )
        self.assertFalse(
            self.avoidance.footprint_contains((2.2, 3.0), pose)
        )

    def test_footprint_rejects_invalid_geometry(self):
        with self.assertRaises(ValueError):
            self.avoidance.footprint_contains(
                (0.0, 0.0), (0.0, 0.0, 0.0), width=-0.31
            )

        with self.assertRaises(ValueError):
            self.avoidance.footprint_contains(
                (float("nan"), 0.0), (0.0, 0.0, 0.0)
            )

    def test_clearance_returns_search_horizon_when_clear(self):
        clearance = self.avoidance.clearance_along_arc([], 0.0)
        self.assertEqual(clearance, 2.0)

    def test_clearance_detects_obstacle_ahead(self):
        clearance = self.avoidance.clearance_along_arc(
            [(1.0, 0.0)], 0.0
        )

        # Physical front bumper reaches the point after about 0.539 m.
        # Sampling protection must make the result more conservative.
        self.assertGreaterEqual(clearance, 0.4)
        self.assertLessEqual(clearance, 0.539)

    def test_clearance_stops_for_obstacle_inside_current_body(self):
        clearance = self.avoidance.clearance_along_arc(
            [(0.2, 0.0)], 0.0
        )
        self.assertEqual(clearance, 0.0)

    def test_clearance_ignores_obstacles_outside_straight_corridor(self):
        clearance = self.avoidance.clearance_along_arc(
            [(1.0, 1.0)], 0.0, horizon=1.6
        )
        self.assertEqual(clearance, 1.6)

    def test_clearance_checks_left_and_right_turns(self):
        steering = math.atan(0.3302)

        for direction in (-1.0, 1.0):
            with self.subTest(direction=direction):
                clearance = self.avoidance.clearance_along_arc(
                    [(1.0, direction)],
                    direction * steering,
                    horizon=1.6,
                )
                self.assertGreater(clearance, 0.0)
                self.assertLess(clearance, 1.6)

    def test_clearance_rejects_invalid_search_inputs(self):
        for settings in (
            {"horizon": -1.0},
            {"sample_step": 0.0},
            {"sample_step": 0.1},
            {"width": -0.31},
        ):
            with self.subTest(settings=settings):
                with self.assertRaises(ValueError):
                    self.avoidance.clearance_along_arc(
                        [], 0.0, **settings
                    )

    def test_map_clearance_returns_horizon_in_open_space(self):
        image = Image.new("L", (80, 80), 255)
        occupancy_map = OccupancyMap(
            image, 0.1, (-4.0, -4.0, 0.0)
        )

        clearance = self.avoidance.clearance_on_map(
            occupancy_map, (0.0, 0.0, 0.0), 0.0
        )
        self.assertEqual(clearance, 2.0)

    def test_map_clearance_detects_wall_ahead(self):
        image = Image.new("L", (80, 80), 255)

        # A vertical wall occupying world x in [1.0, 1.1).
        for row in range(80):
            image.putpixel((50, row), 0)

        occupancy_map = OccupancyMap(
            image, 0.1, (-4.0, -4.0, 0.0)
        )

        clearance = self.avoidance.clearance_on_map(
            occupancy_map, (0.0, 0.0, 0.0), 0.0
        )
        self.assertGreater(clearance, 0.0)
        self.assertLess(clearance, 0.54)

    def test_map_clearance_respects_vehicle_world_heading(self):
        image = Image.new("L", (80, 80), 255)

        # A horizontal wall occupying world y in [1.0, 1.1).
        for column in range(80):
            image.putpixel((column, 29), 0)

        occupancy_map = OccupancyMap(
            image, 0.1, (-4.0, -4.0, 0.0)
        )

        clearance = self.avoidance.clearance_on_map(
            occupancy_map, (0.0, 0.0, math.pi / 2.0), 0.0
        )
        self.assertGreater(clearance, 0.0)
        self.assertLess(clearance, 0.54)

    def test_map_clearance_stops_if_current_footprint_is_blocked(self):
        image = Image.new("L", (80, 80), 0)
        occupancy_map = OccupancyMap(
            image, 0.1, (-4.0, -4.0, 0.0)
        )

        clearance = self.avoidance.clearance_on_map(
            occupancy_map, (0.0, 0.0, 0.0), 0.0
        )
        self.assertEqual(clearance, 0.0)

    def test_horizon_covers_actual_momentum_above_command(self):
        horizon = self.avoidance.required_horizon(2.0, 0.2)
        self.assertGreater(horizon, self.avoidance.stopping_distance(2.0))
        self.assertGreater(horizon, self.avoidance.required_horizon(0.2, 0.2))

    def test_stopping_distance_uses_measured_speed(self):
        self.assertAlmostEqual(self.avoidance.stopping_distance(0.5), 0.35)
        for speed in (-1.0, math.nan, math.inf):
            with self.assertRaises(ValueError):
                self.avoidance.stopping_distance(speed)

    def test_default_lidar_sampling_matches_explicit_map_step(self):
        points = [(0.95, 0.19), (0.7, -0.21), (1.2, 0.0)]
        for steering in (-0.35, 0.0, 0.35):
            self.assertEqual(
                self.avoidance.clearance_along_arc(points, steering),
                self.avoidance.clearance_along_arc(points, steering, sample_step=0.025))

    def test_captured_pose_on_synthetic_open_map_and_scan(self):
        # No captured scan exists: this checks coordinate/sampling handling,
        # not replay of the reported collision.
        pose = (-12.67376, 8.22197, -2.84466)
        occupancy_map = OccupancyMap(
            Image.new("L", (200, 200), 255), 0.05,
            (pose[0] - 5.0, pose[1] - 5.0, 0.0))
        for steering in (-0.35, 0.0, 0.35):
            self.assertEqual(self.avoidance.clearance_on_map(
                occupancy_map, pose, steering, horizon=0.775), 0.775)
            # Synthetic return inside the current body must stop at any yaw.
            self.assertEqual(self.avoidance.clearance_along_arc(
                [(0.2, 0.0)], steering, horizon=0.775), 0.0)


if __name__ == "__main__":
    unittest.main()