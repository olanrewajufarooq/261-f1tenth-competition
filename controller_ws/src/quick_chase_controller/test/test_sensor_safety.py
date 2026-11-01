"""Acquisition and receipt freshness must both pass."""

import unittest

from quick_chase_controller.sensor_safety import SensorSafety


class TestSensorSafety(unittest.TestCase):
    def test_fresh_times(self):
        self.assertIsNone(SensorSafety.check(9.9, 99.9, 10.0, 100.0))

    def test_old_acquisition_cannot_be_refreshed_by_delivery(self):
        self.assertEqual(SensorSafety.check(10.0, 99.0, 10.0, 100.0),
                            "stale sensor timestamp")

    def test_stale_receive_and_clock_guards(self):
        for values in ((9.0, 100.0, 10.0, 100.0),
                        (10.1, 100.0, 10.0, 100.0),
                        (10.0, 101.0, 10.0, 100.0),
                        (10.0, 0.0, 10.0, 100.0),
                        (None, 100.0, 10.0, 100.0)):
            with self.subTest(values=values):
                self.assertIsNotNone(SensorSafety.check(*values))


class TestCommandDeadline(unittest.TestCase):
    def test_two_period_threshold(self):
        self.assertIsNone(SensorSafety.command_gap_reason(0.1, 0.0, 0.2))
        self.assertIsNotNone(SensorSafety.command_gap_reason(0.100001, 0.0, 0.2))

    def test_zero_previous_command_does_not_latch_late_startup(self):
        self.assertIsNone(SensorSafety.command_gap_reason(5.0, 0.0, 0.0))

    def test_rejects_invalid_clock_and_configuration(self):
        for values in ((float('nan'), 0.0, 0.2),
                        (1.0, float('inf'), 0.2),
                        (0.0, 1.0, 0.2),
                        (1.0, 0.0, float('nan'))):
            with self.subTest(values=values):
                self.assertIsNotNone(SensorSafety.command_gap_reason(*values))
        for limit in (0.0, -1.0, float('nan'), float('inf')):
            with self.assertRaises(ValueError):
                SensorSafety.command_gap_reason(1.0, 0.0, 0.2, maximum_gap=limit)