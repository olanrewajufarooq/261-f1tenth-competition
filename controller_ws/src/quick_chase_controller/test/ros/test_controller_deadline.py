"""Exercise the ROS callback's late-command latch without a live ROS graph."""

from types import SimpleNamespace
import unittest
from unittest.mock import patch

try:
    from quick_chase_controller.controller_node import ControllerNode
except ImportError:
    ControllerNode = None


@unittest.skipIf(ControllerNode is None, "ROS Python packages are unavailable")
class TestControllerDeadline(unittest.TestCase):
    def test_late_moving_command_stops_and_latches_until_restart(self):
        events = []
        node = SimpleNamespace(command_deadline_failure=None,
                                last_publish_at=0.0, last_speed=0.2)

        def stop():
            events.append('stop')
            node.last_speed = 0.0
            node.last_publish_at = 0.11

        node.publish_stop = stop
        node.report = events.append
        with patch('quick_chase_controller.controller_node.time.monotonic', return_value=0.11):
            ControllerNode.control_callback(node)
        self.assertEqual(events[0], 'stop')
        self.assertEqual(node.command_deadline_failure, 'command_deadline_latched')
        self.assertIn('restart required', events[1])
        # A zero publication refreshes receipt time but never clears the latch.
        with patch('quick_chase_controller.controller_node.time.monotonic', return_value=0.12):
            ControllerNode.control_callback(node)
        self.assertEqual(events[2], 'stop')
        self.assertEqual(node.command_deadline_failure, 'command_deadline_latched')