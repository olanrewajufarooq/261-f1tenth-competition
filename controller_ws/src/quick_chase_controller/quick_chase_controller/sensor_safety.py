"""Clock checks independent of ROS message and node types."""

import math


class SensorSafety:
    """Validate independent acquisition and delivery clocks."""

    @staticmethod
    def check(received_at, stamp_seconds, monotonic_now, ros_now,
                            maximum_age=0.25, future_tolerance=0.05):
        """Return a fail-closed reason for invalid receive or acquisition times."""
        values = (received_at, stamp_seconds, monotonic_now, ros_now)
        if not all(value is not None and math.isfinite(value) for value in values):
            return "invalid sensor time"
        receive_age = monotonic_now - received_at
        stamp_age = ros_now - stamp_seconds
        if stamp_seconds <= 0.0:
            return "missing sensor timestamp"
        if receive_age < 0.0 or stamp_age < -future_tolerance:
            return "sensor clock moved backwards or timestamp is in the future"
        if receive_age > maximum_age:
            return "stale sensor receipt"
        if stamp_age > maximum_age:
            return "stale sensor timestamp"
        return None

    @staticmethod
    def command_gap_reason(monotonic_now, last_publish_at, previous_speed,
                            maximum_gap=0.10):
        """Detect a late command while the preceding command requested motion.

        Detection only runs when execution resumes; an external watchdog is
        required to stop a vehicle during a completely stalled process.
        """
        if not math.isfinite(maximum_gap) or maximum_gap <= 0.0:
            raise ValueError("Command deadline must be finite and positive.")
        if not all(math.isfinite(value) for value in
                    (monotonic_now, last_publish_at, previous_speed)):
            return "invalid command clock or speed"
        gap = monotonic_now - last_publish_at
        if gap < 0.0:
            return "command clock moved backwards"
        if previous_speed > 0.0 and gap > maximum_gap:
            return "command_deadline_latched"
        return None