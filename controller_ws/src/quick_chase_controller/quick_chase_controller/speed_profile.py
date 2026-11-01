"""Curvature-based nominal speed selection independent of ROS."""

import math


class SpeedProfile:
    """Preview path curvature and bound nominal lateral acceleration.

    This geometric speed request is subsequently limited by obstacle stopping
    clearance. It is not a dynamic stability or collision-free guarantee.
    """

    def __init__(self, lateral_acceleration=0.6, sample_step=0.05,
                    curvature_span=0.15):
        for value in (lateral_acceleration, sample_step, curvature_span):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError("Speed-profile settings must be finite and positive.")
        self.lateral_acceleration = lateral_acceleration
        self.sample_step = sample_step
        self.curvature_span = curvature_span

    def calculate(self, path, progress, maximum_speed, lookahead_distance):
        """Return nominal speed using maximum previewed absolute curvature.

        Three path samples estimate circumcircle curvature at each preview
        position. The preview includes the current position and wraps on the
        closed route; at most one complete lap is needed.
        """
        if not all(math.isfinite(value) for value in
                    (progress, maximum_speed, lookahead_distance)):
            raise ValueError("Speed-profile inputs must be finite.")
        if maximum_speed < 0.0 or lookahead_distance < 0.0:
            raise ValueError("Maximum speed and preview distance must be nonnegative.")
        preview = min(lookahead_distance, path.total_length)
        steps = max(1, math.ceil(preview / self.sample_step))
        max_curvature = 0.0
        span = min(self.curvature_span, path.total_length / 4.0)
        for index in range(steps + 1):
            distance = progress + min(index * self.sample_step, preview)
            before = path.point_at(distance - span)
            center = path.point_at(distance)
            after = path.point_at(distance + span)
            ax, ay = center[0] - before[0], center[1] - before[1]
            bx, by = after[0] - center[0], after[1] - center[1]
            denominator = math.hypot(ax, ay) * math.hypot(bx, by) * math.hypot(ax + bx, ay + by)
            if denominator < 1e-12:
                raise ValueError("Path curvature cannot be resolved at preview position.")
            curvature = 2.0 * abs(ax * by - ay * bx) / denominator
            max_curvature = max(max_curvature, curvature)
        if max_curvature <= 1e-12:
            return maximum_speed
        return min(maximum_speed,
                    math.sqrt(self.lateral_acceleration / max_curvature))