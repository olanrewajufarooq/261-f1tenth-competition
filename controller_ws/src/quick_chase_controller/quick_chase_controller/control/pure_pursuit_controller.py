
"""Pure Pursuit steering independent of ROS."""

import math


from quick_chase_controller.planning.reference_path import ReferencePath


class PurePursuitController:
    """Calculate steering to follow a reference path."""

    def __init__(
        self,
        reference_path: ReferencePath,
        lookahead=0.8,
        wheelbase=0.3302,
        steering_limit=0.4189,
        maximum_path_error=0.5,
    ):
        parameters = (
            lookahead, wheelbase, steering_limit, maximum_path_error
        )
        if not all(
            math.isfinite(value) and value > 0.0
            for value in parameters
        ):
            raise ValueError("Controller parameters must be finite and positive.")

        self.reference_path = reference_path
        self.lookahead = lookahead
        self.wheelbase = wheelbase
        self.steering_limit = steering_limit
        self.maximum_path_error = maximum_path_error

        self.reset()

    def reset(self):
        """Clear tracking history before starting a new run."""
        self._furthest_progress = None
        self._tracking_lost = False

    def calculate_steering(self, x, y, yaw):
        """Return steering in radians for a map-frame rear-axle pose."""
        if self._tracking_lost:
            raise ValueError("Tracking was lost. Reset the controller first.")

        if not all(math.isfinite(value) for value in (x, y, yaw)):
            raise ValueError("Vehicle pose must be finite.")

        progress, path_error = self.reference_path.project(x, y)

        if path_error > self.maximum_path_error:
            self._tracking_lost = True
            raise ValueError("Vehicle is too far from the reference path.")

        change = 0.0

        if self._furthest_progress is not None:
            length = self.reference_path.total_length

            # Signed distance from the furthest accepted progress,
            # accounting for the finish-line wraparound.
            change = (
                progress - self._furthest_progress + length / 2.0
            ) % length - length / 2.0

            if change < -0.1 or change > 1.0:
                self._tracking_lost = True
                raise ValueError("Unexpected change in path progress.")

        target_x, target_y = self.reference_path.point_at(
            progress + self.lookahead
        )

        dx = target_x - x
        dy = target_y - y

        forward = math.cos(yaw) * dx + math.sin(yaw) * dy
        lateral = -math.sin(yaw) * dx + math.cos(yaw) * dy

        if forward <= 1e-6:
            raise ValueError("Lookahead target is not ahead of the vehicle.")

        distance_squared = forward * forward + lateral * lateral
        steering = math.atan2(
            2.0 * self.wheelbase * lateral,
            distance_squared,
        )

        # Backward jitter must not move the progress reference backward.
        if self._furthest_progress is None or change > 0.0:
            self._furthest_progress = progress

        return max(
            -self.steering_limit,
            min(self.steering_limit, steering),
        )
