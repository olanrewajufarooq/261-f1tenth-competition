"""Collision-avoidance calculations independent of ROS."""

import math

import numpy as np


class CollisionAvoidance:
    """Limit forward speed using available stopping clearance."""

    def __init__(
        self,
        braking_deceleration=1.0,
        reaction_time=0.25,
        clearance_margin=0.10,
    ):
        if (
            not math.isfinite(braking_deceleration)
            or braking_deceleration <= 0.0
        ):
            raise ValueError("Braking deceleration must be finite and positive.")

        for value in (reaction_time, clearance_margin):
            if not math.isfinite(value) or value < 0.0:
                raise ValueError("Reaction time and margin must be finite and nonnegative.")

        self.braking_deceleration = braking_deceleration
        self.reaction_time = reaction_time
        self.clearance_margin = clearance_margin

    def stopping_distance(self, measured_speed):
        """Travel reserve for measured speed, reaction latency and braking."""
        if not math.isfinite(measured_speed) or measured_speed < 0.0:
            raise ValueError("Measured speed must be finite and nonnegative.")
        return (self.clearance_margin + measured_speed * self.reaction_time
                + measured_speed ** 2 / (2.0 * self.braking_deceleration))

    def required_horizon(self, measured_speed, requested_speed):
        """Cover both current momentum and the requested operating speed."""
        return max(0.75, self.stopping_distance(measured_speed),
                    self.stopping_distance(requested_speed)) + 0.025

    def limit_speed(self, requested_speed, clearance):
        """Return a forward speed whose stopping distance fits the clearance.

        Clearance is the distance the vehicle can travel along its proposed
        motion before its footprint touches an obstacle, before adding the
        clearance margin. It is not a raw LiDAR range.
        """
        if not math.isfinite(requested_speed) or requested_speed < 0.0:
            raise ValueError("Requested speed must be finite and nonnegative.")

        if not math.isfinite(clearance) or clearance < 0.0:
            raise ValueError("Clearance must be finite and nonnegative.")

        available_distance = max(0.0, clearance - self.clearance_margin)
        deceleration = self.braking_deceleration
        reaction_term = deceleration * self.reaction_time

        # Solve: speed * reaction_time + speed² / (2 * deceleration)
        #        <= available_distance.
        safe_speed = math.sqrt(
            reaction_term ** 2
            + 2.0 * deceleration * available_distance
        ) - reaction_term

        return min(requested_speed, max(0.0, safe_speed))

    def scan_to_points(
        self,
        ranges,
        angle_min,
        angle_increment,
        range_min,
        range_max,
        lidar_offset=0.275,
    ):
        """Convert scan measurements into rear-axle-frame obstacle points.

        Positive infinity means no return within range_max.
        Invalid measurements raise ValueError so the ROS node can stop.
        """
        metadata = (
            angle_min, angle_increment, range_min, range_max, lidar_offset
        )
        if not all(math.isfinite(value) for value in metadata):
            raise ValueError("Scan metadata must be finite.")

        if angle_increment <= 0.0:
            raise ValueError("Scan angle increment must be positive.")

        if range_min < 0.0 or range_max <= range_min:
            raise ValueError("Scan range limits are invalid.")

        if len(ranges) == 0:
            raise ValueError("Scan must contain measurements.")

        points = []

        for index, distance in enumerate(ranges):
            if distance == math.inf:
                continue

            if (
                not math.isfinite(distance)
                or distance < range_min
                or distance > range_max
            ):
                raise ValueError("Scan contains an invalid measurement.")

            angle = angle_min + index * angle_increment
            points.append((
                lidar_offset + distance * math.cos(angle),
                distance * math.sin(angle),
            ))

        return tuple(points)

    def pose_on_arc(self, distance, steering, wheelbase=0.3302):
        """Return rear-axle (x, y, yaw) after travelling a constant-steering arc.

        The starting pose is (0, 0, 0). Distance is in metres;
        steering and returned yaw are in radians.
        """
        if not all(
            math.isfinite(value)
            for value in (distance, steering, wheelbase)
        ):
            raise ValueError("Arc inputs must be finite.")

        if distance < 0.0 or wheelbase <= 0.0:
            raise ValueError("Distance must be nonnegative and wheelbase positive.")

        if abs(steering) >= math.pi / 2.0:
            raise ValueError("Steering must be between -pi/2 and pi/2.")

        curvature = math.tan(steering) / wheelbase

        if abs(curvature) < 1e-9:
            return distance, 0.0, 0.0

        yaw = distance * curvature
        x = math.sin(yaw) / curvature
        y = 2.0 * math.sin(yaw / 2.0) ** 2 / curvature

        return x, y, yaw

    def footprint_contains(
        self,
        point,
        pose,
        length=0.58,
        width=0.31,
        rear_axle_to_center=0.17145,
    ):
        """Return whether an obstacle point lies inside the vehicle body.

        Point and rear-axle pose use the same coordinate frame.
        Pose contains (x, y, yaw). Body boundaries count as occupied.
        """
        point_x, point_y = point
        pose_x, pose_y, yaw = pose

        values = (
            point_x, point_y, pose_x, pose_y, yaw,
            length, width, rear_axle_to_center,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("Footprint inputs must be finite.")

        if length <= 0.0 or width <= 0.0:
            raise ValueError("Vehicle dimensions must be positive.")

        if abs(rear_axle_to_center) > length / 2.0:
            raise ValueError("Rear axle must lie within the vehicle body.")

        dx = point_x - pose_x
        dy = point_y - pose_y

        # Express the obstacle relative to the rear axle.
        forward = math.cos(yaw) * dx + math.sin(yaw) * dy
        lateral = -math.sin(yaw) * dx + math.cos(yaw) * dy

        # Shift the longitudinal reference to the body centre.
        forward -= rear_axle_to_center

        return (
            abs(forward) <= length / 2.0
            and abs(lateral) <= width / 2.0
        )

    def clearance_along_arc(
        self,
        obstacle_points,
        steering,
        horizon=2.0,
        sample_step=0.025,
        wheelbase=0.3302,
        length=0.58,
        width=0.31,
        rear_axle_to_center=0.17145,
    ):
        """Return conservative travel clearance along a turning arc.

        Obstacle points use the current rear-axle coordinate frame.
        An obstacle-free result is bounded by horizon; it does not
        establish that the sensor observed the entire searched area.
        """
        values = (
            steering, horizon, sample_step, wheelbase,
            length, width, rear_axle_to_center,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("Clearance-search inputs must be finite.")

        if horizon < 0.0 or not 0.0 < sample_step <= 0.05:
            raise ValueError("Horizon must be nonnegative; step must be in (0, 0.05].")

        if wheelbase <= 0.0 or length <= 0.0 or width <= 0.0:
            raise ValueError("Vehicle dimensions and wheelbase must be positive.")

        if abs(rear_axle_to_center) > length / 2.0:
            raise ValueError("Rear axle must lie within the vehicle body.")

        if abs(steering) >= math.pi / 2.0:
            raise ValueError("Steering must be between -pi/2 and pi/2.")

        points = tuple(obstacle_points)

        if not all(
            math.isfinite(x) and math.isfinite(y)
            for x, y in points
        ):
            raise ValueError("Obstacle coordinates must be finite.")

        points_array = np.asarray(points, dtype=float).reshape((-1, 2))
        curvature = math.tan(steering) / wheelbase

        # Bound how far any body point can move between sampled poses.
        body_radius = math.hypot(
            abs(rear_axle_to_center) + length / 2.0,
            width / 2.0,
        )
        sampling_buffer = sample_step * (
            1.0 + abs(curvature) * body_radius
        )

        # Longitudinal stopping reserve is applied by limit_speed().
        # Here we add lateral clearance and sampling protection.
        checked_length = length + 2.0 * sampling_buffer
        checked_width = width + 2.0 * (
            sampling_buffer + self.clearance_margin
        )

        steps = max(1, math.ceil(horizon / sample_step))
        previous_distance = 0.0

        for index in range(steps + 1):
            distance = min(index * sample_step, horizon)
            pose = self.pose_on_arc(distance, steering, wheelbase)

            # Transform all returns together; avoid a Python footprint call
            # for every beam at every sampled pose.
            offsets = points_array - np.asarray(pose[:2])
            cosine, sine = math.cos(pose[2]), math.sin(pose[2])
            forward = cosine * offsets[:, 0] + sine * offsets[:, 1]
            lateral = -sine * offsets[:, 0] + cosine * offsets[:, 1]
            if np.any((np.abs(forward - rear_axle_to_center) <= checked_length / 2)
                        & (np.abs(lateral) <= checked_width / 2)):
                return previous_distance

            previous_distance = distance

        return horizon

    def clearance_on_map(
        self,
        occupancy_map,
        pose,
        steering,
        horizon=2.0,
        sample_step=0.025,
    ):
        """Return conservative travel clearance against the static map.

        Pose is the current map-frame rear-axle (x, y, yaw).
        Vehicle dimensions match the pinned F1TENTH configuration.
        """
        x, y, yaw = pose

        values = (x, y, yaw, steering, horizon, sample_step)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("Map-clearance inputs must be finite.")

        if horizon < 0.0 or not 0.0 < sample_step <= 0.05:
            raise ValueError(
                "Horizon must be nonnegative; step must be in (0, 0.05]."
            )

        if abs(steering) >= math.pi / 2.0:
            raise ValueError("Steering must be between -pi/2 and pi/2.")

        wheelbase = 0.3302
        length = 0.58
        width = 0.31
        rear_axle_to_center = 0.17145
        margin = self.clearance_margin

        curvature = math.tan(steering) / wheelbase

        # Include the margin when bounding motion between samples.
        body_radius = math.hypot(
            rear_axle_to_center + length / 2.0 + margin,
            width / 2.0 + margin,
        )
        sampling_buffer = sample_step * (
            1.0 + abs(curvature) * body_radius
        )
        padding = margin + sampling_buffer

        cosine = math.cos(yaw)
        sine = math.sin(yaw)
        steps = max(1, math.ceil(horizon / sample_step))
        previous_distance = 0.0

        for index in range(steps + 1):
            distance = min(index * sample_step, horizon)
            local_x, local_y, local_yaw = self.pose_on_arc(
                distance, steering, wheelbase
            )

            rear_x = x + cosine * local_x - sine * local_y
            rear_y = y + sine * local_x + cosine * local_y
            predicted_yaw = yaw + local_yaw

            center_x = (
                rear_x + rear_axle_to_center * math.cos(predicted_yaw)
            )
            center_y = (
                rear_y + rear_axle_to_center * math.sin(predicted_yaw)
            )

            if not occupancy_map.is_rectangle_free(
                center_x,
                center_y,
                predicted_yaw,
                length + 2.0 * padding,
                width + 2.0 * padding,
            ):
                return previous_distance

            previous_distance = distance

        return horizon