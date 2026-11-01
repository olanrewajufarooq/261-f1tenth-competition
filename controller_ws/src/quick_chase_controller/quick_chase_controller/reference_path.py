"""Reference-path geometry independent of ROS."""

import csv
import math
from bisect import bisect_right


class ReferencePath:
    """Store and query an ordered, closed reference path."""

    def __init__(self, points, widths=None):
        self.points = tuple((float(x), float(y)) for x, y in points)

        if len(self.points) < 3:
            raise ValueError("A reference path requires at least three points.")

        if not all(
            math.isfinite(value)
            for point in self.points
            for value in point
        ):
            raise ValueError("Path coordinates must be finite.")

        self.widths = None if widths is None else tuple(
            (float(right), float(left)) for right, left in widths
        )

        if self.widths is not None and (
            len(self.widths) != len(self.points) or not all(
                math.isfinite(value) and value > 0
                for pair in self.widths for value in pair
            )
        ):
            raise ValueError("Widths must be positive finite pairs for every point.")

        lengths = []
        distances = [0.0]

        for index, start in enumerate(self.points):
            end = self.points[(index + 1) % len(self.points)]
            length = math.hypot(end[0] - start[0], end[1] - start[1])

            if length < 1e-6:
                raise ValueError("Path contains a zero-length segment.")

            lengths.append(length)
            distances.append(distances[-1] + length)

        self.segment_lengths = tuple(lengths)
        self.cumulative_distances = tuple(distances)
        self.total_length = distances[-1]

    @classmethod
    def from_csv(cls, filename):
        """Load the supplied x, y, right-width, left-width CSV format."""
        points = []
        widths = []

        with open(filename, encoding="utf-8") as stream:
            lines = (
                line for line in stream
                if line.strip() and not line.lstrip().startswith("#")
            )

            for row in csv.reader(lines):
                if len(row) != 4:
                    raise ValueError("Each route row must contain four values.")

                x, y, right_width, left_width = map(float, row)

                if not all(
                    math.isfinite(width) and width > 0.0
                    for width in (right_width, left_width)
                ):
                    raise ValueError("Track widths must be finite and positive.")

                points.append((x, y))
                widths.append((right_width, left_width))

        return cls(points, widths=widths)

    def point_at(self, distance):
        """Return interpolated (x, y) at a distance along the closed path."""
        if not math.isfinite(distance):
            raise ValueError("Distance must be finite.")

        distance %= self.total_length
        index = bisect_right(self.cumulative_distances, distance) - 1

        fraction = (
            (distance - self.cumulative_distances[index])
            / self.segment_lengths[index]
        )

        start = self.points[index]
        end = self.points[(index + 1) % len(self.points)]

        return (
            start[0] + fraction * (end[0] - start[0]),
            start[1] + fraction * (end[1] - start[1]),
        )

    def widths_at(self, distance):
        """Interpolate official (right, left) widths, including the seam."""
        if self.widths is None:
            raise ValueError("Reference path has no track widths.")
        if not math.isfinite(distance):
            raise ValueError("Distance must be finite.")
        distance %= self.total_length
        index = bisect_right(self.cumulative_distances, distance) - 1
        fraction = ((distance - self.cumulative_distances[index])
                    / self.segment_lengths[index])
        return tuple(a + fraction * (b - a) for a, b in zip(
            self.widths[index], self.widths[(index + 1) % len(self.points)]))

    def project(self, x, y):
        """Return (progress, distance_to_path), both measured in metres."""
        if not math.isfinite(x) or not math.isfinite(y):
            raise ValueError("Position must be finite.")

        best_error = math.inf
        best_progress = 0.0

        for index, start in enumerate(self.points):
            end = self.points[(index + 1) % len(self.points)]
            dx = end[0] - start[0]
            dy = end[1] - start[1]
            length = self.segment_lengths[index]

            fraction = (
                (x - start[0]) * dx + (y - start[1]) * dy
            ) / (length * length)
            fraction = max(0.0, min(1.0, fraction))

            nearest_x = start[0] + fraction * dx
            nearest_y = start[1] + fraction * dy
            error = math.hypot(x - nearest_x, y - nearest_y)

            if error < best_error:
                best_error = error
                best_progress = (
                    self.cumulative_distances[index] + fraction * length
                )

        return best_progress % self.total_length, best_error
