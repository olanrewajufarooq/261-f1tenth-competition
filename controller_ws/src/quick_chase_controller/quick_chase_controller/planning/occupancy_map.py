"""Static occupancy-map queries independent of ROS."""

import math
from pathlib import Path

from PIL import Image
import yaml


class OccupancyMap:
    """Query free space using ROS map-image conventions."""

    def __init__(
        self,
        image,
        resolution,
        origin,
        free_threshold=0.196,
        negate=False,
    ):
        origin_x, origin_y, origin_yaw = origin

        if not all(math.isfinite(value) for value in (
            resolution, origin_x, origin_y, origin_yaw, free_threshold
        )):
            raise ValueError("Map geometry and threshold must be finite.")

        if resolution <= 0.0:
            raise ValueError("Map resolution must be positive.")

        if not 0.0 <= free_threshold < 1.0:
            raise ValueError("Free threshold must be in [0, 1).")

        self.image = image.convert("L")
        self.resolution = resolution
        self.origin = (origin_x, origin_y, origin_yaw)
        self.free_threshold = free_threshold
        self.negate = bool(negate)

    @classmethod
    def from_yaml(cls, filename):
        """Load an image and its geometry from a map YAML file."""
        filename = Path(filename)

        with filename.open(encoding="utf-8") as stream:
            metadata = yaml.safe_load(stream)

        image_path = filename.parent / metadata["image"]

        with Image.open(image_path) as image:
            return cls(
                image=image,
                resolution=float(metadata["resolution"]),
                origin=metadata["origin"],
                free_threshold=float(metadata["free_thresh"]),
                negate=metadata.get("negate", 0),
            )

    def is_free(self, x, y):
        """Return False for occupied, unknown, or out-of-map positions."""
        if not math.isfinite(x) or not math.isfinite(y):
            return False

        origin_x, origin_y, yaw = self.origin
        dx = x - origin_x
        dy = y - origin_y

        local_x = math.cos(yaw) * dx + math.sin(yaw) * dy
        local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy

        column = math.floor(local_x / self.resolution)
        row_from_bottom = math.floor(local_y / self.resolution)

        width, height = self.image.size

        if not (
            0 <= column < width
            and 0 <= row_from_bottom < height
        ):
            return False

        # Image rows increase downward; map coordinates increase upward.
        row = height - 1 - row_from_bottom
        brightness = self.image.getpixel((column, row)) / 255.0
        occupancy = brightness if self.negate else 1.0 - brightness

        return occupancy < self.free_threshold

    def is_rectangle_free(self, center_x, center_y, yaw, length, width):
        """Check a world-frame rectangle against occupied map cells.

        Unknown and out-of-map areas count as blocked. Cell-size padding
        makes the check conservative when a rectangle touches a cell.
        """
        values = (center_x, center_y, yaw, length, width)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("Rectangle inputs must be finite.")

        if length <= 0.0 or width <= 0.0:
            raise ValueError("Rectangle dimensions must be positive.")

        origin_x, origin_y, map_yaw = self.origin
        map_cos = math.cos(map_yaw)
        map_sin = math.sin(map_yaw)

        dx = center_x - origin_x
        dy = center_y - origin_y

        # Rectangle centre and orientation in map-local coordinates.
        local_x = map_cos * dx + map_sin * dy
        local_y = -map_sin * dx + map_cos * dy
        relative_yaw = yaw - map_yaw

        cosine = math.cos(relative_yaw)
        sine = math.sin(relative_yaw)
        half_length = length / 2.0
        half_width = width / 2.0

        # Any point in a cell is within this distance of its centre.
        cell_radius = self.resolution / math.sqrt(2.0)

        extent_x = (
            abs(cosine) * half_length + abs(sine) * half_width
        )
        extent_y = (
            abs(sine) * half_length + abs(cosine) * half_width
        )

        first_column = math.floor(
            (local_x - extent_x - cell_radius) / self.resolution
        )
        last_column = math.floor(
            (local_x + extent_x + cell_radius) / self.resolution
        )
        first_row = math.floor(
            (local_y - extent_y - cell_radius) / self.resolution
        )
        last_row = math.floor(
            (local_y + extent_y + cell_radius) / self.resolution
        )

        image_width, image_height = self.image.size

        for row in range(first_row, last_row + 1):
            for column in range(first_column, last_column + 1):
                cell_x = (column + 0.5) * self.resolution
                cell_y = (row + 0.5) * self.resolution

                offset_x = cell_x - local_x
                offset_y = cell_y - local_y

                forward = cosine * offset_x + sine * offset_y
                lateral = -sine * offset_x + cosine * offset_y

                if (
                    abs(forward) > half_length + cell_radius
                    or abs(lateral) > half_width + cell_radius
                ):
                    continue

                if not (
                    0 <= column < image_width
                    and 0 <= row < image_height
                ):
                    return False

                world_x = origin_x + map_cos * cell_x - map_sin * cell_y
                world_y = origin_y + map_sin * cell_x + map_cos * cell_y

                if not self.is_free(world_x, world_y):
                    return False

        return True