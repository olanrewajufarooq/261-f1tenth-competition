"""Tests for map-coordinate and occupancy queries."""

import math
import unittest

from PIL import Image

from quick_chase_controller.occupancy_map import OccupancyMap


class TestOccupancyMap(unittest.TestCase):
    def setUp(self):
        # White map, with one occupied cell at its bottom-left corner.
        image = Image.new("L", (3, 3), 255)
        image.putpixel((0, 2), 0)

        self.map = OccupancyMap(
            image, resolution=1.0, origin=(0.0, 0.0, 0.0)
        )

    def test_detects_occupied_cell_with_image_row_flip(self):
        self.assertFalse(self.map.is_free(0.5, 0.5))
        self.assertTrue(self.map.is_free(0.5, 2.5))

    def test_detects_free_cell(self):
        self.assertTrue(self.map.is_free(1.5, 0.5))

    def test_rejects_positions_outside_map(self):
        for position in ((-0.01, 0.5), (3.0, 0.5), (0.5, 3.0)):
            with self.subTest(position=position):
                self.assertFalse(self.map.is_free(*position))

    def test_applies_origin_translation_and_rotation(self):
        image = Image.new("L", (2, 2), 255)
        image.putpixel((0, 1), 0)

        occupancy_map = OccupancyMap(
            image, 1.0, (10.0, 20.0, math.pi / 2.0)
        )

        # Local (0.5, 0.5) becomes world (9.5, 20.5).
        self.assertFalse(occupancy_map.is_free(9.5, 20.5))

        # Local (1.5, 0.5) becomes world (9.5, 21.5).
        self.assertTrue(occupancy_map.is_free(9.5, 21.5))

    def test_treats_unknown_gray_cell_as_blocked(self):
        image = Image.new("L", (1, 1), 128)
        occupancy_map = OccupancyMap(image, 1.0, (0.0, 0.0, 0.0))
        self.assertFalse(occupancy_map.is_free(0.5, 0.5))

    def test_rejects_nonfinite_position(self):
        self.assertFalse(self.map.is_free(float("nan"), 0.5))

    def test_rectangle_is_free_in_open_space(self):
        image = Image.new("L", (40, 40), 255)
        occupancy_map = OccupancyMap(image, 0.1, (0.0, 0.0, 0.0))

        self.assertTrue(
            occupancy_map.is_rectangle_free(2.0, 2.0, 0.0, 0.6, 0.3)
        )

    def test_rectangle_detects_obstacle_inside_its_interior(self):
        image = Image.new("L", (40, 40), 255)
        # Cell centre: (2.05, 2.05).
        image.putpixel((20, 19), 0)
        occupancy_map = OccupancyMap(image, 0.1, (0.0, 0.0, 0.0))

        self.assertFalse(
            occupancy_map.is_rectangle_free(2.0, 2.0, 0.0, 0.6, 0.3)
        )

    def test_rectangle_rotation_changes_collision_result(self):
        image = Image.new("L", (40, 40), 255)
        # Cell centre: (2.05, 2.35).
        image.putpixel((20, 16), 0)
        occupancy_map = OccupancyMap(image, 0.1, (0.0, 0.0, 0.0))

        self.assertTrue(
            occupancy_map.is_rectangle_free(2.0, 2.0, 0.0, 1.0, 0.2)
        )
        self.assertFalse(
            occupancy_map.is_rectangle_free(
                2.0, 2.0, math.pi / 2.0, 1.0, 0.2
            )
        )

    def test_rectangle_cannot_extend_outside_map(self):
        image = Image.new("L", (40, 40), 255)
        occupancy_map = OccupancyMap(image, 0.1, (0.0, 0.0, 0.0))

        self.assertFalse(
            occupancy_map.is_rectangle_free(0.1, 2.0, 0.0, 0.6, 0.3)
        )


if __name__ == "__main__":
    unittest.main()