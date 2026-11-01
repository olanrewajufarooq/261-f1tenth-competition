"""Tests for reference-path loading and geometry."""

from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

from quick_chase_controller.reference_path import ReferencePath


class TestReferencePath(unittest.TestCase):
    def setUp(self):
        # Four 2-metre sides: total length is 8 metres.
        self.square = ReferencePath([
            (0, 0), (2, 0), (2, 2), (0, 2),
        ])

    def test_loads_coordinates_and_skips_comments(self):
        with TemporaryDirectory() as directory:
            filename = Path(directory) / "route.csv"
            filename.write_text(
                "# x, y, right width, left width\n"
                "\n"
                "0, 0, 1, 1\n"
                "2, 0, 1, 1\n"
                "2, 2, 1, 1\n",
                encoding="utf-8",
            )
            path = ReferencePath.from_csv(filename)

        self.assertEqual(path.widths, ((1.0, 1.0),) * 3)

        self.assertEqual(
            path.points,
            ((0.0, 0.0), (2.0, 0.0), (2.0, 2.0)),
        )

    def test_rejects_insufficient_points(self):
        with self.assertRaises(ValueError):
            ReferencePath([(0, 0), (1, 0)])

    def test_rejects_nonfinite_coordinates(self):
        with self.assertRaises(ValueError):
            ReferencePath([(0, 0), (1, 0), (float("nan"), 1)])

    def test_rejects_zero_length_segments(self):
        with self.assertRaises(ValueError):
            ReferencePath([(0, 0), (0, 0), (1, 1)])

    def test_interpolates_between_waypoints(self):
        self.assertAlmostEqual(self.square.total_length, 8.0)
        self.assertEqual(self.square.point_at(1.0), (1.0, 0.0))
        self.assertEqual(self.square.point_at(3.0), (2.0, 1.0))

    def test_wraps_across_finish_line(self):
        self.assertEqual(self.square.point_at(7.0), (0.0, 1.0))
        self.assertEqual(self.square.point_at(8.0), (0.0, 0.0))
        self.assertEqual(self.square.point_at(9.0), (1.0, 0.0))
        self.assertEqual(self.square.point_at(-1.0), (0.0, 1.0))

    def test_projects_onto_segment_interior(self):
        progress, error = self.square.project(1.0, -0.2)
        self.assertAlmostEqual(progress, 1.0)
        self.assertAlmostEqual(error, 0.2)

    def test_projects_onto_closing_segment(self):
        progress, error = self.square.project(-0.2, 1.0)
        self.assertAlmostEqual(progress, 7.0)
        self.assertAlmostEqual(error, 0.2)

    def test_projection_stays_within_segment_endpoints(self):
        progress, error = self.square.project(3.0, -1.0)
        self.assertAlmostEqual(progress, 2.0)
        self.assertAlmostEqual(error, 2.0 ** 0.5)

    def test_rejects_nonfinite_queries(self):
        with self.assertRaises(ValueError):
            self.square.point_at(float("inf"))

        with self.assertRaises(ValueError):
            self.square.project(float("nan"), 0.0)


if __name__ == "__main__":
    unittest.main()