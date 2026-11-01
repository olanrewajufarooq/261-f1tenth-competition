"""Safety and closure contracts for the offline planner."""
import numpy as np
from PIL import Image
import pytest

from quick_chase_controller.planning.reference_path import ReferencePath
from quick_chase_controller.planning.occupancy_map import OccupancyMap
from quick_chase_controller.planning.clearance_path_planner import ClearancePathPlanner, PlanningError


def ring(radius=3.4):
    angles = np.linspace(0, 2 * np.pi, 120, endpoint=False)
    return ReferencePath([(5 + radius * np.cos(a), 5 + radius * np.sin(a)) for a in angles],
                            widths=[(1.2, 1.2)] * len(angles))


def annulus():
    y, x = np.mgrid[:200, :200]
    radius = np.hypot((x + .5) * .05 - 5, (199 - y + .5) * .05 - 5)
    return OccupancyMap(Image.fromarray(np.where((radius > 2) & (radius < 4), 255, 0).astype('uint8')),
                        .05, (0, 0, 0))


def test_retains_widths_and_interpolates_at_seam():
    path = ring()
    assert path.widths_at(path.total_length + .1) == pytest.approx((1.2, 1.2))
    assert path.widths == ((1.2, 1.2),) * 120


def test_optimizes_clearance_and_preserves_baseline():
    baseline = ring()
    original = baseline.points
    planner = ClearancePathPlanner(max_iterations=120)
    planned = planner.plan(baseline, annulus())
    assert baseline.points == original
    assert planner.diagnostics['success']
    assert planner.diagnostics['min_clearance_planned'] > planner.diagnostics['min_clearance_baseline'] + .01
    assert planner.diagnostics['min_clearance_planned'] >= .1
    assert np.max(np.abs(planner.curvature)) <= np.tan(.4189) / .3302 + 1e-5
    assert np.linalg.norm(np.subtract(planned.points[0], planned.points[-1])) <= .0251
    occupancy = annulus()
    points = np.asarray(planned.points)
    for index in range(0, len(points), 20):
        tangent = points[(index + 1) % len(points)] - points[index - 1]
        yaw = np.arctan2(tangent[1], tangent[0])
        center = points[index] + .17145 * np.array([np.cos(yaw), np.sin(yaw)])
        assert occupancy.is_rectangle_free(*center, yaw, .78, .51)


def test_rejects_blocked_map_without_returning_baseline():
    planner = ClearancePathPlanner(max_iterations=10)
    blocked = OccupancyMap(Image.new('L', (200, 200), 0), .05, (0, 0, 0))
    with pytest.raises(PlanningError):
        planner.plan(ring(), blocked)
    assert not planner.diagnostics['success']


def test_requires_official_widths():
    with pytest.raises(PlanningError, match='width'):
        ClearancePathPlanner().plan(ReferencePath([(0, 0), (1, 0), (0, 1)]), annulus())


def test_nonconvergence_is_explicit_and_diagnostics_recorded():
    planner = ClearancePathPlanner(max_iterations=1)
    with pytest.raises(PlanningError, match='converge'):
        planner.plan(ring(), annulus())
    assert not planner.diagnostics['success']
    assert planner.diagnostics['planning_seconds'] > 0


def test_map_boundary_and_unknown_are_blocked():
    planner = ClearancePathPlanner()
    pixels = np.full((100, 100), 255, dtype='uint8')
    pixels[50, 50] = 205
    planner._prepare_map(OccupancyMap(Image.fromarray(pixels), .05, (0, 0, 0)))
    assert planner._distance(np.array([[2.525, 2.475], [-.1, 2.0], [.01, 2.0]])).tolist() == [0, 0, 0]


def test_rejects_insufficient_full_body_clearance():
    baseline = ring()
    planner = ClearancePathPlanner(minimum_clearance=2)
    with pytest.raises(PlanningError, match='insufficient'):
        planner.plan(baseline, annulus())
    assert not planner.diagnostics['success']
    assert planner.diagnostics['min_clearance_planned'] < 2


def test_corridor_edge_rejection_never_reports_success_or_raw_value_error():
    baseline = ring()
    baseline = ReferencePath(
        baseline.points, widths=[(1.2, .10)] * len(baseline.points))
    planner = ClearancePathPlanner()
    with pytest.raises(PlanningError, match='positive corridor width'):
        planner.plan(baseline, annulus())
    assert not planner.diagnostics['success']
    assert planner.diagnostics['reason']

