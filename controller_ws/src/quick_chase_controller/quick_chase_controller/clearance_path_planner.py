"""Offline periodic clearance optimization; no ROS dependencies.

Paths describe the rear axle. Unknown pixels and map boundaries are blocked.
Planning failures are explicit; the caller chooses an independently validated
alternative or stops.
"""
import math
import time

import numpy as np

from scipy.interpolate import CubicSpline
from scipy.ndimage import distance_transform_edt, map_coordinates
from scipy.optimize import minimize

from .reference_path import ReferencePath


class PlanningError(RuntimeError):
    """No improved, validated path could be produced."""


class ClearancePathPlanner:
    """Optimize periodic lateral offsets inside the official route corridor."""

    def __init__(self, length=.58, width=.31, rear_axle_to_center=.17145,
                    wheelbase=.3302, steering_limit=.4189, minimum_clearance=.10,
                    max_iterations=180, knot_count=24, sample_spacing=.10):
        values = (length, width, wheelbase, steering_limit, minimum_clearance,
                    sample_spacing)
        if not all(math.isfinite(v) and v > 0 for v in values):
            raise ValueError('Planner geometry must be positive and finite.')
        if not math.isfinite(rear_axle_to_center) or knot_count < 4:
            raise ValueError('Invalid axle offset or knot count.')
        self.length, self.width = length, width
        self.rear_axle_to_center = rear_axle_to_center
        self.maximum_curvature = math.tan(steering_limit) / wheelbase
        self.minimum_clearance = minimum_clearance
        self.max_iterations, self.knot_count = max_iterations, knot_count
        self.sample_spacing = sample_spacing
        self.diagnostics = {}

    def _prepare_map(self, occupancy_map):
        self.map = occupancy_map
        pixels = np.asarray(occupancy_map.image, dtype=float) / 255
        occupancy = pixels if occupancy_map.negate else 1 - pixels
        free = occupancy < occupancy_map.free_threshold
        # Padding is essential: even an all-white image has blocked boundaries.
        self.field = distance_transform_edt(np.pad(free, 1))[1:-1, 1:-1]
        self.field *= occupancy_map.resolution

    def _distance(self, positions):
        ox, oy, yaw = self.map.origin
        delta = positions - np.array([ox, oy])
        c, s = math.cos(yaw), math.sin(yaw)
        x = (c * delta[..., 0] + s * delta[..., 1]) / self.map.resolution - .5
        local_y = -s * delta[..., 0] + c * delta[..., 1]
        y = self.field.shape[0] - .5 - local_y / self.map.resolution
        distances = map_coordinates(
            self.field, [y.ravel(), x.ravel()], order=1,
            mode='constant', cval=0).reshape(x.shape)
        # Conservative bound for pixel extents and bilinear interpolation.
        return np.maximum(0, distances - math.sqrt(2) * self.map.resolution)

    def _footprint_distances(self, points, headings, dense=False):
        spacing = min(.025, self.map.resolution / 2) if dense else None
        if dense:
            forward = np.linspace(-self.length / 2, self.length / 2,
                                    math.ceil(self.length / spacing) + 1)
            lateral = np.linspace(-self.width / 2, self.width / 2,
                                    math.ceil(self.width / spacing) + 1)
        else:
            forward = [-self.length / 2, 0, self.length / 2]
            lateral = [-self.width / 2, 0, self.width / 2]
        f, lateral_grid = np.meshgrid(forward, lateral)
        f = f.ravel() + self.rear_axle_to_center
        lateral_grid = lateral_grid.ravel()
        tangent = np.column_stack((np.cos(headings), np.sin(headings)))
        normal = np.column_stack((-np.sin(headings), np.cos(headings)))
        positions = points[:, None, :] + tangent[:, None, :] * f[None, :, None]
        positions += normal[:, None, :] * lateral_grid[None, :, None]
        distances = self._distance(positions)
        return distances.min(axis=1) - (spacing / math.sqrt(2) if dense else 0)

    def plan(self, reference_path, occupancy_map):
        """Return a new path or raise PlanningError, retaining diagnostics."""
        started = time.monotonic()
        self.diagnostics = {
            'success': False, 'min_clearance_baseline': None,
            'min_clearance_planned': None,
        }
        try:
            return self._plan(reference_path, occupancy_map)
        except PlanningError as error:
            self.diagnostics['reason'] = str(error)
            raise
        except ValueError as error:
            reason = 'Invalid planned path geometry: ' + str(error)
            self.diagnostics['reason'] = reason
            raise PlanningError(reason) from error
        finally:
            self.diagnostics['planning_seconds'] = time.monotonic() - started

    def _plan(self, reference, occupancy_map):
        if reference.widths is None:
            raise PlanningError('Official route widths are required.')
        self._prepare_map(occupancy_map)
        total = reference.total_length
        # Periodic geometry removes heading discontinuities at the seam.
        base = CubicSpline(reference.cumulative_distances,
                            np.vstack((reference.points, reference.points[0])),
                            bc_type='periodic')
        knots = np.linspace(0, total, self.knot_count + 1)
        n = max(100, math.ceil(total / self.sample_spacing))
        samples = np.linspace(0, total, n, endpoint=False)
        basis = CubicSpline(knots, np.vstack((np.eye(self.knot_count),
                                                np.eye(self.knot_count)[0])),
                            bc_type='periodic')(samples)
        base_points = base(samples)
        derivatives = base(samples, 1)
        normals = np.column_stack((-derivatives[:, 1], derivatives[:, 0]))
        normals /= np.linalg.norm(normals, axis=1)[:, None]
        widths = np.array([reference.widths_at(s) for s in samples])

        def geometry(offsets, distances=samples):
            # The final curve itself is a periodic cubic interpolant.
            points = base_points + (basis @ offsets)[:, None] * normals
            curve = CubicSpline(
                np.r_[samples, total], np.vstack((points, points[0])),
                bc_type='periodic')
            first, second = curve(distances, 1), curve(distances, 2)
            speed = np.linalg.norm(first, axis=1)
            cross = first[:, 0] * second[:, 1] - first[:, 1] * second[:, 0]
            curvature = cross / np.maximum(speed, 1e-9) ** 3
            headings = np.arctan2(first[:, 1], first[:, 0])
            return curve(distances), headings, curvature, curve

        baseline_s = np.linspace(
            0, total, math.ceil(total / .02), endpoint=False)
        # Measure the supplied polyline as the baseline.
        baseline_points = np.array([reference.point_at(s) for s in baseline_s])
        indices = np.searchsorted(
            reference.cumulative_distances, baseline_s, side='right') - 1
        original = np.array(reference.points)
        segment = original[(indices + 1) % len(original)] - original[indices]
        baseline_heading = np.arctan2(segment[:, 1], segment[:, 0])
        baseline_clearance = float(self._footprint_distances(
            baseline_points, baseline_heading, True).min())
        self.diagnostics['min_clearance_baseline'] = baseline_clearance

        def objective(z):
            # Maximize minimum clearance with gentle regularity penalties.
            offsets = z[:-1]
            roughness = (np.roll(offsets, -1) - 2 * offsets
                            + np.roll(offsets, 1))
            return (-z[-1] + .025 * np.mean(z[:-1] ** 2)
                    + .05 * np.mean(roughness ** 2))

        def constraints(z):
            p, h, curvature, _ = geometry(z[:-1])
            offsets = basis @ z[:-1]
            clearance = self._footprint_distances(p, h)
            return np.r_[
                clearance - z[-1], self.maximum_curvature - np.abs(curvature),
                widths[:, 0] + offsets, widths[:, 1] - offsets]

        initial = np.zeros(self.knot_count + 1)
        initial[-1] = max(0, baseline_clearance)
        bounds = [(-float(widths[:, 0].max()), float(widths[:, 1].max()))]
        result = minimize(
            objective, initial, method='SLSQP',
            bounds=bounds * self.knot_count + [(0, None)],
            constraints={'type': 'ineq', 'fun': constraints},
            options={'maxiter': self.max_iterations, 'ftol': 1e-6})
        self.diagnostics.update(
            optimizer_message=str(result.message), iterations=int(result.nit))
        if not result.success or not np.isfinite(result.x).all():
            raise PlanningError(
                'Optimization did not converge: ' + str(result.message))
        if constraints(result.x).min() < -1e-5:
            raise PlanningError(
                'Optimized path violates corridor or curvature constraints.')
        # Include the seam and bound final sample spacing using curve speed.
        _, _, _, curve = geometry(result.x[:-1])
        fine = np.linspace(0, total, math.ceil(total / .01), endpoint=False)
        max_speed = float(np.linalg.norm(curve(fine, 1), axis=1).max())
        dense_s = np.linspace(
            0, total, math.ceil(total * max_speed / .02), endpoint=False)
        points, headings, curvature, _ = geometry(result.x[:-1], dense_s)
        clearance = float(
            self._footprint_distances(points, headings, True).min())
        # Recheck the corridor between optimization stations.
        dense_base_derivative = base(dense_s, 1)
        dense_normals = np.column_stack((
            -dense_base_derivative[:, 1], dense_base_derivative[:, 0]))
        dense_normals /= np.linalg.norm(dense_normals, axis=1)[:, None]
        dense_offset = np.sum((points - base(dense_s)) * dense_normals, axis=1)
        dense_widths = np.array([reference.widths_at(s) for s in dense_s])
        if (np.any(dense_offset < -dense_widths[:, 0] - 1e-5)
                or np.any(dense_offset > dense_widths[:, 1] + 1e-5)):
            raise PlanningError('Dense path exceeds official width corridor.')
        steps = np.linalg.norm(np.roll(points, -1, axis=0) - points, axis=1)
        if steps.max() > .025:
            raise PlanningError('Footprint sample spacing exceeds 0.025 m.')
        self.diagnostics['min_clearance_planned'] = clearance
        self.curvature = curvature
        if np.max(np.abs(curvature)) > self.maximum_curvature + 1e-5:
            raise PlanningError('Dense curvature exceeds steering limit.')
        if clearance < self.minimum_clearance:
            raise PlanningError('Full footprint has insufficient clearance.')
        if clearance <= baseline_clearance + 1e-4:
            raise PlanningError(
                'Full footprint clearance did not improve over baseline.')
        # Exact rectangle check includes the requested wall margin.
        for p, h in zip(points, headings):
            tangent = np.array([math.cos(h), math.sin(h)])
            center = p + self.rear_axle_to_center * tangent
            if not occupancy_map.is_rectangle_free(
                *center, h, self.length + 2 * self.minimum_clearance,
                self.width + 2 * self.minimum_clearance
            ):
                raise PlanningError('Exact rectangle footprint check failed.')
        residual_widths = np.column_stack((
            dense_widths[:, 0] + dense_offset,
            dense_widths[:, 1] - dense_offset))
        if np.any(residual_widths <= 0):
            raise PlanningError(
                'Planned path leaves no positive corridor width.')
        planned = ReferencePath(points, widths=residual_widths)
        self.diagnostics.update(
            success=True, reason='validated', sample_count=len(points),
            max_curvature=float(np.max(np.abs(curvature))))
        return planned