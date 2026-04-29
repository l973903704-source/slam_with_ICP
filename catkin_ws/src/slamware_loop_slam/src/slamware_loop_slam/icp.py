# -*- coding: utf-8 -*-
import numpy as np
from scipy.spatial import cKDTree

from slamware_loop_slam.align import apply_rigid_transform_2d, estimate_rigid_transform_2d


def icp_2d(source_points, target_points, max_iterations=30, tolerance=1e-6,
           max_correspondence_distance=None, trim_percentile=90.0,
           min_correspondences=20, use_centroid_init=False):
    source_points = np.asarray(source_points, dtype=float)
    target_points = np.asarray(target_points, dtype=float)

    if source_points.ndim != 2 or source_points.shape[1] != 2:
        raise ValueError('source_points must have shape (N, 2).')
    if target_points.ndim != 2 or target_points.shape[1] != 2:
        raise ValueError('target_points must have shape (M, 2).')
    if source_points.shape[0] < 2 or target_points.shape[0] < 2:
        raise ValueError('Both point sets must contain at least two points.')

    transformed_source = source_points.copy()
    R_total = np.eye(2)
    t_total = np.zeros(2)

    if use_centroid_init:
        source_centroid = transformed_source.mean(axis=0)
        target_centroid = target_points.mean(axis=0)
        init_t = target_centroid - source_centroid
        transformed_source = transformed_source + init_t
        t_total = init_t.copy()

    tree = cKDTree(target_points)

    errors = []
    prev_error = np.inf
    best_error = np.inf
    R_best = R_total.copy()
    t_best = t_total.copy()
    transformed_best = transformed_source.copy()

    trim_percentile = float(trim_percentile)
    trim_percentile = min(max(trim_percentile, 1.0), 100.0)
    min_correspondences = int(max(min_correspondences, 2))
    required_matches = min(min_correspondences, source_points.shape[0], target_points.shape[0])

    for _ in range(max_iterations):
        distances, indices = tree.query(transformed_source, k=1)

        mask = np.isfinite(distances)
        if max_correspondence_distance is not None and max_correspondence_distance > 0.0:
            mask &= distances <= float(max_correspondence_distance)

        if np.count_nonzero(mask) < required_matches:
            raise ValueError('ICP found too few correspondences.')

        if trim_percentile < 100.0:
            threshold = np.percentile(distances[mask], trim_percentile)
            mask &= distances <= threshold

        if np.count_nonzero(mask) < required_matches:
            raise ValueError('ICP kept too few correspondences after trimming.')

        matched_source = transformed_source[mask]
        matched_target = target_points[indices[mask]]

        dR, dt, _ = estimate_rigid_transform_2d(matched_source, matched_target)
        transformed_source = apply_rigid_transform_2d(transformed_source, dR, dt)

        R_total = np.dot(dR, R_total)
        t_total = np.dot(t_total, dR.T) + dt

        new_distances, _ = tree.query(transformed_source, k=1)
        error_mask = np.isfinite(new_distances)
        if max_correspondence_distance is not None and max_correspondence_distance > 0.0:
            error_mask &= new_distances <= float(max_correspondence_distance)
        if np.count_nonzero(error_mask) >= required_matches:
            if trim_percentile < 100.0:
                error_threshold = np.percentile(new_distances[error_mask], trim_percentile)
                error_mask &= new_distances <= error_threshold
            current_error = float(np.mean(new_distances[error_mask]))
        else:
            current_error = float(np.mean(new_distances))
        errors.append(current_error)

        if current_error < best_error:
            best_error = current_error
            R_best = R_total.copy()
            t_best = t_total.copy()
            transformed_best = transformed_source.copy()

        if abs(prev_error - current_error) < tolerance:
            break
        prev_error = current_error

    theta_best = float(np.arctan2(R_best[1, 0], R_best[0, 0]))
    return R_best, t_best, theta_best, transformed_best, errors
