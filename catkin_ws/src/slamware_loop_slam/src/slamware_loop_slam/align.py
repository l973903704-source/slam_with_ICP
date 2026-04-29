# -*- coding: utf-8 -*-
import numpy as np


def estimate_rigid_transform_2d(source_points, target_points):
    if source_points.shape != target_points.shape:
        raise ValueError('Point sets must have the same shape.')
    if source_points.ndim != 2 or source_points.shape[1] != 2:
        raise ValueError('Point sets must have shape (N, 2).')
    if source_points.shape[0] < 2:
        raise ValueError('At least two points are required.')

    source_centroid = source_points.mean(axis=0)
    target_centroid = target_points.mean(axis=0)

    source_centered = source_points - source_centroid
    target_centered = target_points - target_centroid

    H = np.dot(source_centered.T, target_centered)
    U, _, Vt = np.linalg.svd(H)

    R = np.dot(Vt.T, U.T)
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1.0
        R = np.dot(Vt.T, U.T)

    t = target_centroid - np.dot(source_centroid, R.T)
    theta = np.arctan2(R[1, 0], R[0, 0])
    return R, t, theta


def apply_rigid_transform_2d(points, R, t):
    return np.dot(points, R.T) + t
