# -*- coding: utf-8 -*-
import numpy as np


def rotation_matrix(theta):
    c = np.cos(theta)
    s = np.sin(theta)
    return np.array([[c, -s], [s, c]], dtype=float)


def transform_points(points, dx, dy, dtheta):
    R = rotation_matrix(dtheta)
    t = np.array([dx, dy], dtype=float)
    return np.dot(points, R.T) + t
