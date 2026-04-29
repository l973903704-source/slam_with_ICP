# -*- coding: utf-8 -*-
import numpy as np


def wrap_angle(theta):
    return float(np.arctan2(np.sin(theta), np.cos(theta)))


def compose_pose_2d(pose, delta):
    x, y, theta = pose
    dx, dy, dtheta = delta

    c = np.cos(theta)
    s = np.sin(theta)

    x_new = x + c * dx - s * dy
    y_new = y + s * dx + c * dy
    theta_new = wrap_angle(theta + dtheta)
    return np.array([x_new, y_new, theta_new], dtype=float)


def invert_pose_2d(pose):
    dx, dy, dtheta = pose
    c = np.cos(dtheta)
    s = np.sin(dtheta)

    inv_theta = wrap_angle(-dtheta)
    inv_dx = -(c * dx + s * dy)
    inv_dy = -(-s * dx + c * dy)
    return np.array([inv_dx, inv_dy, inv_theta], dtype=float)


def invert_delta_pose_2d(delta):
    return invert_pose_2d(delta)


def relative_pose_2d(pose_a, pose_b):
    return compose_pose_2d(invert_pose_2d(pose_a), pose_b)


def pose_distance_xy(pose_a, pose_b):
    return float(np.linalg.norm(pose_a[:2] - pose_b[:2]))
