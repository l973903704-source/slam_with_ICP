# -*- coding: utf-8 -*-
import numpy as np
from scipy.optimize import least_squares

from slamware_loop_slam.pose import compose_pose_2d, invert_pose_2d, wrap_angle


def edge_residual(pose_i, pose_j, measurement):
    pred_ij = compose_pose_2d(invert_pose_2d(pose_i), pose_j)
    err = compose_pose_2d(invert_pose_2d(measurement), pred_ij)
    err[2] = wrap_angle(err[2])
    return err


def optimize_pose_graph(initial_poses, edges, max_nfev=100):
    initial_poses = np.asarray(initial_poses, dtype=float)
    num_nodes = initial_poses.shape[0]
    if num_nodes <= 1 or len(edges) == 0:
        return initial_poses.copy()

    x0 = initial_poses[1:].reshape(-1)

    def unpack(x_flat):
        poses = np.zeros_like(initial_poses)
        poses[0] = initial_poses[0]
        poses[1:] = x_flat.reshape(num_nodes - 1, 3)
        poses[:, 2] = np.arctan2(np.sin(poses[:, 2]), np.cos(poses[:, 2]))
        return poses

    def residuals(x_flat):
        poses = unpack(x_flat)
        res = []
        for edge in edges:
            i = int(edge['i'])
            j = int(edge['j'])
            meas = np.asarray(edge['measurement'], dtype=float)
            sqrt_info = np.asarray(edge['sqrt_info'], dtype=float)
            r = edge_residual(poses[i], poses[j], meas)
            res.append(sqrt_info * r)
        if not res:
            return np.zeros(0, dtype=float)
        return np.concatenate(res, axis=0)

    result = least_squares(
        residuals,
        x0,
        method='trf',
        loss='soft_l1',
        f_scale=1.0,
        max_nfev=max_nfev,
        verbose=0,
    )
    return unpack(result.x)
