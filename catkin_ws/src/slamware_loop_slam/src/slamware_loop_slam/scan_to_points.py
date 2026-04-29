# -*- coding: utf-8 -*-
import numpy as np


def scan_to_points(ranges, angles):
    x = ranges * np.cos(angles)
    y = ranges * np.sin(angles)
    return np.stack([x, y], axis=1)


def laser_scan_to_points(msg, min_range=0.05, max_range=20.0, downsample_step=1):
    ranges = np.array(msg.ranges, dtype=np.float64)
    angles = msg.angle_min + np.arange(len(ranges), dtype=np.float64) * msg.angle_increment

    msg_min = msg.range_min if np.isfinite(msg.range_min) and msg.range_min > 0.0 else min_range
    msg_max = msg.range_max if np.isfinite(msg.range_max) and msg.range_max > 0.0 else max_range
    effective_min = max(float(min_range), float(msg_min))
    effective_max = min(float(max_range), float(msg_max))

    valid = np.isfinite(ranges)
    valid &= ranges >= effective_min
    valid &= ranges <= effective_max

    ranges = ranges[valid]
    angles = angles[valid]

    downsample_step = int(max(downsample_step, 1))
    if downsample_step > 1:
        ranges = ranges[::downsample_step]
        angles = angles[::downsample_step]

    if ranges.size == 0:
        return np.empty((0, 2), dtype=np.float64)

    return scan_to_points(ranges, angles)
