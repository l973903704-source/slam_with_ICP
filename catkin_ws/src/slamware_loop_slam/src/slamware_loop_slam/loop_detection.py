# -*- coding: utf-8 -*-
import numpy as np


class LoopCandidate(object):
    def __init__(self, keyframe_id, descriptor_distance, pose_distance):
        self.keyframe_id = keyframe_id
        self.descriptor_distance = descriptor_distance
        self.pose_distance = pose_distance


def make_radial_descriptor(points, max_radius, num_bins):
    if points.size == 0:
        return np.zeros(num_bins, dtype=float)

    radii = np.linalg.norm(points, axis=1)
    hist, _ = np.histogram(radii, bins=num_bins, range=(0.0, max_radius))
    desc = hist.astype(float)
    norm = np.linalg.norm(desc)
    if norm > 1e-12:
        desc /= norm
    return desc


def descriptor_distance(desc_a, desc_b):
    return float(np.linalg.norm(desc_a - desc_b))


def find_loop_candidates(current_id, current_pose, current_descriptor, keyframes,
                         min_index_gap, max_pose_distance, max_descriptor_distance,
                         max_candidates=3):
    candidates = []
    for kf in keyframes:
        kf_id = int(kf['id'])
        if current_id - kf_id < min_index_gap:
            continue

        pose_dist = float(np.linalg.norm(current_pose[:2] - kf['local_pose'][:2]))
        if max_pose_distance > 0.0 and pose_dist > max_pose_distance:
            continue

        desc_dist = descriptor_distance(current_descriptor, kf['descriptor'])
        if desc_dist > max_descriptor_distance:
            continue

        candidates.append(LoopCandidate(kf_id, desc_dist, pose_dist))

    candidates.sort(key=lambda c: (c.descriptor_distance, c.pose_distance, c.keyframe_id))
    return candidates[:max_candidates]
