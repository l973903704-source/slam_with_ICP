#!/usr/bin/env python2
# -*- coding: utf-8 -*-
from __future__ import print_function

import csv
import math
import os

import numpy as np
import rospy
import tf
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan

from slamware_loop_slam.icp import icp_2d
from slamware_loop_slam.loop_detection import find_loop_candidates, make_radial_descriptor
from slamware_loop_slam.pose import compose_pose_2d, invert_pose_2d, relative_pose_2d
from slamware_loop_slam.pose_graph import optimize_pose_graph
from slamware_loop_slam.scan_to_points import laser_scan_to_points


def default_result_prefix(prefix_name):
    package_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
    return os.path.join(package_dir, 'result', prefix_name)


class PoseGraphSLAMNode(object):
    def __init__(self):
        self.scan_topic = rospy.get_param('~scan_topic', '/scan')
        self.local_pose_topic = rospy.get_param('~local_pose_topic', '/slamware_local_pose')
        self.local_path_topic = rospy.get_param('~local_path_topic', '/slamware_local_path')
        self.optimized_pose_topic = rospy.get_param('~optimized_pose_topic', '/slamware_pg_pose')
        self.optimized_path_topic = rospy.get_param('~optimized_path_topic', '/slamware_pg_path')
        self.frame_id = rospy.get_param('~frame_id', 'map')
        self.local_child_frame_id = rospy.get_param('~local_child_frame_id', 'slamware_local_base_link')
        self.optimized_child_frame_id = rospy.get_param('~optimized_child_frame_id', 'slamware_pg_base_link')

        self.min_range = float(rospy.get_param('~min_range', 0.10))
        self.max_range = float(rospy.get_param('~max_range', 40.0))
        self.downsample_step = int(rospy.get_param('~downsample_step', 2))
        self.min_points = int(rospy.get_param('~min_points', 50))
        self.max_iterations = int(rospy.get_param('~max_iterations', 40))
        self.tolerance = float(rospy.get_param('~tolerance', 1e-5))
        self.max_correspondence_distance = float(rospy.get_param('~max_correspondence_distance', 0.60))
        self.trim_percentile = float(rospy.get_param('~trim_percentile', 85.0))
        self.min_correspondences = int(rospy.get_param('~min_correspondences', 30))
        self.use_centroid_init = bool(rospy.get_param('~use_centroid_init', False))
        self.max_icp_error = float(rospy.get_param('~max_icp_error', 0.30))
        self.max_step_translation = float(rospy.get_param('~max_step_translation', 0.60))
        self.max_step_rotation_deg = float(rospy.get_param('~max_step_rotation_deg', 35.0))
        self.max_consecutive_rejects = int(rospy.get_param('~max_consecutive_rejects', 5))
        self.invert_icp_result = bool(rospy.get_param('~invert_icp_result', True))
        self.publish_tf = bool(rospy.get_param('~publish_tf', False))
        self.publish_optimized_tf = bool(rospy.get_param('~publish_optimized_tf', False))
        self.save_csv = bool(rospy.get_param('~save_csv', True))
        self.csv_prefix = rospy.get_param('~csv_prefix', default_result_prefix('pose_graph'))
        self.log_scan_info_once = bool(rospy.get_param('~log_scan_info_once', True))
        self.path_publish_step = int(rospy.get_param('~path_publish_step', 1))
        self.max_path_poses = int(rospy.get_param('~max_path_poses', 3000))
        self.csv_flush_interval = int(rospy.get_param('~csv_flush_interval', 20))
        self.min_correspondences = max(self.min_correspondences, 2)
        self.max_consecutive_rejects = max(self.max_consecutive_rejects, 1)
        self.path_publish_step = max(self.path_publish_step, 1)

        self.keyframe_translation_thresh = float(rospy.get_param('~keyframe_translation_thresh', 0.30))
        self.keyframe_rotation_thresh_deg = float(rospy.get_param('~keyframe_rotation_thresh_deg', 15.0))
        self.loop_min_index_gap = int(rospy.get_param('~loop_min_index_gap', 12))
        self.loop_candidate_distance = float(rospy.get_param('~loop_candidate_distance', 2.00))
        self.loop_descriptor_bins = int(rospy.get_param('~loop_descriptor_bins', 24))
        self.loop_descriptor_threshold = float(rospy.get_param('~loop_descriptor_threshold', 0.45))
        self.loop_icp_error_threshold = float(rospy.get_param('~loop_icp_error_threshold', 0.25))
        self.loop_max_correspondence_distance = float(rospy.get_param('~loop_max_correspondence_distance', 0.80))
        self.loop_max_measurement_translation = float(rospy.get_param('~loop_max_measurement_translation', 1.50))
        self.loop_max_measurement_rotation_deg = float(rospy.get_param('~loop_max_measurement_rotation_deg', 60.0))
        self.loop_max_candidates = int(rospy.get_param('~loop_max_candidates', 3))
        self.optimize_every_n_keyframes = int(rospy.get_param('~optimize_every_n_keyframes', 5))
        self.optimize_on_loop_only = bool(rospy.get_param('~optimize_on_loop_only', True))
        self.max_optimizer_nfev = int(rospy.get_param('~max_optimizer_nfev', 80))

        self.odom_edge_weight_xy = float(rospy.get_param('~odom_edge_weight_xy', 10.0))
        self.odom_edge_weight_theta = float(rospy.get_param('~odom_edge_weight_theta', 15.0))
        self.loop_edge_weight_xy = float(rospy.get_param('~loop_edge_weight_xy', 20.0))
        self.loop_edge_weight_theta = float(rospy.get_param('~loop_edge_weight_theta', 25.0))

        self.prev_points = None
        self.current_local_pose = np.array([0.0, 0.0, 0.0], dtype=float)
        self.current_optimized_pose = self.current_local_pose.copy()
        self.last_keyframe_local_pose = None
        self.frame_count = 0
        self.path_sample_count = 0
        self.consecutive_rejects = 0
        self.keyframes = []
        self.edges = []
        self.loop_pairs = set()
        self.logged_scan_info = False

        self.local_pose_pub = rospy.Publisher(self.local_pose_topic, PoseStamped, queue_size=10)
        self.local_path_pub = rospy.Publisher(self.local_path_topic, Path, queue_size=10)
        self.optimized_pose_pub = rospy.Publisher(self.optimized_pose_topic, PoseStamped, queue_size=10)
        self.optimized_path_pub = rospy.Publisher(self.optimized_path_topic, Path, queue_size=10)
        self.tf_broadcaster = tf.TransformBroadcaster() if (self.publish_tf or self.publish_optimized_tf) else None

        self.local_path_msg = Path()
        self.local_path_msg.header.frame_id = self.frame_id
        self.optimized_path_msg = Path()
        self.optimized_path_msg.header.frame_id = self.frame_id

        self.local_csv_file = None
        self.local_csv_writer = None
        self.optimized_csv_file = None
        self.optimized_csv_writer = None
        self.loop_csv_file = None
        self.loop_csv_writer = None
        self.keyframes_csv_path = None
        if self.save_csv:
            csv_dir = os.path.dirname(self.csv_prefix)
            if csv_dir and not os.path.exists(csv_dir):
                os.makedirs(csv_dir)

            self.local_csv_file = open(self.csv_prefix + '_local.csv', 'wb')
            self.local_csv_writer = csv.writer(self.local_csv_file)
            self.local_csv_writer.writerow(['stamp', 'x', 'y', 'theta_rad'])

            self.optimized_csv_file = open(self.csv_prefix + '_optimized.csv', 'wb')
            self.optimized_csv_writer = csv.writer(self.optimized_csv_file)
            self.optimized_csv_writer.writerow(['stamp', 'x', 'y', 'theta_rad'])

            self.loop_csv_file = open(self.csv_prefix + '_loops.csv', 'wb')
            self.loop_csv_writer = csv.writer(self.loop_csv_file)
            self.loop_csv_writer.writerow(['stamp', 'from_id', 'to_id', 'dx', 'dy', 'dtheta_rad', 'icp_error'])
            self.keyframes_csv_path = self.csv_prefix + '_keyframes.csv'
            rospy.loginfo('Saving pose graph CSVs with prefix %s', self.csv_prefix)

        self.scan_sub = rospy.Subscriber(self.scan_topic, LaserScan, self.scan_callback, queue_size=1)
        rospy.on_shutdown(self.on_shutdown)
        rospy.loginfo('pose_graph_slam_node started on %s', self.scan_topic)

    def on_shutdown(self):
        self._write_keyframes_snapshot()
        for handle in [self.local_csv_file, self.optimized_csv_file, self.loop_csv_file]:
            if handle is not None:
                handle.flush()
                handle.close()

    def _write_keyframes_snapshot(self):
        if self.keyframes_csv_path is None:
            return

        tmp_path = self.keyframes_csv_path + '.tmp'
        with open(tmp_path, 'wb') as handle:
            writer = csv.writer(handle)
            writer.writerow([
                'id',
                'stamp',
                'local_x',
                'local_y',
                'local_theta_rad',
                'optimized_x',
                'optimized_y',
                'optimized_theta_rad',
            ])
            for kf in self.keyframes:
                local_pose = kf['local_pose']
                optimized_pose = kf['optimized_pose']
                writer.writerow([
                    kf['id'],
                    kf['stamp'],
                    float(local_pose[0]),
                    float(local_pose[1]),
                    float(local_pose[2]),
                    float(optimized_pose[0]),
                    float(optimized_pose[1]),
                    float(optimized_pose[2]),
                ])

        if os.path.exists(self.keyframes_csv_path):
            os.remove(self.keyframes_csv_path)
        os.rename(tmp_path, self.keyframes_csv_path)

    def _append_local_path_pose(self, pose_msg):
        self.path_sample_count += 1
        step = max(self.path_publish_step, 1)
        if step > 1 and (self.path_sample_count % step) != 0:
            return
        self.local_path_msg.poses.append(pose_msg)
        if self.max_path_poses > 0 and len(self.local_path_msg.poses) > self.max_path_poses:
            overflow = len(self.local_path_msg.poses) - self.max_path_poses
            del self.local_path_msg.poses[:overflow]

    def _reject_scan(self, points, reason):
        self.consecutive_rejects += 1
        rospy.logwarn_throttle(1.0, '%s; rejected=%d/%d',
                               reason, self.consecutive_rejects, self.max_consecutive_rejects)
        if self.consecutive_rejects >= self.max_consecutive_rejects:
            self.prev_points = points
            self.consecutive_rejects = 0
            rospy.logwarn('Reset ICP reference scan after repeated rejected frames.')

    def _motion_is_usable(self, motion_delta, errors):
        if len(errors) == 0:
            return False, 'ICP returned no error history'

        final_error = float(errors[-1])
        trans = float(np.linalg.norm(motion_delta[:2]))
        rot_deg = abs(float(np.rad2deg(motion_delta[2])))

        if final_error > self.max_icp_error:
            return False, 'ICP error %.4f > %.4f' % (final_error, self.max_icp_error)
        if trans > self.max_step_translation:
            return False, 'ICP translation jump %.4f m > %.4f m' % (trans, self.max_step_translation)
        if rot_deg > self.max_step_rotation_deg:
            return False, 'ICP rotation jump %.2f deg > %.2f deg' % (rot_deg, self.max_step_rotation_deg)
        return True, ''

    def _initial_optimized_pose(self, local_pose):
        if not self.keyframes:
            return local_pose.copy()
        last_kf = self.keyframes[-1]
        correction = compose_pose_2d(last_kf['optimized_pose'], invert_pose_2d(last_kf['local_pose']))
        return compose_pose_2d(correction, local_pose)

    def _make_pose_msg(self, pose, stamp):
        x, y, theta = pose
        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = self.frame_id
        pose_msg.pose.position.x = float(x)
        pose_msg.pose.position.y = float(y)
        pose_msg.pose.position.z = 0.0
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = math.sin(theta / 2.0)
        pose_msg.pose.orientation.w = math.cos(theta / 2.0)
        return pose_msg

    def _publish_local(self, stamp):
        pose_msg = self._make_pose_msg(self.current_local_pose, stamp)
        self.local_pose_pub.publish(pose_msg)
        self.local_path_msg.header.stamp = stamp
        self._append_local_path_pose(pose_msg)
        self.local_path_pub.publish(self.local_path_msg)
        if self.local_csv_writer is not None:
            row = [stamp.to_sec()] + self.current_local_pose.tolist()
            self.local_csv_writer.writerow(row)
            if self.csv_flush_interval > 0 and (self.frame_count % self.csv_flush_interval) == 0:
                self.local_csv_file.flush()
        if self.tf_broadcaster is not None and self.publish_tf:
            x, y, theta = self.current_local_pose
            self.tf_broadcaster.sendTransform(
                (float(x), float(y), 0.0),
                (0.0, 0.0, math.sin(theta / 2.0), math.cos(theta / 2.0)),
                stamp,
                self.local_child_frame_id,
                self.frame_id,
            )

    def _publish_optimized(self, stamp):
        pose_msg = self._make_pose_msg(self.current_optimized_pose, stamp)
        self.optimized_pose_pub.publish(pose_msg)
        self.optimized_path_msg.header.stamp = stamp
        self.optimized_path_msg.poses = [self._make_pose_msg(kf['optimized_pose'], stamp) for kf in self.keyframes]
        self.optimized_path_pub.publish(self.optimized_path_msg)
        if self.optimized_csv_writer is not None:
            row = [stamp.to_sec()] + self.current_optimized_pose.tolist()
            self.optimized_csv_writer.writerow(row)
            if self.csv_flush_interval > 0 and (self.frame_count % self.csv_flush_interval) == 0:
                self.optimized_csv_file.flush()
        if self.tf_broadcaster is not None and self.publish_optimized_tf:
            x, y, theta = self.current_optimized_pose
            self.tf_broadcaster.sendTransform(
                (float(x), float(y), 0.0),
                (0.0, 0.0, math.sin(theta / 2.0), math.cos(theta / 2.0)),
                stamp,
                self.optimized_child_frame_id,
                self.frame_id,
            )

    def _maybe_add_keyframe(self, stamp, points):
        if self.last_keyframe_local_pose is None:
            self._add_keyframe(stamp, points, is_first=True)
            return

        delta = relative_pose_2d(self.last_keyframe_local_pose, self.current_local_pose)
        trans = float(np.linalg.norm(delta[:2]))
        rot_deg = abs(np.rad2deg(delta[2]))
        if trans >= self.keyframe_translation_thresh or rot_deg >= self.keyframe_rotation_thresh_deg:
            self._add_keyframe(stamp, points, is_first=False)

    def _add_keyframe(self, stamp, points, is_first):
        kf_id = len(self.keyframes)
        descriptor = make_radial_descriptor(points, max_radius=self.max_range, num_bins=self.loop_descriptor_bins)
        optimized_pose = self._initial_optimized_pose(self.current_local_pose)
        keyframe = {
            'id': kf_id,
            'stamp': stamp.to_sec(),
            'points': points.copy(),
            'descriptor': descriptor,
            'local_pose': self.current_local_pose.copy(),
            'optimized_pose': optimized_pose,
        }
        self.keyframes.append(keyframe)

        if not is_first:
            prev = self.keyframes[-2]
            meas = relative_pose_2d(prev['local_pose'], keyframe['local_pose'])
            self.edges.append({
                'i': prev['id'],
                'j': keyframe['id'],
                'measurement': meas,
                'sqrt_info': np.array([self.odom_edge_weight_xy, self.odom_edge_weight_xy, self.odom_edge_weight_theta], dtype=float),
                'type': 'odom',
            })

        self.last_keyframe_local_pose = self.current_local_pose.copy()
        found_loop = False
        if not is_first:
            found_loop = self._try_add_loop_edges_for_latest_keyframe(stamp)

        should_optimize = False
        if len(self.keyframes) >= 2:
            if found_loop:
                should_optimize = True
            elif (not self.optimize_on_loop_only) and (kf_id % max(self.optimize_every_n_keyframes, 1) == 0):
                should_optimize = True

        if should_optimize:
            self._optimize_graph()

        self._update_current_optimized_pose()
        self._write_keyframes_snapshot()
        rospy.loginfo('keyframes=%d, edges=%d, loops=%d', len(self.keyframes), len(self.edges), len(self.loop_pairs))

    def _try_add_loop_edges_for_latest_keyframe(self, stamp):
        latest = self.keyframes[-1]
        candidates = find_loop_candidates(
            current_id=latest['id'],
            current_pose=latest['local_pose'],
            current_descriptor=latest['descriptor'],
            keyframes=self.keyframes[:-1],
            min_index_gap=self.loop_min_index_gap,
            max_pose_distance=self.loop_candidate_distance,
            max_descriptor_distance=self.loop_descriptor_threshold,
            max_candidates=self.loop_max_candidates,
        )

        added = False
        for cand in candidates:
            pair = (cand.keyframe_id, latest['id'])
            if pair in self.loop_pairs:
                continue
            ref = self.keyframes[cand.keyframe_id]
            try:
                _, t_est, theta_est, _, errors = icp_2d(
                    source_points=ref['points'],
                    target_points=latest['points'],
                    max_iterations=self.max_iterations,
                    tolerance=self.tolerance,
                    max_correspondence_distance=self.loop_max_correspondence_distance,
                    trim_percentile=self.trim_percentile,
                    min_correspondences=self.min_correspondences,
                    use_centroid_init=self.use_centroid_init,
                )
            except Exception as exc:
                rospy.logwarn('Loop ICP failed for %s: %s', str(pair), str(exc))
                continue

            if len(errors) == 0:
                continue

            scan_delta = np.array([t_est[0], t_est[1], theta_est], dtype=float)
            motion_measurement = invert_pose_2d(scan_delta) if self.invert_icp_result else scan_delta
            final_error = float(errors[-1])
            if final_error > self.loop_icp_error_threshold:
                continue
            loop_trans = float(np.linalg.norm(motion_measurement[:2]))
            loop_rot_deg = abs(float(np.rad2deg(motion_measurement[2])))
            if self.loop_max_measurement_translation > 0.0 and loop_trans > self.loop_max_measurement_translation:
                rospy.logwarn(
                    'Loop rejected %d <-> %d: translation %.3f m > %.3f m',
                    ref['id'],
                    latest['id'],
                    loop_trans,
                    self.loop_max_measurement_translation,
                )
                continue
            if self.loop_max_measurement_rotation_deg > 0.0 and loop_rot_deg > self.loop_max_measurement_rotation_deg:
                rospy.logwarn(
                    'Loop rejected %d <-> %d: rotation %.2f deg > %.2f deg',
                    ref['id'],
                    latest['id'],
                    loop_rot_deg,
                    self.loop_max_measurement_rotation_deg,
                )
                continue

            self.edges.append({
                'i': ref['id'],
                'j': latest['id'],
                'measurement': motion_measurement,
                'sqrt_info': np.array([self.loop_edge_weight_xy, self.loop_edge_weight_xy, self.loop_edge_weight_theta], dtype=float),
                'type': 'loop',
            })
            self.loop_pairs.add(pair)
            added = True
            if self.loop_csv_writer is not None:
                self.loop_csv_writer.writerow([
                    stamp.to_sec(),
                    ref['id'],
                    latest['id'],
                    float(motion_measurement[0]),
                    float(motion_measurement[1]),
                    float(motion_measurement[2]),
                    final_error,
                ])
                self.loop_csv_file.flush()
            rospy.loginfo(
                'Loop added: %d <-> %d | desc=%.4f, pose_dist=%.4f, icp_error=%.6f',
                ref['id'],
                latest['id'],
                cand.descriptor_distance,
                cand.pose_distance,
                final_error,
            )
        return added

    def _optimize_graph(self):
        initial = np.array([kf['optimized_pose'] for kf in self.keyframes], dtype=float)
        if np.allclose(initial, 0.0):
            initial = np.array([kf['local_pose'] for kf in self.keyframes], dtype=float)
        optimized = optimize_pose_graph(initial, self.edges, max_nfev=self.max_optimizer_nfev)
        for idx, kf in enumerate(self.keyframes):
            kf['optimized_pose'] = optimized[idx]

    def _update_current_optimized_pose(self):
        if not self.keyframes:
            self.current_optimized_pose = self.current_local_pose.copy()
            return
        last_kf = self.keyframes[-1]
        correction = compose_pose_2d(last_kf['optimized_pose'], invert_pose_2d(last_kf['local_pose']))
        self.current_optimized_pose = compose_pose_2d(correction, self.current_local_pose)

    def scan_callback(self, msg):
        if self.log_scan_info_once and not self.logged_scan_info:
            rospy.loginfo(
                'First scan info: frame=%s, angle_min=%.4f, angle_max=%.4f, angle_increment=%.6f, range_min=%.3f, range_max=%.3f, points=%d',
                msg.header.frame_id,
                msg.angle_min,
                msg.angle_max,
                msg.angle_increment,
                msg.range_min,
                msg.range_max,
                len(msg.ranges),
            )
            self.logged_scan_info = True

        points = laser_scan_to_points(
            msg=msg,
            min_range=self.min_range,
            max_range=self.max_range,
            downsample_step=self.downsample_step,
        )
        if points.shape[0] < self.min_points:
            rospy.logwarn_throttle(2.0, 'Too few valid scan points (%d), skip this frame', points.shape[0])
            return

        if self.prev_points is None:
            self.prev_points = points
            self._maybe_add_keyframe(msg.header.stamp, points)
            self._publish_local(msg.header.stamp)
            self._publish_optimized(msg.header.stamp)
            return

        try:
            _, t_est, theta_est, _, errors = icp_2d(
                source_points=self.prev_points,
                target_points=points,
                max_iterations=self.max_iterations,
                tolerance=self.tolerance,
                max_correspondence_distance=self.max_correspondence_distance,
                trim_percentile=self.trim_percentile,
                min_correspondences=self.min_correspondences,
                use_centroid_init=self.use_centroid_init,
            )
        except Exception as exc:
            self._reject_scan(points, 'ICP failed: %s' % str(exc))
            return

        scan_delta = np.array([t_est[0], t_est[1], theta_est], dtype=float)
        motion_delta = invert_pose_2d(scan_delta) if self.invert_icp_result else scan_delta

        usable, reason = self._motion_is_usable(motion_delta, errors)
        if not usable:
            self._reject_scan(points, reason)
            return

        self.current_local_pose = compose_pose_2d(self.current_local_pose, motion_delta)
        self.prev_points = points
        self.frame_count += 1
        self.consecutive_rejects = 0

        self._maybe_add_keyframe(msg.header.stamp, points)
        self._update_current_optimized_pose()

        if len(errors) > 0 and self.frame_count % 10 == 0:
            rospy.loginfo(
                'frame=%d, local_dx=%.4f, local_dy=%.4f, local_dtheta(deg)=%.4f, final_error=%.6f',
                self.frame_count,
                motion_delta[0],
                motion_delta[1],
                np.rad2deg(motion_delta[2]),
                errors[-1],
            )

        self._publish_local(msg.header.stamp)
        self._publish_optimized(msg.header.stamp)


if __name__ == '__main__':
    rospy.init_node('pose_graph_slam_node')
    PoseGraphSLAMNode()
    rospy.spin()
