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
from slamware_loop_slam.pose import compose_pose_2d, invert_delta_pose_2d
from slamware_loop_slam.scan_to_points import laser_scan_to_points


def default_result_path(filename):
    package_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
    return os.path.join(package_dir, 'result', filename)


class ScanICPNode(object):
    def __init__(self):
        self.scan_topic = rospy.get_param('~scan_topic', '/scan')
        self.pose_topic = rospy.get_param('~pose_topic', '/slamware_icp_pose')
        self.path_topic = rospy.get_param('~path_topic', '/slamware_icp_path')
        self.frame_id = rospy.get_param('~frame_id', 'map')
        self.child_frame_id = rospy.get_param('~child_frame_id', 'slamware_icp_base_link')
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
        self.save_csv = bool(rospy.get_param('~save_csv', True))
        self.csv_path = rospy.get_param('~csv_path', default_result_path('no_loop_path.csv'))
        self.log_scan_info_once = bool(rospy.get_param('~log_scan_info_once', True))
        self.path_publish_step = int(rospy.get_param('~path_publish_step', 1))
        self.max_path_poses = int(rospy.get_param('~max_path_poses', 3000))
        self.csv_flush_interval = int(rospy.get_param('~csv_flush_interval', 20))
        self.min_correspondences = max(self.min_correspondences, 2)
        self.max_consecutive_rejects = max(self.max_consecutive_rejects, 1)
        self.path_publish_step = max(self.path_publish_step, 1)

        self.prev_points = None
        self.current_pose = np.array([0.0, 0.0, 0.0], dtype=float)
        self.frame_count = 0
        self.path_sample_count = 0
        self.consecutive_rejects = 0
        self.logged_scan_info = False

        self.pose_pub = rospy.Publisher(self.pose_topic, PoseStamped, queue_size=10)
        self.path_pub = rospy.Publisher(self.path_topic, Path, queue_size=10)
        self.tf_broadcaster = tf.TransformBroadcaster() if self.publish_tf else None

        self.path_msg = Path()
        self.path_msg.header.frame_id = self.frame_id

        self.csv_file = None
        self.csv_writer = None
        if self.save_csv:
            csv_dir = os.path.dirname(self.csv_path)
            if csv_dir and not os.path.exists(csv_dir):
                os.makedirs(csv_dir)
            self.csv_file = open(self.csv_path, 'wb')
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(['stamp', 'x', 'y', 'theta_rad'])
            rospy.loginfo('Saving no-loop trajectory to %s', self.csv_path)

        self.scan_sub = rospy.Subscriber(self.scan_topic, LaserScan, self.scan_callback, queue_size=1)
        rospy.on_shutdown(self.on_shutdown)
        rospy.loginfo('scan_icp_node started on %s', self.scan_topic)

    def on_shutdown(self):
        if self.csv_file is not None:
            self.csv_file.flush()
            self.csv_file.close()

    def _append_path_pose(self, pose_msg):
        self.path_sample_count += 1
        step = max(self.path_publish_step, 1)
        if step > 1 and (self.path_sample_count % step) != 0:
            return
        self.path_msg.poses.append(pose_msg)
        if self.max_path_poses > 0 and len(self.path_msg.poses) > self.max_path_poses:
            overflow = len(self.path_msg.poses) - self.max_path_poses
            del self.path_msg.poses[:overflow]

    def _reject_scan(self, points, reason):
        self.consecutive_rejects += 1
        rospy.logwarn_throttle(1.0, '%s; rejected=%d/%d',
                               reason, self.consecutive_rejects, self.max_consecutive_rejects)
        if self.consecutive_rejects >= self.max_consecutive_rejects:
            self.prev_points = points
            self.consecutive_rejects = 0
            rospy.logwarn('Reset ICP reference scan after repeated rejected frames.')

    def _motion_is_usable(self, delta_est, errors):
        if len(errors) == 0:
            return False, 'ICP returned no error history'

        final_error = float(errors[-1])
        trans = float(np.linalg.norm(delta_est[:2]))
        rot_deg = abs(float(np.rad2deg(delta_est[2])))

        if final_error > self.max_icp_error:
            return False, 'ICP error %.4f > %.4f' % (final_error, self.max_icp_error)
        if trans > self.max_step_translation:
            return False, 'ICP translation jump %.4f m > %.4f m' % (trans, self.max_step_translation)
        if rot_deg > self.max_step_rotation_deg:
            return False, 'ICP rotation jump %.2f deg > %.2f deg' % (rot_deg, self.max_step_rotation_deg)
        return True, ''

    def publish_pose_and_path(self, stamp):
        x, y, theta = self.current_pose

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

        self.pose_pub.publish(pose_msg)
        self.path_msg.header.stamp = stamp
        self._append_path_pose(pose_msg)
        self.path_pub.publish(self.path_msg)

        if self.tf_broadcaster is not None:
            self.tf_broadcaster.sendTransform(
                (float(x), float(y), 0.0),
                (0.0, 0.0, math.sin(theta / 2.0), math.cos(theta / 2.0)),
                stamp,
                self.child_frame_id,
                self.frame_id,
            )

        if self.csv_writer is not None:
            self.csv_writer.writerow([stamp.to_sec(), float(x), float(y), float(theta)])
            if self.csv_flush_interval > 0 and (self.frame_count % self.csv_flush_interval) == 0:
                self.csv_file.flush()

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
            self.publish_pose_and_path(msg.header.stamp)
            rospy.loginfo('Received first usable scan')
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

        delta_est = np.array([t_est[0], t_est[1], theta_est], dtype=float)
        if self.invert_icp_result:
            delta_est = invert_delta_pose_2d(delta_est)

        usable, reason = self._motion_is_usable(delta_est, errors)
        if not usable:
            self._reject_scan(points, reason)
            return

        self.current_pose = compose_pose_2d(self.current_pose, delta_est)
        self.prev_points = points
        self.frame_count += 1
        self.consecutive_rejects = 0

        if len(errors) > 0 and self.frame_count % 10 == 0:
            rospy.loginfo(
                'frame=%d, robot_dx=%.4f, robot_dy=%.4f, robot_dtheta(deg)=%.4f, final_error=%.6f',
                self.frame_count,
                delta_est[0],
                delta_est[1],
                np.rad2deg(delta_est[2]),
                errors[-1],
            )

        self.publish_pose_and_path(msg.header.stamp)


if __name__ == '__main__':
    rospy.init_node('scan_icp_node')
    ScanICPNode()
    rospy.spin()
