#!/usr/bin/env python3


"""
Post process for the version of the operation we had in septembre
"""
from copy import deepcopy
import random
import os
import rospy as rp
from math import cos, sin, asin, atan2, sqrt, pi, log, pow, tan, atan, exp
from geometry_msgs.msg import Quaternion, Vector3, PointStamped
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros as tf


from data_reader import DataReader
from acquisition import Acquisition, quaternion_from_euler

import yaml


class FramatomeProcess():

    def __init__(self):

        # node setup
        rp.init_node('Framatome_process_node')
        self.rate = rp.Rate(1)
        self.now = rp.Time.now()
        self.root_dir = '/home/victor/Downloads/framatome'
        self.data_hierarchie = ['set', 'struct', 'plaque', 'pose']

        # buffer setup
        Acquisition.global_tf_buffer = tf.Buffer(debug=False)
        Acquisition.now = self.now
        self.tf_broadcaster = tf.StaticTransformBroadcaster()

        # reader all data
        self.reader = DataReader(
            self.root_dir, self.data_hierarchie, plotting=False, now=self.now)
        print(f'We have {self.reader.get_n_valid()} correct acquisitions')

        # pub for vizualisation
        self.points_pub = rp.Publisher(
            '/robot/marker/points', PointStamped, queue_size=14)
        self.markers_pub = rp.Publisher(
            '/robot/marker/markers', MarkerArray, queue_size=6)

        # read raw measured points on robot
        with open(os.path.join(self.root_dir, 'transforms_metadata.yaml')) as handle:
            self.raw_points_data = yaml.safe_load(handle)

    def get_random_acquisition(self) -> Acquisition:
        return random.choice(self.reader.dataset_serial)

    def publish_points(self):
        """
            Publish points for the corners of markers and the tools position
        """
        for name, marker in self.raw_points_data['markers'].items():
            id = marker['id']
            for n_corner, corner in marker['corners'].items():
                p = PointStamped()
                p.header.frame_id = 'robot_frame'
                p.header.stamp = self.now
                p.point.x = corner['x']/1000
                p.point.y = corner['y']/1000
                p.point.z = corner['z']/1000
                self.points_pub.publish(p)
        for tool in self.raw_points_data['tools'].values():
            p = PointStamped()
            p.header.frame_id = 'robot_frame'
            p.header.stamp = self.now
            p.point.x = tool['x']/1000
            p.point.y = tool['y']/1000
            p.point.z = tool['z']/1000
            self.points_pub.publish(p)

    def publish_markers(self):
        """
            Publish marker for rviz
        """
        markers = MarkerArray()
        i = 0
        for name, marker in self.raw_points_data['markers'].items():
            id = marker['id']

            tf_robot_marker: tf.TransformStamped = Acquisition.global_tf_buffer.lookup_transform(
                'robot_frame', f'marker_{name}_frame', Acquisition.now)

            marker = Marker()
            marker.id = i
            i += 1
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.header.stamp = self.now
            marker.header.frame_id = 'robot_frame'
            marker.pose.position = tf_robot_marker.transform.translation
            marker.pose.orientation = tf_robot_marker.transform.rotation
            marker.lifetime = rp.Duration(1)
            marker.color.a = 1.0
            marker.color.r = 133/255
            marker.color.g = 4/255
            marker.color.b = 43/255
            marker.scale = Vector3(
                x=self.reader.marker_size, z=self.reader.marker_size, y=0.01)

            marker_text = deepcopy(marker)
            marker_text.id = i
            i += 1
            marker_text.text = name
            marker_text.type = Marker.TEXT_VIEW_FACING
            marker_text.color.r = 1
            marker_text.color.g = 1
            marker_text.color.b = 1

            markers.markers.append(marker)
            markers.markers.append(marker_text)

        self.markers_pub.publish(markers)

    def work(self):

        acq = process.get_random_acquisition()
        # acq = process.reader.dataset_serial[10]
        print(acq)
        acq.compute_all_metadata()
        acq_transforms = acq.get_random_transform_set()
        while not rp.is_shutdown():
            self.publish_points()
            self.publish_markers()
            for acq_tf in acq_transforms:
                self.tf_broadcaster.sendTransform(acq_tf)
            self.rate.sleep()


if __name__ == '__main__':
    process = FramatomeProcess()

    # process.reader.compute_all_base_frames()
    # outdata = process.reader.compute_all_pose_in_base_frame()

    process.reader.plot_all_data()

    # outdata = process.reader.compute_metadata_out()

    # process.work()

    # outdata.to_csv('/home/victor/Downloads/framatome/output_transformed_data.csv')

# TODO : test ot see is transform are correct by plotting points and tf
