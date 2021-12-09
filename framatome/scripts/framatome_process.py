#!/usr/bin/env python


import os
import rospy as rp
import csv
import itertools  
import pynmea2
from math import cos, sin, asin, atan2, sqrt, pi, log, pow, tan, atan, exp
import datetime
import tf2_ros as tf
import tf2_geometry_msgs
import tf as old_tf


from PyKDL import Rotation, Vector
from forssea_utilities.helpers import get_param

from data_reader import *


class eiffageProcess():

    def __init__(self):
        rp.init_node('Eiffage_process_node')      
        self.rate = rp.Rate(1)

        self.tf_buffer = tf.Buffer(debug=False)
        self.tf_broadcaster = tf.StaticTransformBroadcaster()

        #self.data = read_dataset()
        self.data = read_dataset_jaouad()
        self.structure_data = read_structure_file()
 
        self.camera_frame = 'camera_frame'
        self.M0 = "Marker_" + str(get_param('/eiffage_brest/reference_marker', 58))
        self.M_n = "M_n" 
        self.R0 = "R0"

    def work_once(self):
        for pile in self.data:
            print('---')
            print('Processing {}'.format(pile))
            now = rp.Time.now()           

 
            marker_data = self.data[pile]
            structure_data = self.structure_data[pile]
 
            marker_reference = marker_data[self.M0]
            del marker_data[self.M0] 
            marker_pile = marker_data[marker_data.keys()[0]]
            marker_data[self.M0] = marker_reference               

            self.M_n = "Marker_"+pile            
            self.M0_name = "Reference_marker_"+pile
 
            [x,y,z,qx,qy,qz,qw] = structure_data 
            tf_R0 = tf.TransformStamped()
            tf_R0.header.frame_id = self.M0_name
            tf_R0.header.stamp = now
            tf_R0.child_frame_id = self.R0
            tf_R0.transform.translation.x = x
            tf_R0.transform.translation.y = y
            tf_R0.transform.translation.z = z
            tf_R0.transform.rotation.x = qx
            tf_R0.transform.rotation.y = qy
            tf_R0.transform.rotation.z = qz
            tf_R0.transform.rotation.w = qw
            self.tf_buffer.set_transform(tf_R0, 'structure data')

            [x,y,z,qx,qy,qz,qw] = marker_reference
            tf_M0 = tf.TransformStamped()
            tf_M0.header.frame_id = self.camera_frame
            tf_M0.header.stamp = now
            tf_M0.child_frame_id = self.M0_name
            tf_M0.transform.translation.x = x
            tf_M0.transform.translation.y = y
            tf_M0.transform.translation.z = z
            tf_M0.transform.rotation.x = qx
            tf_M0.transform.rotation.y = qy
            tf_M0.transform.rotation.z = qz
            tf_M0.transform.rotation.w = qw
            self.tf_buffer.set_transform(tf_M0, 'recording data')
            
            [x,y,z,qx,qy,qz,qw] = marker_pile
            tf_M_n = tf.TransformStamped()
            tf_M_n.header.frame_id = self.camera_frame
            tf_M_n.header.stamp = now
            tf_M_n.child_frame_id = self.M_n
            tf_M_n.transform.translation.x = x
            tf_M_n.transform.translation.y = y
            tf_M_n.transform.translation.z = z
            tf_M_n.transform.rotation.x = qx
            tf_M_n.transform.rotation.y = qy
            tf_M_n.transform.rotation.z = qz
            tf_M_n.transform.rotation.w = qw
            self.tf_buffer.set_transform(tf_M_n, 'recording data')

            the_tf = self.tf_buffer.lookup_transform(self.R0, self.M_n, now)
            self.tf_broadcaster.sendTransform(tf_R0)
            self.tf_broadcaster.sendTransform(tf_M0)
            self.tf_broadcaster.sendTransform(tf_M_n)
            #print(self.tf_buffer.all_frames_as_yaml())            
 
        print(' --- ')
        print(' DONE')        
        print(' --- ')
 
    def work(self):
        while not rp.is_shutdown():
            self.work_once()     
            self.rate.sleep()
            



if __name__ == '__main__':
    process = eiffageProcess()
    process.work()
