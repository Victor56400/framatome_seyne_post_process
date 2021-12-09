#!/usr/bin/env python


import message_filters

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

from forssea_utilities.helpers import get_param
from forssea_utilities.ddynamic_reconfigure import DDynamicReconfigure
from forssea_utilities.DynamicParametersServer import DynamicParametersServer

import netifaces as ni
from navcam_vnav.publisher import GPS_PUBLISHER
from navcam_vnav.gpsHandler import GPS_HANDLER, GPS_TESTER, dd2dmlat, dd2dmlon, dm2rad, dm2deg, deg2rad, rad2deg, PosCam_Mark2PosCam_World
from navcam_vnav.cfg import NavcamVnavConfig
from navcam_vnav.conventionHandler import survey_navcam2ros_navcam

from geometry_msgs.msg import TransformStamped, PoseStamped, PointStamped
from forssea_vision_msgs.msg import MarkerEulerArray, MarkerEuler, MarkerArray
from std_msgs.msg import Float32, String
from PyKDL import Rotation, Vector

from data_reader import *


class gpshandlerNode():

    def __init__(self):
        rp.init_node("navcam_vnav_node")
        self.rate = rp.Rate(1)
        self.PATH=get_param("file_path","")

        # ros convention
        self.tf_buffer = tf.Buffer(debug=False)
        self.tf_listener = tf.TransformListener(self.tf_buffer)

        # Find navcam ip address
        self.camera_frame = get_param('camera_frame', 'camera_frame')
        self.timeout = rp.Duration(0)
        
        
        # tf broadcaster
        self.tf_broadcast = tf.TransformBroadcaster()

        # dynamic param server
        self.server = DynamicParametersServer(NavcamVnavConfig)
        self.server.start(self.callback_param)

        self.data = read_dataset()
        perfect_m1 = self.data['marker1_perfect']
        self.ids = [i for i in self.data]

        self.data_corners_full = read_dataset_full_corners()        

        self.hole_data, self.full_hole_data = read_full_data_for_ros()
        self.jaouad_data = read_jaouad_data_for_ros()
        

        
    def work(self):
            for pile in self.:
                #print('name ::: ', name)
                [p,h,x,y,z,qx,qy,qz,qw] = self.jaouad_data[name]
                transfo = tf.TransformStamped()
                transfo.header.frame_id = self.camera_frame
                transfo.child_frame_id = name
                transfo.transform.translation.x = x
                transfo.transform.translation.y = y
                transfo.transform.translation.z = z
                transfo.transform.rotation.x = qx
                transfo.transform.rotation.y = qy
                transfo.transform.rotation.z = qz
                transfo.transform.rotation.w = qw
                self.tf_buffer.set_transform(transfo, 'dataset')
                #self.tf_broadcast.sendTransform(transfo)
 
                tx,ty,tz =0,0,0    
                if 'run2' in name or 'Corner' in name:
                    tx, ty, tz = 0, 0.88, -0.473
                if 'run3' in name or 'run4' in name:
                    tx, ty, tz = 0.1785, 0.88, -0.473
                if 'run5' in name or 'run6' in name:
                    tx, ty, tz = -0.1785, 0.88, -0.473
                #if 'Run_Corners_1' in name or 'Run_Corners_2' in name:
                if True: 
                #print('if 1', name)
                    if p==1 or p==2: tx, ty, tz = 0, 0.88, -0.473
                    if p==3 or p==4: tx, ty, tz = 0.1785, 0.88, -0.473
                    if p==5 or p==6: tx, ty, tz = -0.1785, 0.88, -0.473
                if 'Run_Corners_3' in name or 'Run_Corners_4' in name:
                    if p==1 or p==2: tx, ty, tz = -0.008, 0.89, -0.537 
                    if p==3 or p==4: tx, ty, tz = 0.1705, 0.89, -0.537 
                    if p==5 or p==6: tx, ty, tz = -0.1865, 0.89, -0.537 
                 
            
                transfo_robot = tf.TransformStamped()
                transfo_robot.header.frame_id = name
                transfo_robot.child_frame_id = name+'_tool'
                transfo_robot.transform.translation.x = tx
                transfo_robot.transform.translation.y = ty
                transfo_robot.transform.translation.z = tz
                transfo_robot.transform.rotation.x = 0
                transfo_robot.transform.rotation.y = 0
                transfo_robot.transform.rotation.z = 0
                transfo_robot.transform.rotation.w = 1
                self.tf_buffer.set_transform(transfo_robot, 'dataset')
                #self.tf_broadcast.sendTransform(transfo_robot)
                transfo = self.tf_buffer.lookup_transform(self.camera_frame, name+'_tool', rp.Time(0))
                self.tf_broadcast.sendTransform(transfo)


            # try to find tf to structire
            transfo_struct = tf.TransformStamped()
            transfo_struct.header.frame_id = self.camera_frame
            transfo_struct.child_frame_id = "base_link"
            transfo_struct.transform.translation.x = 0
            transfo_struct.transform.translation.y = 0
            transfo_struct.transform.translation.z = 0
            transfo_struct.transform.rotation.x = 0
            transfo_struct.transform.rotation.y = 0
            transfo_struct.transform.rotation.z = 0
            transfo_struct.transform.rotation.w = 1
            self.tf_buffer.set_transform(transfo_robot, 'dataset')
            #self.tf_broadcast.sendTransform(transfo_robot)
            self.tf_broadcast.sendTransform(transfo_struct)

           

            
            if not self.wrote: 
                for name_o in self.jaouad_data:
                    print(name_o)
                    [p,h_o,x,y,z,qx,qy,qz,qw] = self.jaouad_data[name_o]
                    transfo = self.tf_buffer.lookup_transform(self.camera_frame, name_o+'_tool', rp.Time(0))
                    self.pos['camera->'+name_o] = ['{}'.format(h_o) ,transfo.transform.translation.x, transfo.transform.translation.y, transfo.transform.translation.z]
                write_csv(self.pos, 'out_corners_jouad_tool.csv')


            self.wrote=True
            print('DONE')

       
                    




            


    def callback_param(self, config, level):
	if level > 0:
		self.IP_SURFACE = str(config['IP_SURFACE'])
		self.PROTOCOL_TYPE = config['PROTOCOL_TYPE']

		self.PORT_SURFACE = config['PORT_SURFACE']      
		self.GPS_PUBLISHER.CONNECTED = self.GPS_PUBLISHER.refresh_connection(self.IP_NAVCAM, self.IP_SURFACE, self.PORT_SURFACE, self.PROTOCOL_TYPE)
		print("RECONFIGURE VNAV --- IP SURFACE : {0} ; PROTOCOL : {1} ; PORT SURFACE : {2}".format(self.IP_SURFACE, self.PROTOCOL_TYPE, self.PORT_SURFACE))
	return config

if __name__ == '__main__':
    node = gpshandlerNode()
    node.work()
