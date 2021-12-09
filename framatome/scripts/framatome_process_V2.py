#!/usr/bin/env python


"""
Post process for the version of the operation we had in june 
"""
import message_filters
import numpy as np
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
        print(self.data)
        self.structure_data = read_structure_file_V2()
        self.pile2marker = read_pile_marker()
        print(self.pile2marker)
        self.mesure2piles = read_mesure_pile()
        print(self.mesure2piles)
        
        self.n_mesure = get_param('/eiffage_brest/n_mesure  ', 4)
        self.mesure_names = ['mesure_'+str(i+1) for i in range(self.n_mesure)] 

        self.camera_frame = 'camera_frame'
        self.M0 = "Marker_" + str(get_param('/eiffage_brest/reference_marker', 7))
        self.Mn = "M_n" 
        self.Mn1="M_n_1"
        self.R0 = "R0"

        self.tf_buffer_final = tf.Buffer(debug=False)


    def work_once(self):
        now = rp.Time.now()     
        for mesure in self.mesure2piles:
            print('---')
            print('Processing {}'.format(mesure))
            print('Pile depart : {} ; Pile cible : {}'.format(self.mesure2piles[mesure][0], self.mesure2piles[mesure][1]))
            pile_depart = self.mesure2piles[mesure][0]
            pile_arrive = self.mesure2piles[mesure][1]
            echantillons = self.data[mesure]
            #print(markers_data)

            [L,h,ln,qx,qy,qz,qw] = self.structure_data['Pile_M0']
            tf_R0 = tf.TransformStamped()
            tf_R0.header.frame_id = self.R0
            tf_R0.header.stamp = now
            tf_R0.child_frame_id = 'pile_M0'
            tf_R0.transform.translation.x = L
            tf_R0.transform.translation.y = ln
            tf_R0.transform.translation.z = h
            #tf_R0.transform.translation.x = 0
            #tf_R0.transform.translation.y = 0
            #tf_R0.transform.translation.z = 0

            tf_R0.transform.rotation.x = 0
            tf_R0.transform.rotation.y = 0
            tf_R0.transform.rotation.z = 0
            tf_R0.transform.rotation.w = 1
            self.tf_buffer.set_transform(tf_R0, 'structure data')
            self.tf_buffer_final.set_transform(tf_R0, 'structure data')
            self.tf_broadcaster.sendTransform(tf_R0)
            #print(tf_R0)

            markers = set()
            for echantillon, echantillon_data in echantillons.items():
                for marker, marker_data in echantillon_data.items():
                    markers.add(marker)
                    if marker == self.pile2marker[pile_depart]: marker_name = 'marker_depart_'+  echantillon 
                    if marker == self.pile2marker[pile_arrive]: marker_name = 'marker_arrive_'+  echantillon 
                    [x,y,z,qx,qy,qz,qw] = marker_data
                    tf_M_n = tf.TransformStamped()
                    tf_M_n.header.frame_id = self.camera_frame + '_' + echantillon 
                    tf_M_n.header.stamp = now
                    tf_M_n.child_frame_id = marker_name 
                    tf_M_n.transform.translation.x = x
                    tf_M_n.transform.translation.y = y
                    tf_M_n.transform.translation.z = z
                    tf_M_n.transform.rotation.x = qx
                    tf_M_n.transform.rotation.y = qy
                    tf_M_n.transform.rotation.z = qz
                    tf_M_n.transform.rotation.w = qw
                    self.tf_buffer.set_transform(tf_M_n, 'recording data')

            markers = list(markers)
            echantillon_tf = []
            for echantillon, echantillon_data in echantillons.items():
                tf_pile = self.tf_buffer.lookup_transform('marker_depart_' + echantillon ,'marker_arrive_'+ echantillon,now)
                x = tf_pile.transform.translation.x
                y = tf_pile.transform.translation.y
                z = tf_pile.transform.translation.z
                qx = tf_pile.transform.rotation.x
                qy = tf_pile.transform.rotation.y
                qz = tf_pile.transform.rotation.z
                qw = tf_pile.transform.rotation.w
                echantillon_tf.append([mesure, echantillon,x,y,z,qx,qy,qz,qw])

            
            echantillon_tf = pd.DataFrame(echantillon_tf, columns = ['mesure','echantillon','x','y','z','qx','qy','qz','qw'])
            x = echantillon_tf['x'].median()
            y = echantillon_tf['y'].median()
            z = echantillon_tf['z'].median()
            #x = echantillon_tf['x'].mean()
            #y = echantillon_tf['y'].mean()
            #z = echantillon_tf['z'].mean()


            Q = np.array(echantillon_tf)[:,5:].astype(float)
            W = [1 for i in range(len(Q))]
            q = avg_q(Q,W) 
    
            tf_mesure = tf.TransformStamped()
            tf_mesure.header.frame_id = pile_depart
            tf_mesure.header.stamp = now
            tf_mesure.child_frame_id = pile_arrive
            tf_mesure.transform.translation.x = x
            tf_mesure.transform.translation.y = y
            tf_mesure.transform.translation.z = z
            tf_mesure.transform.rotation.x = q[0]
            tf_mesure.transform.rotation.y = q[1]
            tf_mesure.transform.rotation.z = q[2]
            tf_mesure.transform.rotation.w = q[3] 
            self.tf_buffer_final.set_transform(tf_mesure, 'structure data')
            self.tf_broadcaster.sendTransform(tf_mesure)
          
             
            tf_final = self.tf_buffer_final.lookup_transform(self.R0, pile_arrive,now)
            frame = tf_final.child_frame_id
            frame_origin = tf_final.header.frame_id
            x = tf_final.transform.translation.x
            y = tf_final.transform.translation.y
            z = tf_final.transform.translation.z
            qx = tf_final.transform.rotation.x
            qy = tf_final.transform.rotation.y
            qz = tf_final.transform.rotation.z
            qw = tf_final.transform.rotation.w

            r,p,h = old_tf.transformations.euler_from_quaternion([qx,qy,qz,qw], 'sxyz') 
            r = rad2deg(r)
            p = rad2deg(p)
            h = rad2deg(h)
            #print(frame, round(x,4),round(y,4),round(z,4), round(r,4),round(p,4),round(h,4)) 
            print('{},{},{},{}'.format(frame, round(x,4),round(y,4),round(z,4))) 

            tf_final = self.tf_buffer_final.lookup_transform('pile_M0', pile_arrive,now)
            frame = tf_final.child_frame_id
            x = tf_final.transform.translation.x
            y = tf_final.transform.translation.y
            z = tf_final.transform.translation.z
            qx = tf_final.transform.rotation.x
            qy = tf_final.transform.rotation.y
            qz = tf_final.transform.rotation.z
            qw = tf_final.transform.rotation.w

            r,p,h = old_tf.transformations.euler_from_quaternion([qx,qy,qz,qw], 'sxyz') 
            r = rad2deg(r)
            p = rad2deg(p)
            h = rad2deg(h)

            #print(frame, round(x,4),round(y,4),round(z,4), round(r,4),round(p,4),round(h,4)) 

 

                ##print(tf_M_n)
                ##the_tf = self.tf_buffer.lookup_transform(self.R0, self.M_n, now)
                #self.tf_broadcaster.sendTransform(tf_M_n)
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
