import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from sklearn.mixture import GaussianMixture
from sklearn.datasets import make_classification
from sklearn.cluster import KMeans
from scipy.spatial.transform import Rotation
import pandas as pd
import numpy as np
from functools import total_ordering
from math import cos, sin, pi
from transforms3d.quaternions import mat2quat
from transforms3d.euler import euler2quat, quat2euler, quat2mat, _AXES2TUPLE
import matplotlib.patches as mpatches
from mpl_toolkits.mplot3d import Axes3D
from typing import Collection, Dict, Iterable
import os
from glob import glob
from collections import Counter
import random
from geometry_msgs.msg import Quaternion, Vector3, PointStamped, TransformStamped
import tf2_geometry_msgs
import tf as old_tf
import tf2_ros as tf
import cv2

import warnings
warnings.simplefilter(action='ignore', category=Warning)


def deg2rad(deg: float) -> float:
    """
        Converts degrees to radians
    """
    return pi*deg/180


def rad2deg(rad: float) -> float:
    """
        Converts radians to degrees
    """
    return 180*rad/pi


def is_axes_valid(axes: str) -> bool:
    """
        Check if the given axes are valid
    """
    return axes in _AXES2TUPLE


def quaternion_from_euler(e: Iterable, axes: str) -> Iterable:
    """
        takes euler angles with provided convention as input and return quaternion
    """
    if not is_axes_valid(axes):
        raise ValueError(f'Incorrect axes string {axes} was given')
    q = euler2quat(ai=e[0], aj=e[1], ak=e[2],
                   axes=axes)  # get the quaternion of the rotation
    return (q[1], q[2], q[3], q[0])


def avg_q(q: Iterable[Iterable[float]], w: Iterable[float]) -> np.array:
    '''
    Averaging Quaternions.

    Arguments:
        Q(ndarray): an Mx4 ndarray of quaternions.
        weights(list): an M elements list, a weight for each quaternion.

    TODO : make this function with geometry msg quaternion in and out
    '''

    return np.linalg.eigh(np.einsum('ij,ik,i->...jk', q, q, w))[1][:, -1]


class Acquisition:
    """
        An object of this class represent of data acquisition on a single point
    """
    # this variable represent to metadata that is computed
    output_metadata = {'set': 'ok', 'structure': 'ok', 'plaque': 'ok', 'hole': 'ok',  # position
                       'night': 'ok', 'tarp': 'ok', 'marker': 'ok', 'backlit': 'ok', 'screw': 'ok',  # metadata in
                       'n_trusted_mesure': 'ok',
                       'x_mean': 'ok', 'y_mean': 'ok', 'z_mean': 'ok', 'roll_mean': 'ok', 'pitch_mean': 'ok', 'yaw_mean': 'ok',  # mean
                       'x_median': 'ok', 'y_median': 'ok', 'z_median': 'ok', 'roll_median': 'ok', 'pitch_median': 'ok', 'yaw_median': 'ok',  # median
                       'x_std': 'ok', 'y_std': 'ok', 'z_std': 'ok', 'roll_std': 'ok', 'pitch_std': 'ok', 'yaw_std': 'ok',  # std
                       "x_mean_clustered": 'ok', "y_mean_clustered": 'ok', "z_mean_clustered": 'ok',
                       "x_mean_clustered_trusted": 'ok', "y_mean_clustered_trusted": 'ok', "z_mean_clustered_trusted": 'ok',
                       "x_std_clustered": 'ok', "y_std_clustered": 'ok', "z_std_clustered": 'ok',
                       "x_std_clustered_trusted": 'ok', "y_std_clustered_trusted": 'ok', "z_std_clustered_trusted": 'ok',
                       "roll_mean_clustered": 'ok', "pitch_mean_clustered": 'ok', "yaw_mean_clustered": 'ok',
                       "roll_mean_clustered_trusted": 'ok', "pitch_mean_clustered_trusted": 'ok', "yaw_mean_clustered_trusted": 'ok',
                       "roll_std_clustered": 'ok', "pitch_std_clustered": 'ok', "yaw_std_clustered": 'ok',
                       "roll_std_clustered_trusted": 'ok', "pitch_std_clustered_trusted": 'ok', "yaw_std_clustered_trusted": 'ok',
                       'transformed_x_mean': 'ok', 'transformed_y_mean': 'ok', 'transformed_z_mean': 'ok',
                       'transformed_x_median': 'ok', 'transformed_y_median': 'ok', 'transformed_z_median': 'ok',
                       'transformed_x_std': 'ok', 'transformed_y_std': 'ok', 'transformed_z_std': 'ok',
                       "transformed_x_mean_clustered": 'ok', "transformed_y_mean_clustered": 'ok', "transformed_z_mean_clustered": 'ok',
                       "transformed_x_mean_clustered_trusted": 'ok', "transformed_y_mean_clustered_trusted": 'ok', "transformed_z_mean_clustered_trusted": 'ok',
                       "transformed_x_std_clustered": 'ok', "transformed_y_std_clustered": 'ok', "transformed_z_std_clustered": 'ok',
                       "transformed_x_std_clustered_trusted": 'ok', "transformed_y_std_clustered_trusted": 'ok', "transformed_z_std_clustered_trusted": 'ok'
                       }

    # this represents the metadat of the acquisition
    metadata = None
    # this represents the transforms between tool, marker and robot
    base_transforms = None
    robot_data = None
    trust_threshold = 5

    # tf buffer commun to ALL acquisition
    global_tf_buffer: tf.Buffer = None
    now = None
    n_set = 0
    n_structure = 0
    n_plaque = 3
    n_hole = 20

    base_frames_tool = dict()
    base_frames_marker = dict()
    base_acquisition = dict()

    def __init__(self, path: str) -> None:
        self.path = path
        self.trust_threshold = Acquisition.trust_threshold

        self.set = 0
        self.structure = 0
        self.plaque = 0
        self.hole = 0

        self.n_time_called = 0
        # use the path to know where i am
        self.parse_path(path)
        # use the metadata to have additional info
        self.parse_metadata()

        self.local_avg_tool_frame = self.get_avg_hole_frame()
        self.local_avg_marker_frame = self.get_avg_marker_frame()

        self.data = pd.read_csv(path)
        self.valid = not(self.data.empty)

        self.max_trust_factor = self.data[' Trust Factor'].max()
        self.valid = self.max_trust_factor >= self.trust_threshold

        self.local_tf_buffer = tf.Buffer()

    def compute_all_metadata(self):
        """
            Compute all metadata, the order of computation is important
        """
        self.compute_trusted_data()
        self.get_mean_point()
        self.get_median_point()
        self.get_rotation_mean()
        self.get_rotation_median()
        self.get_std()
        self.transform_measures()
        self.get_clustering()

    def compute_trusted_data(self):
        """
            compute the data with a high enough trust factor
        """
        self.trusted_data = self.data[self.data[' Trust Factor']
                                      >= Acquisition.trust_threshold]
        self.n_trusted_mesure = len(self.trusted_data)
        if self.n_trusted_mesure == 0:
            self.trusted_data = self.data

    def parse_path(self, path: str) -> None:
        """
            parse the path to know what set,structure,plaque and hole it is
        """
        splited = path.split('/')
        for word in splited:
            if 'set' in word:
                self.set = int(word.replace('set', ''))
                if self.set > Acquisition.n_set:
                    Acquisition.n_set = self.set
            if 'struct' in word:
                self.structure = int(word.replace('struct', ''))
                if self.structure > Acquisition.n_structure:
                    Acquisition.n_structure = self.structure
            if 'plaque' in word:
                self.plaque = int(word.replace('plaque', ''))
            if 'pose' in word:
                self.hole = int(word.replace('pose', ''))

    def parse_metadata(self):
        """
            Use the metadata file to get the data for the acquisition
        """
        data = Acquisition.metadata[(Acquisition.metadata['set'] == self.set) &
                                    (Acquisition.metadata['structure'] == self.structure) &
                                    (Acquisition.metadata['plaque'] == self.plaque)]
        self.night = data.at[data.index[0], 'night'] == 1
        self.marker = data.at[data.index[0], 'marker']
        self.tarp = data.at[data.index[0], 'tarp'] == 1
        self.backlit = data.at[data.index[0], 'backlit'] == 1
        screw_holes = list(
            map(int, data.at[data.index[0], 'screw'].split(',')))
        self.screw_hole = self.hole in screw_holes
        self.robot_side = Acquisition.robot_data['structure'][
            self.structure]['plaque'][self.plaque]['robot_side']
        self.tool_side = Acquisition.robot_data['structure'][self.structure]['plaque'][self.plaque]['tool_side']
        self.tool_transform = Acquisition.base_transforms['tools'][self.tool_side]
        self.marker_transform = Acquisition.base_transforms['markers'][self.robot_side]
        self.total_transform = self.tool_transform @ self.marker_transform

    def __repr__(self) -> str:
        return f"Acquisition(from set {self.set}, structure {self.structure}, plaque {self.plaque}, hole {self.hole})"

    def __repr_metadata__(self) -> str:
        return f"Acquisition {'at night' if self.night else 'during day'} of{' backlit' if self.backlit else ''} marker {self.marker} with{'out' if self.tarp else ''} tarp. {'The screw may have had an impact' if self.screw_hole else ''}"

    def get_mean_point(self):
        """
            return the mean point of the data
        """
        self.x_mean = self.trusted_data[' X'].mean()
        self.y_mean = self.trusted_data[' Y'].mean()
        self.z_mean = self.trusted_data[' Z'].mean()
        self.roll_mean = self.trusted_data[' Roll'].mean()
        self.pitch_mean = self.trusted_data[' Pitch'].mean()
        self.yaw_mean = self.trusted_data[' Yaw'].mean()

    def get_median_point(self):
        """
            return the mean point of the data
        """
        self.x_median = self.trusted_data[' X'].median()
        self.y_median = self.trusted_data[' Y'].median()
        self.z_median = self.trusted_data[' Z'].median()
        self.roll_median = self.trusted_data[' Roll'].median()
        self.pitch_median = self.trusted_data[' Pitch'].median()
        self.yaw_median = self.trusted_data[' Yaw'].median()

    def get_std(self):
        """
            return the std of the data (point, orientation) for a given trust_threshold
        """
        data = self.trusted_data

        self.x_std = data[' X'].std()
        self.y_std = data[' Y'].std()
        self.z_std = data[' Z'].std()
        self.roll_std = data[' Roll'].std()
        self.pitch_std = data[' Pitch'].std()
        self.yaw_std = data[' Yaw'].std()

    def get_std_more_than_thresh(self, trust_threshold):
        """
            return the std of the data (point, orientation) for a given trusted_threshold
        """
        data = self.data[self.data[' Trust Factor'] >= trust_threshold]

        std_x = data[' X'].std()
        std_y = data[' Y'].std()
        std_z = data[' Z'].std()
        std_roll = data[' Roll'].std()
        std_pitch = data[' Pitch'].std()
        std_yaw = data[' Yaw'].std()
        return (std_x, std_y, std_z, std_roll, std_pitch, std_yaw)

    def get_rotation_mean(self):
        """
            Get rotation information using mean rpy
        """
        self.q_mean = quaternion_from_euler(
            (self.pitch_mean, self.roll_mean, self.yaw_mean), 'ryxz')
        try:
            rotation = Rotation.from_quat(self.q_mean)
            self.axes_mean = rotation.as_rotvec()
            self.angle_mean = rotation.magnitude()
        except:
            pass

    def get_rotation_median(self):
        """
            Get rotation information using median rpy
        """
        self.q_median = quaternion_from_euler(
            (self.pitch_median, self.roll_mean, self.yaw_mean), 'ryxz')
        try:
            rotation = Rotation.from_quat(self.q_median)
            self.axes_median = rotation.as_rotvec()
            self.angle_median = rotation.magnitude()
        except:
            pass

    def get_clustering(self):
        """
            Compute all clustering related info, must be done alfter all other metadata computation
        """
        self.compute_trusted_data()
        data = self.data

        clustering = GaussianMixture(2)
        x = [[data[' Roll'].loc[i], data[' Pitch'].loc[i],
              data[' Yaw'].loc[i]] for i in data.index]

        clustering.fit(x)
        self.identified_clusters = clustering.fit_predict(x)
        n_cluster = Counter(self.identified_clusters)
        self.major_cluster = 0 if n_cluster[0] > n_cluster[1] else 1

        keys = (' X', ' Y', ' Z', ' Roll', ' Pitch', ' Yaw')

        # compute metadata for position and angles
        for key in keys:
            name_mean = key[1:].lower() + "_mean_clustered"
            name_mean_trusted = key[1:].lower() + "_mean_clustered_trusted"
            name_std = key[1:].lower() + "_std_clustered"
            name_std_trusted = key[1:].lower() + "_std_clustered_trusted"

            self.__dict__[
                name_mean] = data[key][self.identified_clusters == self.major_cluster].mean()
            self.__dict__[
                name_mean_trusted] = data[self.identified_clusters == self.major_cluster][data[' Trust Factor'] == 5][key].mean()
            self.__dict__[
                name_std] = data[key][self.identified_clusters == self.major_cluster].std()
            self.__dict__[
                name_std_trusted] = data[self.identified_clusters == self.major_cluster][data[' Trust Factor'] == 5][key].std()
        self.dict_std = {' Roll': {'total': self.roll_std, 'clustered': self.roll_std_clustered, 'clustered_trusted': self.roll_std_clustered_trusted},
                         ' Pitch': {'total': self.pitch_std, 'clustered': self.pitch_std_clustered, 'clustered_trusted': self.pitch_std_clustered_trusted},
                         ' Yaw': {'total': self.yaw_std, 'clustered': self.yaw_std_clustered, 'clustered_trusted': self.yaw_std_clustered_trusted}}

        # compute metadata for transformed positions
        keys = 'xyz'
        for key in keys:
            name_mean = f'transformed_{key}_mean_clustered'
            name_mean_trusted = f'transformed_{key}_mean_clustered_trusted'
            name_std = f'transformed_{key}_std_clustered'
            name_std_trusted = f'transformed_{key}_std_clustered_trusted'
            self.__dict__[
                name_mean] = self.transformed_data[key][self.identified_clusters == self.major_cluster].mean()
            self.__dict__[
                name_mean_trusted] = self.transformed_data[self.identified_clusters == self.major_cluster][data[' Trust Factor'] == 5][key].mean()
            self.__dict__[
                name_std] = self.transformed_data[key][self.identified_clusters == self.major_cluster].std()
            self.__dict__[
                name_std_trusted] = self.transformed_data[self.identified_clusters == self.major_cluster][data[' Trust Factor'] == 5][key].std()

    def plot_angles_with_cluster(self):
        """
            plot graphs relevant to angle measures
        """
        self.get_clustering()
        data = self.data
        # data = self.data

        # clustering = GaussianMixture(2)
        # x = [[data[' Roll'].iloc[i], data[' Pitch'].iloc[i],
        #      data[' Yaw'].iloc[i]] for i in data.index]

        # clustering.fit(x)
        # identified_clusters = clustering.fit_predict(x)
        # n_cluster = Counter(identified_clusters)
        # major_cluster = 0 if n_cluster[0] > n_cluster[1] else 1

        colors = ['darkblue', 'deepskyblue', 'red', 'pink']
        labels = ['Trusted - Cluster 1', 'Untrusted - Cluster 1',
                  'Trusted - Cluster 2', 'Untrusted - Cluster 2']
        color = []
        for i in data.index:
            if data[' Trust Factor'].loc[i] >= self.trust_threshold:
                if self.identified_clusters[i] == self.major_cluster:
                    color.append(colors[0])
                else:
                    color.append(colors[2])
            else:
                if self.identified_clusters[i] == self.major_cluster:
                    color.append(colors[1])
                else:
                    color.append(colors[3])

        fig, (ax1, ax2, ax3) = plt.subplots(1, 3)
        fig.suptitle(
            f'Angle measurement, clustered, with trust threshold at {self.trust_threshold}', fontsize=20)

        color_patchs = []
        for c, l in zip(colors, labels):
            color_patchs.append(mpatches.Patch(color=c, label=l))

        for ax, key in zip((ax1, ax2, ax3), (' Roll', ' Pitch', ' Yaw')):
            ax.scatter(x=data.index, y=data[key], c=color)
            ax.legend(handles=color_patchs)
            ax.set_xlabel('Index of measure')
            ax.set_ylabel(key[1:-1] + " (deg)")
            ax.set_title(
                f"Total STD : {self.dict_std[key]['total']:.4f}\n Major cluster STD : {self.dict_std[key]['clustered']:.4f}\nMajor cluster trusted STD : {self.dict_std[key]['clustered_trusted']:.4f}")

        fig2 = plt.figure()
        fig2.suptitle('3D plot of the measured angles', fontsize=20)
        ax3d = fig2.add_subplot(111, projection='3d')
        ax3d.scatter(xs=data[' Roll'], ys=data[' Pitch'],
                     zs=data[' Yaw'], color=color)

        ax3d.set_xlabel('Roll')
        ax3d.set_ylabel('Pitch')
        ax3d.set_zlabel('Yaw')
        ax3d.legend(handles=color_patchs)

    def plot_points(self):
        """
            plot points of measure
        """
        self.get_clustering()
        data = self.data
        fig, (ax1, ax2, ax3) = plt.subplots(1, 3)
        fig.suptitle(
            f'marker position, clustered with the angle, with trust threshold at {self.trust_threshold}', fontsize=20)
        colors = ['darkblue', 'deepskyblue', 'red', 'pink']
        labels = ['Trusted - Cluster 1', 'Untrusted - Cluster 1',
                  'Trusted - Cluster 2', 'Untrusted - Cluster 2']
        color = []
        for i in data.index:
            if data[' Trust Factor'].loc[i] >= self.trust_threshold:
                if self.identified_clusters[i] == self.major_cluster:
                    color.append(colors[0])
                else:
                    color.append(colors[2])
            else:
                if self.identified_clusters[i] == self.major_cluster:
                    color.append(colors[1])
                else:
                    color.append(colors[3])
        color_patchs = []
        for c, l in zip(colors, labels):
            color_patchs.append(mpatches.Patch(color=c, label=l))
        for ax, key in zip((ax1, ax2, ax3), (' X', ' Y', ' Z')):
            ax.scatter(x=data.index, y=data[key], c=color)
            ax.legend(handles=color_patchs)
            ax.set_xlabel('Index of measure')
            ax.set_ylabel(key[1:] + " m")
            ax.set_title(
                f"Total trusted STD : {getattr(self,key[1:].lower()+'_std'):.4f}")

        fig2 = plt.figure()
        fig2.suptitle('3D plot of the measured angles', fontsize=20)
        ax3d = fig2.add_subplot(111, projection='3d')
        ax3d.scatter(xs=data[' X'], ys=data[' Y'],
                     zs=data[' Z'], c=color)

        ax3d.set_xlabel('X')
        ax3d.set_ylabel('Y')
        ax3d.set_zlabel('Z')
        ax3d.legend(handles=color_patchs)

    def plot_transformed_points(self):
        """
            plot points transformed by tf
        """
        self.get_clustering()
        data = self.data
        fig, (ax1, ax2, ax3) = plt.subplots(1, 3)
        fig.suptitle(
            f'Tools position, clustered with the angle, with trust threshold at {self.trust_threshold}', fontsize=20)
        colors = ['darkblue', 'deepskyblue', 'red', 'pink']
        labels = ['Trusted - Cluster 1', 'Untrusted - Cluster 1',
                  'Trusted - Cluster 2', 'Untrusted - Cluster 2']
        color = []
        for i in data.index:
            if data[' Trust Factor'].loc[i] >= self.trust_threshold:
                if self.identified_clusters[i] == self.major_cluster:
                    color.append(colors[0])
                else:
                    color.append(colors[2])
            else:
                if self.identified_clusters[i] == self.major_cluster:
                    color.append(colors[1])
                else:
                    color.append(colors[3])
        color_patchs = []
        for c, l in zip(colors, labels):
            color_patchs.append(mpatches.Patch(color=c, label=l))
        for ax, key in zip((ax1, ax2, ax3), ('transformed_x', 'transformed_y', 'transformed_z')):
            color = 'blue'
            ax.scatter(x=data.index, y=data[key], c=color)
            ax.legend(handles=color_patchs)
            ax.set_xlabel('Index of measure')
            ax.set_ylabel(key + " m")
            ax.set_title(
                f"Marker postion STD : {data[' ' +key[-1].upper()].std():.4f}\nTransformed points STD : {data[key].std():.4f}")

        for ax, key in zip((ax1, ax2, ax3), (' X', ' Y', ' Z')):
            color = 'red'
            ax.scatter(x=data.index, y=data[key], c=color)
            ax.legend(handles=color_patchs)
            ax.set_xlabel('Index of measure')
            ax.set_ylabel(key + " m")

        fig2 = plt.figure()
        fig2.suptitle('3D plot of the measured angles', fontsize=20)
        ax3d = fig2.add_subplot(111, projection='3d')
        ax3d.scatter(xs=data['transformed_x'], ys=data['transformed_y'],
                     zs=data['transformed_z'], c='blue')

        ax3d.scatter(xs=data[' X'], ys=data[' Y'],
                     zs=data[' Z'], c='red')
        ax3d.set_xlabel('X')
        ax3d.set_ylabel('Y')
        ax3d.set_zlabel('Z')
        ax3d.legend(handles=color_patchs)

    @property
    def tool_frame(self) -> str:
        return f'tool_{self.tool_side}_frame'

    @property
    def marker_frame(self) -> str:
        return f'marker_{self.robot_side}_frame'

    def get_marker_frame_measure(self, i: int) -> str:
        return f'{self.marker_frame}_measure_{i}'

    def get_tool_frame_measure(self, i: int) -> str:
        return f'{self.tool_frame}_measure_{i}'

    def get_robot_frame_measure(self, i: int) -> str:
        return f'robot_frame_measure_{i}'

    def get_avg_hole_frame(self) -> str:
        return f'set{self.set}_structure{self.structure}_plaque{self.plaque}_hole{self.hole}_frame'

    def get_avg_marker_frame(self) -> str:
        return f'set{self.set}_structure{self.structure}_plaque{self.plaque}_marker{self.hole}_frame'

    def transform_measures(self) -> None:
        """
            get all measures from an acquisition and transform then to get the positin of the tool
        """
        self.n_time_called += 1
        data = self.data

        res = {'x': [], 'y': [], 'z': [],
               'qx': [], 'qy': [], 'qz': [], 'qw': []}

        for i in data.index:

            # frame per measure, usefull if we need to broadcast all tf
            frame_marker_mesure = self.get_marker_frame_measure(i)
            frame_tool_mesure = self.get_tool_frame_measure(i)
            frame_robot_mesure = self.get_robot_frame_measure(i)

            # We use the measure of the camera to create a tf from camera to marker
            tf_measure = tf.TransformStamped()
            tf_measure.header.stamp = Acquisition.now
            tf_measure.header.frame_id = 'navcam'
            tf_measure.child_frame_id = frame_marker_mesure

            tf_measure.transform.translation = Vector3(x=data[' X'].loc[i],
                                                       y=data[' Y'].loc[i],
                                                       z=data[' Z'].loc[i])

            # this have always been how I compute the marker quaternion from the navcam measure
            q = quaternion_from_euler(
                (deg2rad(data[' Pitch'].loc[i]), deg2rad(data[' Roll'].loc[i]), deg2rad(data[' Yaw'].loc[i])+90), 'ryxz')
            tf_measure.transform.rotation = Quaternion(
                x=q[0], y=q[1], z=q[2], w=q[3])

            # We set this tf in the local buffer
            self.local_tf_buffer.set_transform(
                tf_measure, 'measure from the camera')

            # In the node buffer we look for the tf from marker to tool

            # 1 : marker -> robot
            tf_marker_robot = Acquisition.global_tf_buffer.lookup_transform(
                self.marker_frame, 'robot_frame', Acquisition.now)
            # We change the name of the frames so we can broadcast then all at the same time
            tf_marker_robot.header.frame_id = frame_marker_mesure
            tf_marker_robot.child_frame_id = frame_robot_mesure
            self.local_tf_buffer.set_transform(tf_marker_robot, 'robot tf')

            # 2 : robot -> tool
            tf_robot_tool = Acquisition.global_tf_buffer.lookup_transform(
                'robot_frame', self.tool_frame, Acquisition.now)
            # We change the name of the frames so we can broadcast then all at the same time
            tf_robot_tool.header.frame_id = frame_robot_mesure
            tf_robot_tool.child_frame_id = frame_tool_mesure
            self.local_tf_buffer.set_transform(tf_robot_tool, 'robot tf')

            # we look for the tf from the camera to the tool
            tf_cam_tool = self.local_tf_buffer.lookup_transform(
                'navcam', frame_tool_mesure, Acquisition.now)

            res['x'].append(tf_cam_tool.transform.translation.x)
            res['y'].append(tf_cam_tool.transform.translation.y)
            res['z'].append(tf_cam_tool.transform.translation.z)
            res['qx'].append(tf_cam_tool.transform.rotation.x)
            res['qy'].append(tf_cam_tool.transform.rotation.y)
            res['qz'].append(tf_cam_tool.transform.rotation.z)
            res['qw'].append(tf_cam_tool.transform.rotation.w)

        self.transformed_data = pd.DataFrame(
            res, index=data.index, columns=['x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])

        # we compute some metadata about that
        self.transformed_x_mean = self.transformed_data['x'].mean()
        self.transformed_y_mean = self.transformed_data['y'].mean()
        self.transformed_z_mean = self.transformed_data['z'].mean()

        self.transformed_x_median = self.transformed_data['x'].median()
        self.transformed_y_median = self.transformed_data['y'].median()
        self.transformed_z_median = self.transformed_data['z'].median()

        self.transformed_x_std = self.transformed_data['x'].std()
        self.transformed_y_std = self.transformed_data['y'].std()
        self.transformed_z_std = self.transformed_data['z'].std()

    def get_random_transform_set(self, in_cluster=False) -> Iterable[tf.TransformStamped]:
        """
            return a random set of transform stamped (random in a sense of using one random measure)
            if in_clusted is True, the random measure is forced to be in the major angle cluster
            tf cam->marker ; tf marker->robot ; tf robot->tool
        """
        if not in_cluster:
            i = random.choice(self.data.index)

            # tf cam->marker
            tf_cam_marker: tf.TransformStamped = self.local_tf_buffer.lookup_transform(
                'navcam', self.get_marker_frame_measure(i), self.now)
            tf_cam_marker.child_frame_id = self.marker_frame

            # tf marker->robot
            tf_marker_robot: tf.TransformStamped = self.local_tf_buffer.lookup_transform(
                self.get_marker_frame_measure(i), self.get_robot_frame_measure(i), self.now)
            tf_marker_robot.child_frame_id = 'robot_frame'
            tf_marker_robot.header.frame_id = self.marker_frame

            # tf robot->tool
            tf_robot_tool: tf.TransformStamped = self.local_tf_buffer.lookup_transform(
                self.get_robot_frame_measure(i), self.get_tool_frame_measure(i), self.now)
            tf_robot_tool.child_frame_id = self.tool_frame
            tf_robot_tool.header.frame_id = 'robot_frame'

        return (tf_cam_marker, tf_marker_robot, tf_robot_tool)

    def compute_mean_tf_tool(self) -> TransformStamped:
        """
            return the mean tf for culstered trusted measure
        """
        data = self.transformed_data[self.identified_clusters ==
                                     self.major_cluster][self.data[' Trust Factor'] == 5]
        if len(data.index) == 0:
            data = self.transformed_data[self.identified_clusters ==
                                         self.major_cluster]
        mean_tf = TransformStamped()
        mean_tf.header.stamp = self.now
        mean_tf.header.frame_id = 'navcam'
        mean_tf.child_frame_id = self.local_avg_tool_frame
        mean_tf.transform.translation.x = self.transformed_x_mean_clustered_trusted
        mean_tf.transform.translation.y = self.transformed_y_mean_clustered_trusted
        mean_tf.transform.translation.z = self.transformed_z_mean_clustered_trusted

        Q = []
        for i in data.index:
            Q.append([data['qx'].loc[i], data['qy'].loc[i],
                     data['qz'].loc[i], data['qw'].loc[i]])

        w = [1]*len(Q)
        q_mean = avg_q(Q, w)
        mean_tf.transform.rotation.x = q_mean[0]
        mean_tf.transform.rotation.y = q_mean[1]
        mean_tf.transform.rotation.z = q_mean[2]
        mean_tf.transform.rotation.w = q_mean[3]

        Acquisition.global_tf_buffer.set_transform(mean_tf, 'avg measure')

    def compute_mean_tf_marker(self) -> TransformStamped:
        """
            return the mean tf for culstered trusted measure
        """
        mean_tf = TransformStamped()
        mean_tf.header.stamp = self.now
        mean_tf.header.frame_id = 'navcam'
        mean_tf.child_frame_id = self.local_avg_marker_frame
        mean_tf.transform.translation.x = self.x_mean_clustered_trusted
        mean_tf.transform.translation.y = self.y_mean_clustered_trusted
        mean_tf.transform.translation.z = self.z_mean_clustered_trusted

        q_mean = self.q_mean
        mean_tf.transform.rotation.x = q_mean[0]
        mean_tf.transform.rotation.y = q_mean[1]
        mean_tf.transform.rotation.z = q_mean[2]
        mean_tf.transform.rotation.w = q_mean[3]

        Acquisition.global_tf_buffer.set_transform(mean_tf, 'avg measure')

    def get_acquisition_key(self) -> str:
        """
            return the acquisition
        """
        return f'{Acquisition.root_dir}/set{self.set}/struct{self.structure}/plaque{self.plaque}/pose{self.hole}'

    def get_base_frame(self):
        """
            return the base frame
        """
        return Acquisition.base_frames_tool[self.get_acquisition_key()]

    def get_base_frame(self):
        """
            return the base frame
        """
        return Acquisition.base_frames_marker[self.get_acquisition_key()]

    def get_base_acquisition(self):
        """
            return the base frame
        """
        return Acquisition.base_acquisition[self.get_acquisition_key()]

    def compute_tool_pose_in_base_frame(self) -> TransformStamped:
        """
            Compute the pose of the measure in the base frame
        """
        self.base_frame = self.get_base_frame()
        if Acquisition.global_tf_buffer.can_transform(self.local_avg_tool_frame, self.base_frame, self.now):
            return Acquisition.global_tf_buffer.lookup_transform(self.base_frame, self.local_avg_tool_frame, self.now)
        else:
            res = TransformStamped()
            res.header.frame_id = "failed"
            return res

    def compute_marker_pose_in_base_frame(self) -> TransformStamped:
        """
            Compute the pose of the measure in the base frame
        """
        self.base_frame = self.get_base_frame()
        if Acquisition.global_tf_buffer.can_transform(self.local_avg_marker_frame, self.base_frame, self.now):
            return Acquisition.global_tf_buffer.lookup_transform(self.base_frame, self.local_avg_marker_frame, self.now)
        else:
            res = TransformStamped()
            res.header.frame_id = "failed"
            return res
