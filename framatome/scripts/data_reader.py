from acquisition import Acquisition, deg2rad, is_axes_valid, rad2deg, is_axes_valid, quaternion_from_euler, avg_q
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import Quaternion, Vector3, PointStamped
import tf2_ros as tf
import pandas as pd
import numpy as np
from math import cos, sin, pi
from scipy.spatial import transform
from transforms3d.quaternions import mat2quat
from transforms3d.euler import euler2quat, quat2euler, quat2mat, _AXES2TUPLE
import matplotlib.patches as mpatches
from mpl_toolkits.mplot3d import Axes3D
from typing import Collection, Dict, Iterable
import os
from glob import glob
import random
import yaml

import cv2


class DataReader:
    """
        Factory for the data acquisition
    """

    def __init__(self, root_dir: str, data_hierarchie: Iterable[str], plotting: bool = False, now=None) -> None:
        """
            root_dir is the root of the dataset
        """
        self.now = now
        self.plot = plotting
        self.root_dir = root_dir
        self.data_hierarchie = data_hierarchie
        self.marker_size = 0.24
        self.marker_geometry = np.array([[self.marker_size / 2, 0, self.marker_size / 2, 1],
                                         [-self.marker_size / 2, 0,
                                             self.marker_size / 2, 1],
                                         [-self.marker_size / 2, 0, -
                                             self.marker_size / 2, 1],
                                         [self.marker_size / 2, 0, -self.marker_size / 2, 1]]).T

        Acquisition.metadata = self.get_metadata()
        Acquisition.base_transforms = self.get_transforms_metadata()
        Acquisition.robot_data = self.get_robot_metadata()
        self.n_set = self.get_n_set()
        self.dataset_serial = []
        self.get_all_dataset_dir(self.root_dir)

    def get_n_set(self) -> int:
        """
            return the number of set of acquisition there is (complete or not)
        """
        return len(glob(os.path.join(self.root_dir, 'set*')))

    def get_all_dataset_dir(self, path: str, hierarchie=0) -> Dict:
        """
            get all dataset sub directory
        """
        if os.path.isdir(path):
            d = {}
            for name in os.listdir(path):
                if os.path.isdir(os.path.join(path, name)) and self.data_hierarchie[hierarchie] in name:
                    d[name] = self.get_all_dataset_dir(
                        os.path.join(path, name), hierarchie+1)
                else:
                    if 'posit_nmea.csv' in name:
                        obj = Acquisition(os.path.join(path, name))
                        self.dataset_serial.append(obj)

        else:
            d = os.path.getsize(path)
        return d

    def get_metadata(self) -> pd.DataFrame:
        """
            get the metadatafile
        """
        return pd.read_csv(
            os.path.join(self.root_dir, 'condition_metadata.csv'))

    def get_transforms_metadata(self) -> dict:
        """
            get the transform data file
        """
        transforms = {'markers': {}, 'tools': {}}
        if self.plot:
            fig = plt.figure()
            ax3d = fig.add_subplot(111, projection='3d')
            ax3d.plot(xs=self.marker_geometry[0, :],
                      ys=self.marker_geometry[1, :],
                      zs=self.marker_geometry[2, :],
                      label='Base marker'
                      )
        with open(os.path.join(self.root_dir, 'transforms_metadata.yaml')) as handle:
            raw_data = yaml.safe_load(handle)
        for name, tool in raw_data['tools'].items():
            M = np.eye(4)
            M[0, 3] = tool['x']/1000
            M[1, 3] = tool['y']/1000
            M[2, 3] = tool['z']/1000
            transforms['tools'][name] = M

        for name, marker in raw_data['markers'].items():
            id = marker['id']
            X, Y, Z = [], [], []
            p = []
            for n_corner, corner in marker['corners'].items():
                x = corner['x']/1000
                y = corner['y']/1000
                z = corner['z']/1000
                X.append(x)
                Y.append(y)
                Z.append(z)
                p.append(np.array([x, y, z]))
                if self.plot:
                    ax3d.text(x, y, z, f'{n_corner}')

            measured_points = np.concatenate(
                [[X], [Y], [Z], [[1, 1, 1, 1]]], axis=0)
            R, t = self.rigid_transform_3D(
                self.marker_geometry[0:3, :], measured_points[0:3, :])
            M_rec = np.eye(4)
            M_rec[0:3, 0:3] = R
            M_rec[0:3, 3] = t.T
            # M_rec[0:3, 3] = -t.T
            transforms['markers'][name] = M_rec

            if self.plot:
                print(M_rec)
                print(measured_points)
                reproj = M_rec @ measured_points

                c1 = ((X[0] + X[2]) / 2,
                      (Y[0] + Y[2]) / 2,
                      (Z[0] + Z[2]) / 2)
                c2 = ((X[1] + X[3]) / 2,
                      (Y[1] + Y[3]) / 2,
                      (Z[1] + Z[3]) / 2)
                c = ((c1[0] + c2[0]) / 2,
                     (c1[1] + c2[1]) / 2,
                     (c1[2] + c2[2]) / 2)

                ax3d.plot(xs=[c[0]], ys=[c[1]], zs=[c[2]], marker='x')
                ax3d.plot(xs=X, ys=Y, zs=Z, label=f'marker {name}')
                ax3d.plot(xs=reproj[0, :],
                          ys=reproj[1, :],
                          zs=reproj[2, :],
                          linestyle='dashed',
                          label=f'reprojection marker {name}')
        if self.plot:

            ax3d.legend()
            ax3d.set_xlabel('x')
            ax3d.set_ylabel('y')
            ax3d.set_zlabel('z')
            plt.show()

        for namedict in ['markers', 'tools']:
            for name, transfo in transforms[namedict].items():
                rot = Rotation.from_matrix(transfo[:3, :3])
                q = rot.as_quat()
                t = transfo[:3, 3]
                tf_marker = tf.TransformStamped()
                tf_marker.header.frame_id = "robot_frame"
                tf_marker.header.stamp = self.now
                tf_marker.child_frame_id = f'{namedict[:-1]}_{name}_frame'
                tf_marker.transform.translation = Vector3(
                    x=t[0], y=t[1], z=t[2])
                tf_marker.transform.rotation = Quaternion(
                    x=q[0], y=q[1], z=q[2], w=q[3])
                Acquisition.global_tf_buffer.set_transform(
                    tf_marker, 'measurement of the robot')

        return transforms

    def rigid_transform_3D(self, A, B) -> Iterable[np.array]:
        """
            Input: expects 3xN matrix of points
            Returns R,t
            R = 3x3 rotation matrix
            t = 3x1 column vector
        """
        assert A.shape == B.shape

        num_rows, num_cols = A.shape
        if num_rows != 3:
            raise Exception(
                f"matrix A is not 3xN, it is {num_rows}x{num_cols}")

        num_rows, num_cols = B.shape
        if num_rows != 3:
            raise Exception(
                f"matrix B is not 3xN, it is {num_rows}x{num_cols}")

        # find mean column wise
        centroid_A = np.mean(A, axis=1)
        centroid_B = np.mean(B, axis=1)

        # ensure centroids are 3x1
        centroid_A = centroid_A.reshape(-1, 1)
        centroid_B = centroid_B.reshape(-1, 1)

        # subtract mean
        Am = A - centroid_A
        Bm = B - centroid_B

        H = Am @ np.transpose(Bm)

        # sanity check
        # if linalg.matrix_rank(H) < 3:
        #    raise ValueError("rank of H = {}, expecting 3".format(linalg.matrix_rank(H)))

        # find rotation
        U, S, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T

        # special reflection case
        if np.linalg.det(R) < 0:
            # print("det(R) < R, reflection detected!, correcting for it ...")
            Vt[2, :] *= -1
            R = Vt.T @ U.T

        t = -R @ centroid_A + centroid_B

        return R, t

    def get_robot_metadata(self) -> dict:
        """
            get which marker goes with which struct/plaque
        """
        with open(os.path.join(self.root_dir, 'robot_metadata.yaml')) as handle:
            return yaml.safe_load(handle)

    def get_n_valid(self) -> int:
        n = 0
        for obj in self.dataset_serial:
            if obj.valid:
                n += 1
        return n

    def get_xyz(self):
        X = []
        Y = []
        Z = []
        for obj in self.dataset_serial:
            if obj.valid and obj.structure == 2:
                (x, y, z) = obj.transformed_x_median, obj.transformed_y_median, obj.transformed_z_median
                X.append(x)
                Y.append(y)
                Z.append(z)
        return (X, Y, Z)

    def compute_metadata_out(self) -> pd.DataFrame:
        """
            build the output dataframe
        """
        col = []
        for k, v in Acquisition.output_metadata.items():
            if v == 'ok':
                col.append(k)

        lines = {name: [] for name in col}
        n_acq = len(self.dataset_serial)
        for i, acquisition in enumerate(self.dataset_serial):
            print(f'{i/n_acq*100:3.1f}% computed, doing {acquisition}')
            try:
                acquisition.compute_all_metadata()
            except:
                print('WARNING : failed to compute all metadata')
            for name in col:
                try:
                    lines[name].append(getattr(acquisition, name))
                except:
                    lines[name].append(0)
        output = pd.DataFrame(data=lines, columns=col)
        return output


if __name__ == '__main__':

    root_dir = '/home/victor/Downloads/framatome'
    data_hierarchie = ['set', 'struct', 'plaque', 'pose']

    reader = DataReader(root_dir, data_hierarchie, plotting=False)
    print(f'We have {reader.get_n_valid()} correct acquisitions')

    for i in [50]:
        acq = reader.dataset_serial[i]
        acq.transform_all_measure()
        print(acq)

        print('tf total:')
        print(acq.total_transform)
        r = np.array(transform.Rotation.from_matrix(
            acq.total_transform[0:3, 0:3]).as_euler('xyz'))*180/np.pi
        t = np.array(acq.total_transform[0:3, 3])
        print(r, t)
        print(np.linalg.norm(t))
        print()

        print('tf marker:')
        print(acq.marker_transform)
        r = np.array(transform.Rotation.from_matrix(
            acq.marker_transform[0:3, 0:3]).as_euler('xyz'))*180/np.pi
        t = np.array(acq.marker_transform[0:3, 3])
        print(r, t)
        print(np.linalg.norm(t))
        print()

    # acq.transform_all_measure()
    # x = acq.data[' X']
    # y = acq.data[' Y']
    # z = acq.data[' Z']
    # tx = acq.transformed_data['x']
    # ty = acq.transformed_data['y']
    # tz = acq.transformed_data['z']

    # data = reader.compute_metadata_out()

    # data = data[data['structure'] == 1]
    # data = data[data['plaque'] == 1]

    # print(data.columns)

    fig = plt.figure()
    ax3d = fig.add_subplot(111, projection='3d')
    # ax3d.scatter(xs=data['transformed_x_mean'],
    #             ys=data['transformed_y_mean'],
    #             zs=data['transformed_z_mean'],
    #             label='transformed'
    #             )

    # ax3d.scatter(xs=data['x_mean'],
    #             ys=data['y_mean'],
    #             zs=data['z_mean'],
    #             label='raw'
    #             )
    # print(Acquisition.base_transforms)
    ax3d.plot(xs=reader.marker_geometry[0, :],
              ys=reader.marker_geometry[1, :],
              zs=reader.marker_geometry[2, :],
              label='Base marker'
              )
    for i in range(4):
        x = reader.marker_geometry[0, i]
        y = reader.marker_geometry[1, i]
        z = reader.marker_geometry[2, i]
        ax3d.text(x, y, z, f'{i+1}')

    with open(os.path.join(reader.root_dir, 'transforms_metadata.yaml')) as handle:
        raw_data = yaml.safe_load(handle)
    centers = {}
    for name, marker in raw_data['markers'].items():
        X, Y, Z, C = [], [], [], []
        for n_corner, corner in marker['corners'].items():
            x = corner['x']/1000
            y = corner['y']/1000
            z = corner['z']/1000
            X.append(x)
            Y.append(y)
            Z.append(z)
            C.append(np.array([x, y, z]))
            ax3d.text(x, y, z, f'{n_corner}')

        ax3d.plot(xs=X, ys=Y, zs=Z, label=f'marker {name}')
        centers[name] = (C[0] + C[2]) / 2

        measured_points = np.concatenate([[X], [Y], [Z]], axis=0)

    for name, tf in Acquisition.base_transforms['markers'].items():
        t = tf[0, :3]
        x = t[0]
        y = t[1]
        z = t[2]
        ax3d.scatter(
            xs=centers[name][0],
            ys=centers[name][1],
            zs=centers[name][2],
            marker='x'
        )
        ax3d.quiver(0, 0, 0, x, y, z)

    for name, tf in Acquisition.base_transforms['tools'].items():
        x = [tf[0, 3]]
        y = [tf[1, 3]]
        z = [tf[2, 3]]
        ax3d.quiver(0, 0, 0, x, y, z, label='tool '+name)
        ax3d.scatter(xs=x, ys=y, zs=z, label='tool '+name, marker='x')

    ax3d.scatter(xs=[0], ys=[0], zs=[0], label='Ref point '+name, marker='x')
    ax3d.legend()
    ax3d.set_xlabel('X')
    ax3d.set_ylabel('Y')
    ax3d.set_zlabel('Z')
    max_limit = 1
    ax3d.set_xlim3d(-max_limit, max_limit)
    ax3d.set_ylim3d(-max_limit, max_limit)
    ax3d.set_zlim3d(-max_limit, max_limit)
    plt.show()


def trust_std(reader: DataReader):
    trust = []
    N = []
    x_std = []
    y_std = []
    z_std = []
    roll_std = []
    pitch_std = []
    yaw_std = []

    for i in range(2, 6):
        trust.append(i)
        Acquisition.trust_threshold = i
        metadata_output = reader.compute_metadata_out()
        N.append(metadata_output['n_trusted_mesure'].sum())
        x_std.append(metadata_output['x_std'].mean())
        y_std .append(metadata_output['y_std'].mean())
        z_std.append(metadata_output['z_std'].mean())
        roll_std.append(metadata_output['roll_std'].mean())
        pitch_std.append(metadata_output['pitch_std'].mean())
        yaw_std.append(metadata_output['yaw_std'].mean())

    fig, (ax1, ax2) = plt.subplots(1, 2)

    ax1.scatter(trust, x_std, label='x')
    ax1.scatter(trust, y_std, label='y')
    ax1.scatter(trust, z_std, label='z')
    ax2.scatter(trust, roll_std, label='roll')
    ax2.scatter(trust, pitch_std, label='pitch')
    ax2.scatter(trust, yaw_std, label='yaw')

    ax1.legend()
    ax2.legend()

    fig.suptitle(
        'Standard deviation as a function of the trust factor', fontsize=20)

    ax1.set_xlabel('Trust', fontsize=15)
    ax1.set_ylabel('STD (m)', fontsize=15)
    ax2.set_xlabel('Trust', fontsize=15)
    ax2.set_ylabel('STD (deg)', fontsize=15)

    plt.show()
