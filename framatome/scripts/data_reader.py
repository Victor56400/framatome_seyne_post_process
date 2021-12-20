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

        Acquisition.root_dir = self.root_dir
        Acquisition.metadata = self.get_metadata()
        Acquisition.base_transforms = self.get_transforms_metadata()
        Acquisition.robot_data = self.get_robot_metadata()
        self.n_set = self.get_n_set()
        self.dataset_serial: Iterable[Acquisition] = []
        self.dataset_dict: Dict[Acquisition] = dict()
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
                        self.dataset_dict[path] = obj
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
            if obj.valid and obj.structure == 1:
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
                break
            except:
                print('WARNING : failed to compute all metadata')
            for name in col:
                try:
                    lines[name].append(getattr(acquisition, name))
                except:
                    lines[name].append(0)
        output = pd.DataFrame(data=lines, columns=col)
        return output

    def get_acquisition(self, set: int, structure: int, plaque: int, hole: int) -> Acquisition:
        """
            return the acquisition
        """
        return self.dataset_dict[self.get_acquisition_key(set, structure, plaque, hole)]

    def get_acquisition_key(self, set: int, structure: int, plaque: int, hole: int) -> str:
        """
            return the acquisition
        """
        return f'{self.root_dir}/set{set}/struct{structure}/plaque{plaque}/pose{hole}'

    def compute_all_base_frames(self, verbose=True):
        """
            Compute all the base frames for all plaques
        """
        for i in range(1, Acquisition.n_set+1):  # for all set a acquisition
            base_frame_tool, base_frame_marker, base_acq = None, None, None
            for j in range(1, Acquisition.n_structure+1):  # for all structures
                for k in range(1, Acquisition.n_plaque+1):  # for each plaques
                    for l in range(1, Acquisition.n_hole+1):  # for each holes
                        acq_key = self.get_acquisition_key(
                            set=i, structure=j, plaque=k, hole=l)
                        try:
                            acq = self.get_acquisition(
                                set=i, structure=j, plaque=k, hole=l)
                        except KeyError:
                            if verbose:
                                print(f'{acq_key} -> failed to read data')
                        if base_frame_tool is None:
                            try:
                                acq.compute_all_metadata()
                                # acq.compute_mean_tf_tool()
                                acq.compute_mean_tf_marker()
                                base_frame_tool = acq.local_avg_tool_frame
                                base_frame_marker = acq.local_avg_marker_frame
                                base_acq = acq
                                if verbose:
                                    print(f'{acq} -> become base frame')
                                Acquisition.base_frames_tool[acq_key] = base_frame_tool
                                Acquisition.base_frames_marker[acq_key] = base_frame_marker
                                Acquisition.base_acquisition[acq_key] = base_acq
                            except:
                                pass
                        else:
                            Acquisition.base_frames_tool[acq_key] = base_frame_tool
                            Acquisition.base_frames_marker[acq_key] = base_frame_marker
                            Acquisition.base_acquisition[acq_key] = base_acq
                            try:
                                acq.compute_all_metadata()
                                acq.compute_mean_tf_tool()
                                acq.compute_mean_tf_marker()
                                if verbose:
                                    print(
                                        f'{acq} -> take base frame off {acq.get_base_acquisition()}')

                            except:
                                print(f'{acq} -> failed to compute tf ')

    def compute_all_pose_in_base_frame(self):
        """
            compute the pose of all acquistion in the base frame
        """
        col = {'set': [], 'structure': [], 'plaque': [], 'hole': [],
               'reference_frame': [], 'x_tool': [], 'y_tool': [], 'z_tool': [], 'x_marker': [], 'y_marker': [], 'z_marker': []}

        for i in range(1, Acquisition.n_set+1):  # for all set a acquisition
            for j in range(1, Acquisition.n_structure+1):  # for all structures
                for k in range(1, Acquisition.n_plaque+1):  # for each plaques
                    try:
                        for l in range(1, Acquisition.n_hole+1):  # for each holes
                            acq = self.get_acquisition(
                                set=i, structure=j, plaque=k, hole=l)
                            transform_marker = acq.compute_marker_pose_in_base_frame()
                            transform_tool = acq.compute_tool_pose_in_base_frame()
                            col['set'].append(acq.set)
                            col['structure'].append(acq.structure)
                            col['plaque'].append(acq.plaque)
                            col['hole'].append(acq.hole)
                            col['reference_frame'].append(
                                transform_marker.header.frame_id)
                            col['x_marker'].append(
                                transform_marker.transform.translation.x)
                            col['y_marker'].append(
                                transform_marker.transform.translation.y)
                            col['z_marker'].append(
                                transform_marker.transform.translation.z)
                            col['x_tool'].append(
                                transform_tool.transform.translation.x)
                            col['y_tool'].append(
                                transform_tool.transform.translation.y)
                            col['z_tool'].append(
                                transform_tool.transform.translation.z)

                    except KeyError:
                        print(f'{acq} -> could not read data')

        return pd.DataFrame(col, columns=col.keys())

    def print_n_time_called(self):
        """
            debug utils
        """
        for acq in self.dataset_serial:
            print(f'{acq} -> {acq.n_time_called}')

    def plot_all_data(self, set=1, structure=1, plaque=1, hole=6):
        """
            plot the raw data
        """

        acq = self.get_acquisition(
            set=set, structure=structure, plaque=plaque, hole=hole)

        fig = plt.figure(f'{acq}')
        ax3d = fig.add_subplot(111, projection='3d')

        X = acq.data[' X']
        Y = acq.data[' Y']
        Z = acq.data[' Z']
        x_avg = X.mean()
        y_avg = Y.mean()
        z_avg = Z.mean()
        x_std = X.std()
        y_std = Y.std()
        z_std = Z.std()

        ax3d.scatter(xs=X, ys=Y, zs=Z, color='red', label='raw data')
        ax3d.scatter(xs=x_avg, ys=y_avg, zs=z_avg,
                     color='blue', label='mean data')

        ax3d.legend()
        print(f'{acq} -\n   averages : x={x_avg} ; y={y_avg} ; z={z_avg}\n   std : x={x_std} ; y={y_std} ; z{z_std}')
        plt.show()
