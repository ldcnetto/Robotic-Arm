import numpy as np
import math
import matplotlib.pyplot as plt
from kinematics import ForwardKinematics

class JointTrajectory:
    @staticmethod
    def get_angle_complement(theta):
        return np.sign(theta) * (2 * math.pi - abs(theta))

    @staticmethod
    def get_deloc_vector(delta, a, v_max, ts):
        if delta == 0:
            return np.array([0])
        if delta < 0:
            a = -a
            v_max = -v_max
        t_acc = v_max / a
        deloc_acc = a * t_acc**2 / 2
        deloc_inertia = (delta - 2 * deloc_acc)
        if deloc_acc * 2 > delta:
            t_acc = math.sqrt(delta / a)
            v_max = t_acc * a
            deloc_acc = a * t_acc**2 / 2
            deloc_inertia = 0
        t_inertia = deloc_inertia / v_max
        t_total = t_inertia + 2 * t_acc
        sample_array = np.arange(0, t_total + ts, ts)
        vel_array = []
        deloc_array = []
        for t in sample_array:
            vel_sample = None
            deloc_sample = None
            if t < t_acc:
                vel_sample = t * a
                deloc_sample = a * t**2 / 2
            elif t < (t_total - t_acc):
                vel_sample = v_max
                deloc_sample = deloc_acc + v_max * (t - t_acc)
            elif t < t_total:
                delta_t = (t - t_acc - t_inertia)
                vel_sample = v_max - a * delta_t
                deloc_sample = deloc_acc + deloc_inertia + v_max * delta_t - a * delta_t**2 / 2
            else:
                vel_sample = 0
                deloc_sample = delta
            vel_array.append(vel_sample)
            deloc_array.append(deloc_sample)
        return np.array(deloc_array)

    @staticmethod
    def get_joint_traj_vector(init, final, a, v_max, ts):
        delta = final - init
        if abs(delta) > math.pi:
            delta = -JointTrajectory.get_angle_complement(delta)
        deloc_vector = JointTrajectory.get_deloc_vector(delta, a, v_max, ts)
        traj_vector = deloc_vector + init
        return traj_vector

    @staticmethod
    def traj_joint(theta1_init, theta2_init, theta1_final, theta2_final, ts=0.1):
        a = 1.0
        v_max = 0.91
        a_1, a_2 = JointTrajectory.get_ajusted_accelerations(theta1_init, theta2_init, theta1_final, theta2_final, a, v_max, ts)
        traj_theta1 = JointTrajectory.get_joint_traj_vector(theta1_init, theta1_final, a_1, v_max, ts)
        traj_theta2 = JointTrajectory.get_joint_traj_vector(theta2_init, theta2_final, a_2, v_max, ts)
        traj_theta1, traj_theta2 = JointTrajectory.make_arrays_equal_size(traj_theta1, traj_theta2)
        return traj_theta1, traj_theta2

    @staticmethod
    def make_arrays_equal_size(arr1, arr2):
        len1 = len(arr1)
        len2 = len(arr2)
        if len1 < len2:
            arr1 = np.pad(arr1, (0, len2 - len1), mode='edge')
        elif len1 > len2:
            arr2 = np.pad(arr2, (0, len1 - len2), mode='edge')
        return arr1, arr2

    @staticmethod
    def calculate_trajectory_time(delta, a, v_max):
        if delta == 0:
            return 0
        if delta < 0:
            delta = -delta
        t_acc = v_max / a
        deloc_acc = a * t_acc**2 / 2
        deloc_inertia = (delta - 2 * deloc_acc)
        if deloc_acc * 2 > delta:
            t_acc = math.sqrt(delta / a)
            v_max = t_acc * a
            deloc_acc = a * t_acc**2 / 2
            deloc_inertia = 0
        t_inertia = deloc_inertia / v_max
        t_total = t_inertia + 2 * t_acc
        return t_total

    @staticmethod
    def get_ajusted_accelerations(t1_init, t2_init, t1_final, t2_final, a, v_max, ts):
        delta1 = t1_final - t1_init
        delta2 = t2_final - t2_init
        time1 = JointTrajectory.calculate_trajectory_time(delta1, a, v_max)
        time2 = JointTrajectory.calculate_trajectory_time(delta2, a, v_max)
        if time1 < time2:
            a_ajusted = v_max**2 / (v_max * time2 - abs(delta1))
            if time2 <= 2 * v_max / a_ajusted:
                a_ajusted = 4 * abs(delta1) / time2**2
            return a_ajusted, a
        elif time2 < time1:
            a_ajusted = v_max**2 / (v_max * time1 - abs(delta2))
            if time1 <= 2 * v_max / a_ajusted:
                a_ajusted = 4 * abs(delta2) / time1**2
            return a, a_ajusted
        return a, a

class EuclideanTrajectory:
    @staticmethod
    def get_fk_from_traj(traj_1, traj_2):
        x_array = []
        y_array = []
        for (t1, t2) in zip(traj_1, traj_2):
            coords = ForwardKinematics.fk(t1, t2)
            x_array.append(coords[0])
            y_array.append(coords[1])
        return x_array, y_array

    @staticmethod
    def plot_trajs(traj_1, traj_2):
        r = np.full(traj_1.shape, 1)
        fig = plt.figure(figsize=(10, 12))
        gs = fig.add_gridspec(3, 2)
        ax = fig.add_subplot(gs[0, 0])
        ax.plot(traj_1)
        ax.grid(True)
        ax.set_title("Trajectory 1")
        ax = fig.add_subplot(gs[0, 1])
        ax.plot(traj_2)
        ax.grid(True)
        ax.set_title("Trajectory 2")
        ax = fig.add_subplot(gs[1, 0], projection='polar')
        ax.plot(traj_1, r)
        ax.set_title("Trajectory 1 polar")
        ax = fig.add_subplot(gs[1, 1], projection='polar')
        ax.plot(traj_2, r)
        ax.set_title("Trajectory 2 polar")
        ax = fig.add_subplot(gs[2, :])
        euclidian_traj = EuclideanTrajectory.get_fk_from_traj(traj_1, traj_2)
        ax.set_xlim([-2.5, 2.5])
        ax.set_ylim([-2.5, 2.5])
        ax.plot(euclidian_traj[0], euclidian_traj[1])
        ax.grid(True)
        ax.set_title("Robot Trajectory")
        fig.tight_layout()
        fig.show()