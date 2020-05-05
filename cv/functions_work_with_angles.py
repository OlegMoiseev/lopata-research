import numpy as np
from cv2 import RQDecomp3x3
from numpy.linalg import norm


def create_r_x_gamma(angle_roll):
    rot_m_x = np.array([[1, 0, 0],
                        [0, np.cos(angle_roll), -np.sin(angle_roll)],
                        [0, np.sin(angle_roll), np.cos(angle_roll)]])
    return rot_m_x


def create_r_y_beta(angle_pitch):
    rot_m_y = np.array([[np.cos(angle_pitch), 0, np.sin(angle_pitch)],
                        [0, 1, 0],
                        [-np.sin(angle_pitch), 0, np.cos(angle_pitch)]])
    return rot_m_y


def create_r_z_alpha(angle_yaw):
    rot_m_z = np.array([[np.cos(angle_yaw), -np.sin(angle_yaw), 0],
                        [np.sin(angle_yaw), np.cos(angle_yaw), 0],
                        [0, 0, 1]])
    return rot_m_z


def create_rot_m(roll, pitch, yaw):
    """
    :param roll: gamma, x-axis
    :param pitch: beta, y-axis
    :param yaw: alpha, z-axis
    :return: rotation matrix
    """
    r_z_yaw = create_r_z_alpha(yaw)
    r_y_pitch = create_r_y_beta(pitch)
    r_x_roll = create_r_x_gamma(roll)
    tmp = np.matmul(r_z_yaw, r_y_pitch)
    ret_m = np.matmul(tmp, r_x_roll)
    return ret_m


if __name__ == "__main__":

    # roll = np.radians(143)
    # pitch = np.radians(-361)
    # yaw = np.radians(12.32)
    #
    # rot_m = create_rot_m(roll, pitch, yaw)
    # print(rot_m)
    # rpy = RQDecomp3x3(rot_m)
    # print(rpy[0])

    vec1 = np.array([0.87880082, 1.35488877, -7.11622651])
    vec2 = np.array([-1.18758871, -0.7849843, -7.29187168])
    delta = vec2 - vec1
    print(delta)
    print(norm(delta))
