import cv2
from cv.cam_calib import *
import csv
from vpython import *


def get_marker(id):
    marker_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    return cv2.aruco.drawMarker(marker_dict, id, 200)


def get_markers_files(num_of_markers):
    for id in range(num_of_markers):  # 16 pieces, for ex
        marker = get_marker(id)
        cv2.imwrite(str(id) + '.jpg', marker)


def write_3_vectors(vec_x, vec_y, vec_z, vec_time, file_name):
    with open(file_name, 'w', newline='') as csvfile:
        datawriter = csv.writer(csvfile)
        datawriter.writerow(['s_x', 's_y', 's_z', 't'])
        for i in range(len(vec_x)):
            datawriter.writerow([vec_x[i], vec_y[i], vec_z[i], vec_time[i]])


def get_rotation_from_vecs(rvecs):
    rotM = np.zeros(shape=(3, 3))
    cv2.Rodrigues(rvecs[0][0], rotM, jacobian=0)
    ypr = cv2.RQDecomp3x3(rotM)
    return ypr[0], rotM


def euler_2_cartesian(roll, pitch, yaw):
    x = cos(yaw) * cos(pitch)
    y = sin(yaw) * cos(pitch)
    z = sin(pitch)
    return x, y, z


def move_arr(arr):
    size = len(arr)
    for i in range(size - 1):
        arr[i] = arr[i + 1]


def mv_avg_filt(arr, new_elem):
    move_arr(arr)
    arr[-1] = new_elem
    return arr.mean()