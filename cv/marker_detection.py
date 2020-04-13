import cv2
from cv.cam_calib import *
import csv
import time
from vpython import *
from test_vis import create_scene


def get_marker(id):
    marker_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    return cv2.aruco.drawMarker(marker_dict, id, 200)


def write_3_vectors(vec_x, vec_y, vec_z, vec_time, file_name):
    with open(file_name, 'w', newline='') as csvfile:
        datawriter = csv.writer(csvfile)
        datawriter.writerow(['s_x', 's_y', 's_z', 't'])
        for i in range(len(vec_x)):
            datawriter.writerow([vec_x[i], vec_y[i], vec_z[i], vec_time[i]])


def get_markers_files(num_of_markers):
    for id in range(num_of_markers):  # 16 pieces, for ex
        marker = get_marker(id)
        cv2.imwrite(str(id) + '.jpg', marker)


class PoseMeasurer:
    def __init__(self, calib_file_path, num_camera=2):
        self.cam_calib_w_i = CameraCalibrationWI((9, 6))
        self.cam_calib_w_i.load_camera_calibration(calib_file_path)
        self.marker_dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.camera = cv2.VideoCapture(num_camera)

    def get_pose_markers(self):
        _, frame = self.camera.read()
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(frame, self.marker_dictionary)
        rvecs, tvecs = None, None
        if not ids is None:
            rvecs, tvecs, _objPoints = cv2.aruco.estimatePoseSingleMarkers(corners, 0.05,
                                                                           self.cam_calib_w_i.camera_matrix,
                                                                           self.cam_calib_w_i.distortion_coefficients)
        return frame, corners, ids, rvecs, tvecs

    def draw_markers(self, frame, corners, ids, rvecs, tvecs):
        frame = cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        frame = cv2.aruco.drawAxis(frame, self.cam_calib_w_i.camera_matrix, self.cam_calib_w_i.distortion_coefficients,
                                   rvecs, tvecs, 0.05)
        return frame


def get_rotation_from_vecs(rvecs):
    rotM = np.zeros(shape=(3, 3))
    cv2.Rodrigues(rvecs[0][0], rotM, jacobian=0)
    ypr = cv2.RQDecomp3x3(rotM)
    return ypr[0]


def euler_2_cartesian(roll, pitch, yaw):
    x = cos(yaw) * cos(pitch)
    y = sin(yaw) * cos(pitch)
    z = sin(pitch)
    return x, y, z


if __name__ == '__main__':
    PoseMeasurer = PoseMeasurer('CamCalib.json', 2)
    cv2.namedWindow("Camera", cv2.WINDOW_AUTOSIZE)

    create_scene()
    ar = arrow(pos=vector(1, 1, 1), axis=vector(0, 0, 0), shaftwidth=0.1)

    while True:
        frame, corners, ids, rvecs, tvecs = PoseMeasurer.get_pose_markers()
        if rvecs is not None and tvecs is not None:
            frame = PoseMeasurer.draw_markers(frame, corners, ids, rvecs, tvecs)
            tmp = get_rotation_from_vecs(rvecs)
            roll, pitch, yaw = radians(tmp[2]), radians(tmp[1]), radians(tmp[0])
            x, y, z = euler_2_cartesian(roll, pitch, yaw)

            ar.pos = vector(z, -y, -x)
            ar.axis = vector(-z, y, x)

        cv2.imshow("Camera", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cv2.destroyWindow("Camera")
