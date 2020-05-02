import cv2
from cv.cam_calib import *
import csv
import time
from vpython import *
from test_vis import create_scene
from work_with_angles import create_rot_m


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
    def __init__(self, calib_file_path, num_camera=0):
        self.cam_calib_w_i = CameraCalibrationWI((9, 6))
        self.cam_calib_w_i.load_camera_calibration(calib_file_path)
        self.marker_dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.camera = cv2.VideoCapture(num_camera)
        self.marker_side_length = 0.07  # in meters

    def get_pose_markers(self):
        _, frame = self.camera.read()
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(frame, self.marker_dictionary)
        rvecs, tvecs = None, None
        if not ids is None:
            rvecs, tvecs, _objPoints = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_side_length,
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
    # print(ypr[0])
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


if __name__ == '__main__':
    PoseMeasurer = PoseMeasurer('CamCalib.json', 0)
    cv2.namedWindow("Camera", cv2.WINDOW_AUTOSIZE)

    scene = create_scene()
    # ar = arrow(pos=vector(1, 1, 1), axis=vector(0, 0, 0), shaftwidth=0.5)
    # scene.camera.follow(ar)

    ball = sphere(pos=vector(0, 0, 0), radius=0.5)
    window_size = 10
    x_arr = np.zeros(window_size)
    y_arr = np.zeros(window_size)
    z_arr = np.zeros(window_size)

    x_file, y_file, z_file, t = [0 for i in range(100)], [0 for i in range(100)], [0 for i in range(100)], []
    start_time = time.time()
    x_curve = gcurve(color=color.red)  # a graphics curve
    y_curve = gcurve(color=color.green)
    z_curve = gcurve(color=color.blue)

    height_img = 640
    width_img = 480
    while True:
        frame, corners, ids, rvecs, tvecs = PoseMeasurer.get_pose_markers()
        if rvecs is not None and tvecs is not None:
            try:
                frame = PoseMeasurer.draw_markers(frame, corners, ids, rvecs, tvecs)
                cv2.circle(frame, (int(height_img / 2), int(width_img / 2)), 5, (0, 0, 255))

                tmp, rot_mat = get_rotation_from_vecs(rvecs)
                roll, pitch, yaw = radians(tmp[0]), radians(tmp[1]), radians(tmp[2])
                x, y, z = euler_2_cartesian(roll, pitch, yaw)

                x = mv_avg_filt(x_arr, x)
                y = mv_avg_filt(y_arr, y)
                z = mv_avg_filt(z_arr, z)

                # ar.pos = 5 * vector(z, -y, -x)
                # ar.axis = 5 * vector(-z, y, x)
                # coord = rot_mat * np.array([[PoseMeasurer.marker_side_length, 0, 0]]).T + tvecs[0]
                # coord_xyz = coord[0]

                translate_vec = tvecs[0]
                coord = np.matmul(rot_mat, translate_vec.T)
                print(translate_vec)
                ball.pos = vector(coord[0], coord[1], coord[2])

                x_file.pop(0)
                x_file.append(coord[0])

                y_file.pop(0)
                y_file.append(coord[1])

                z_file.pop(0)
                z_file.append(coord[2])

                t.append(time.time() - start_time)

                x_curve.delete()
                y_curve.delete()
                z_curve.delete()

                for i in range(len(x_file)):
                    x_curve.plot(i, float(x_file[i]))
                    y_curve.plot(i, float(y_file[i]))
                    # z_curve.plot(i, z_file[i])
                # 0.87880082  1.35488877    -7.11622651
                # -1.18758871 -0.7849843  -7.29187168



            except Exception as ex:
                print(ex)

        cv2.imshow("Camera", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cv2.destroyWindow("Camera")
    # with open('../data/up.csv', 'w', newline='') as csvfile:
    #     datawriter = csv.writer(csvfile)
    #     datawriter.writerow(['x', 'y', 'z', 't'])
    #     for i in range(len(x_file)):
    #         datawriter.writerow([x_file[i], y_file[i], z_file[i], t[i]])
