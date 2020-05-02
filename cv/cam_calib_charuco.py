import numpy as np
import cv2.cv2 as cv2
from cv2 import aruco
import glob
import json


class CameraCalibration:
    def __init__(self, board_dimensional):
        self.board_size = board_dimensional
        self.camera_matrix = None
        self.distortion_coefficients = None

    def get_board_corners(self, images):
        stop_criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
                         50, 0.001)
        undistort_corners_coordinates = []
        distort_corners_coordinates = []
        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
        objp = np.zeros((self.board_size[0] * self.board_size[1], 3), np.float32)
        # this is some kind of magic...
        objp[:, :2] = np.mgrid[0:self.board_size[0], 0:self.board_size[1]].T.reshape(-1, 2)
        for image in images:
            found, corners = cv2.findChessboardCorners(image, self.board_size)
            if found:
                distort_corners_coordinates.append(objp)
                cv2.cornerSubPix(image, corners, (11, 11), (-1, -1), stop_criteria)
                undistort_corners_coordinates.append(corners)
        return distort_corners_coordinates, undistort_corners_coordinates

    def calculate_intrinsic_parameters(self, calibration_images):
        chessboard_undistorted_points, chessboard_distorted_points = self.get_board_corners(calibration_images)
        success, self.camera_matrix, self.distortion_coefficients, \
        rotation_vectors, translation_vectors = \
            cv2.calibrateCamera(chessboard_undistorted_points,  # calibrated image - object points
                                chessboard_distorted_points,  # raw image - image points
                                calibration_images[0].shape[::-1],
                                None, None)

        mean_error = 0
        for i in range(len(chessboard_undistorted_points)):
            imgpoints2, _ = cv2.projectPoints(chessboard_undistorted_points[i], rotation_vectors[i],
                                              translation_vectors[i], self.camera_matrix, self.distortion_coefficients)
            error = cv2.norm(chessboard_distorted_points[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
            mean_error += error
        print("Total error: ", mean_error / len(chessboard_undistorted_points))


class CameraCalibrationWI(CameraCalibration):
    def __init__(self, board_dimensional):
        super().__init__(board_dimensional)

    def save_camera_calibration(self, path_save):
        with open(path_save, 'w') as calib_file:
            data = {'mat_rows': self.camera_matrix.shape[0],
                    'mat_cols': self.camera_matrix.shape[1]}

            mat = []
            for i in range(data['mat_rows']):
                for j in range(data['mat_cols']):
                    mat.append(self.camera_matrix[i, j])
            data['mat'] = mat

            data['distortion_rows'] = self.distortion_coefficients.shape[0]
            data['distortion_cols'] = self.distortion_coefficients.shape[1]

            dist = []
            for i in range(data['distortion_rows']):
                for j in range(data['distortion_cols']):
                    dist.append(self.distortion_coefficients[i, j])
            data['distortion'] = dist

            json.dump(data, calib_file, indent="")

    def load_camera_calibration(self, path_calibration):
        with open(path_calibration, 'r') as calib_file:
            data = json.load(calib_file)

            self.camera_matrix = np.zeros((data['mat_rows'], data['mat_cols']))
            for i in range(data['mat_rows']):
                for j in range(data['mat_cols']):
                    self.camera_matrix[i, j] = data['mat'][data['mat_cols'] * i + j]

            self.distortion_coefficients = np.zeros((data['distortion_rows'], data['distortion_cols']))
            for i in range(data['distortion_rows']):
                for j in range(data['distortion_cols']):
                    self.distortion_coefficients[i, j] = data['distortion'][data['distortion_cols'] * i + j]

    def calculate_blurriness(self, frame):
        return cv2.Laplacian(frame, cv2.CV_64F).var()

    def focus_setting(self, camera):
        while True:
            ret, frame = camera.read()
            blurriness = self.calculate_blurriness(frame)
            cv2.putText(frame, "{:12.4f}".format(blurriness), (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (150, 150, 150),
                        lineType=cv2.LINE_AA)
            cv2.imshow("Focus setting", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        cv2.destroyWindow("Focus setting")

    def camera_calibration_images_from_folder(self, folder_name, count_of_images):
        calibration_images = []
        images = glob.glob(folder_name + '*.jpg')
        for i in range(count_of_images):
            img = cv2.imread(images[i])
            calibration_images.append(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY))
        self.calculate_intrinsic_parameters(calibration_images)
        self.save_camera_calibration("CamCalib.json")

    def camera_calibration_process(self, camera, count_of_frames):
        cv2.namedWindow("Camera", cv2.WINDOW_AUTOSIZE)
        count_of_good_frames = 0
        saved_images = []
        while True:
            ret, frame = camera.read()
            frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            found, found_corners = cv2.findChessboardCorners(frame_gray, self.board_size,
                                                             cv2.CALIB_CB_ADAPTIVE_THRESH +
                                                             cv2.CALIB_CB_NORMALIZE_IMAGE +
                                                             cv2.CALIB_CB_FILTER_QUADS +
                                                             cv2.CALIB_CB_FAST_CHECK)
            if found:
                frame_to_draw = frame.copy()
                cv2.drawChessboardCorners(frame_to_draw, self.board_size, found_corners, found)

                cv2.imshow("Camera", frame_to_draw)
                if cv2.waitKey(1) & 0xFF == ord(' '):
                    count_of_good_frames += 1
                    temp = frame_gray.copy()
                    saved_images.append(temp)
                    print(str(count_of_good_frames) + '/' + str(count_of_frames))
                    if len(saved_images) == count_of_frames:
                        print('Start saving images...')
                        for i in range(len(saved_images)):
                            cv2.imwrite('images/' + str(i) + '.jpg', saved_images[i])

                        print('Start calibration...')
                        self.calculate_intrinsic_parameters(saved_images)

                        print('Saving calibration parameters...')
                        self.save_camera_calibration("CamCalib.json")

                        print("Successfully saved!")
                        break

            else:
                cv2.imshow("Camera", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
        cv2.destroyWindow("Camera")

    def undistort(self, image):
        if self.distortion_coefficients is None or self.camera_matrix is None:
            return image
        return cv2.undistort(image, self.camera_matrix, self.distortion_coefficients)


def create_ch_board():
    squares_x = 7
    squares_y = 5
    square_len = 1
    marker_len = .8
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    ch_board = aruco.CharucoBoard_create(squares_x, squares_y, square_len, marker_len, aruco_dict)
    return ch_board, aruco_dict


def save_camera_calibration(cam_mat, dist_coef, path_save):
    with open(path_save, 'w') as calib_file:
        data = {'mat_rows': cam_mat.shape[0],
                'mat_cols': cam_mat.shape[1]}

        mat = []
        for i in range(data['mat_rows']):
            for j in range(data['mat_cols']):
                mat.append(cam_mat[i, j])
        data['mat'] = mat

        data['distortion_rows'] = dist_coef.shape[0]
        data['distortion_cols'] = dist_coef.shape[1]

        dist = []
        for i in range(data['distortion_rows']):
            for j in range(data['distortion_cols']):
                dist.append(dist_coef[i, j])
        data['distortion'] = dist

        json.dump(data, calib_file, indent="")

    # cam_calib_w_i = CameraCalibrationWI((9, 6))
    # # cam_calib_w_i.camera_calibration_images_from_folder('images/', 10)
    #
    # # cam_calib_w_i.load_camera_calibration('CamCalib.json')
    # cam_calib_w_i.camera_calibration_process(video_capture, 10)
    # # for i in range(10):
    # #     name_raw = 'images/' + str(i) + '.jpg'
    # #     name_undistort = 'images/' + str(i) + '_1.jpg'
    # #     img = cv2.imread(name_raw)
    # #     img_undistort = cam_calib_w_i.undistort(img)
    # #     cv2.imwrite(name_undistort, img_undistort)


def calc_coeffs():
    board, dict = create_ch_board()

    # 0 - камера на ноуте, >=1 - камера подключенная
    flag = 0
    video_capture = cv2.VideoCapture(flag)

    allCorners = []
    allIds = []

    count_of_frames = 3
    count_of_good_frames = 0
    saved_images = []

    while True:

        ret, frame = video_capture.read()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        res = cv2.aruco.detectMarkers(gray, dict)

        if len(res[0]) > 0:
            cv2.aruco.drawDetectedMarkers(frame, res[0], res[1])
            cv2.imshow('frame', frame)
            if cv2.waitKey(1) & 0xFF == ord(' '):
                count_of_good_frames += 1

                res2 = cv2.aruco.interpolateCornersCharuco(res[0], res[1], gray, board)
                if res2[1] is not None and res2[2] is not None and len(res2[1]) > 3:
                    allCorners.append(res2[1])
                    allIds.append(res2[2])
                    print(str(count_of_good_frames) + '/' + str(count_of_frames))
                    saved_images.append(frame)

                if count_of_good_frames == count_of_frames:
                    break
        else:
            cv2.imshow('frame', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Calibration fails for lots of reasons. Release the video if we do
    try:
        print('Start saving images...')
        for i in range(len(saved_images)):
            cv2.imwrite('images/' + str(i) + '.jpg', saved_images[i])

        print('Start calibration...')
        imsize = gray.shape
        cal = cv2.aruco.calibrateCameraCharuco(allCorners, allIds, board, imsize, None, None)
        cam_matrix = cal[1]
        dist_coeffs = cal[2]

        print('Saving calibration parameters...')
        save_camera_calibration(cam_matrix, dist_coeffs, "CamCalib.json")

        print("Successfully saved!")

    except:
        video_capture.release()

    video_capture.release()
    cv2.destroyAllWindows()


def load_camera_calibration(path_calibration):
    with open(path_calibration, 'r') as calib_file:
        data = json.load(calib_file)

        camera_matrix = np.zeros((data['mat_rows'], data['mat_cols']))
        for i in range(data['mat_rows']):
            for j in range(data['mat_cols']):
                camera_matrix[i, j] = data['mat'][data['mat_cols'] * i + j]

        distortion_coefficients = np.zeros((data['distortion_rows'], data['distortion_cols']))
        for i in range(data['distortion_rows']):
            for j in range(data['distortion_cols']):
                distortion_coefficients[i, j] = data['distortion'][data['distortion_cols'] * i + j]

        return camera_matrix, distortion_coefficients


if __name__ == '__main__':
    calc_coeffs()
    # for i in range(3):
    #     name_raw = 'images/' + str(i) + '.jpg'
    #     name_undistort = 'images/' + str(i) + '_1.jpg'
    #     img = cv2.imread(name_raw)
    #     camera_matrix, distortion_coefficients = load_camera_calibration("CamCalib.json")
    #     img_undistort = cv2.undistort(img, camera_matrix, distortion_coefficients)
    #
    #     cv2.imwrite(name_undistort, img_undistort)



