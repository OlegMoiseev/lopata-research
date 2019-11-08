import numpy as np
import cv2.cv2 as cv2
import glob
import json


class CameraCalibration:
    def __init__(self, board_dimensional):
        self.board_size = board_dimensional

    def get_board_corners(self, images):
        stop_criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
                         50, 0.001)
        img_corners = []
        world_corners = []
        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
        objp = np.zeros((self.board_size[0] * self.board_size[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:self.board_size[0],
                      0:self.board_size[1]].T.reshape(-1, 2)
        for image in images:
            found, corners = cv2.findChessboardCorners(image, self.board_size)
            if found:
                world_corners.append(objp)
                cv2.cornerSubPix(image, corners, (11, 11), (-1, -1), stop_criteria)
                img_corners.append(corners)
        return world_corners, img_corners

    def calculate_intrinsic_parameters(self, calibration_images):
        chessboard_world_space_points, chessboard_image_space_points = self.get_board_corners(calibration_images)
        success, self.camera_matrix, self.distortion_coefficients, rvec, tvec = \
            cv2.calibrateCamera(chessboard_world_space_points,
                                chessboard_image_space_points,
                                calibration_images[0].shape[::-1],
                                None, None)


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

    def load_camera_calibration(self):
        pass

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
            calibration_images.append(cv2.cvtColor(images[i], cv2.COLOR_BGR2GRAY))
        self.calculate_intrinsic_parameters(calibration_images)

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
                        print('Started calibration...')
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


# 0 - камера на ноуте, 1 - камера подключенная
flag = 2
video_capture = cv2.VideoCapture(flag)

cam_calib_w_i = CameraCalibrationWI((9, 6))
cam_calib_w_i.camera_calibration_process(video_capture, 5)

# # Check success
# if not video_capture.isOpened():
#     raise Exception("Could not open video device")
#
# # prepare object points
# nx = 9  # number of inside corners in x
# ny = 6  # number of inside corners in y
#
# # termination criteria
# criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
#
# # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
# objp = np.zeros((nx*ny, 3), np.float32)
# objp[:, :2] = np.mgrid[0:nx, 0:ny].T.reshape(-1, 2)
#
# # Arrays to store object points and image points from all the images.
# objpoints = []      # 3d point in real world space
# imgpoints = []      # 2d points in image plane.
# path_calib = '/ph/calib/'
# path_raw = '/ph/raw/'
#
# count = 0
# ret2 = False
# while True:
#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         break
#     ret1, frame = video_capture.read()
#     img = cv2.cvtColor(frame.copy(), cv2.COLOR_BGR2GRAY)
#     if ret2:
#         if cv2.waitKey(1) & 0xFF == ord('f'):
#             cv2.imwrite(path_raw + "frame" + str(count) + ".jpg", frame)
#             count += 1
#             print('taken frame')
#     else:
#         if cv2.waitKey(1) & 0xFF == ord('f'):
#             print("i don't see chessboard")
#         ret2, corners = cv2.findChessboardCorners(img, (nx, ny), None)
#
#     fm = cv2.Laplacian(frame, cv2.CV_64F).var()
#     frame = cv2.drawChessboardCorners(frame, (7, 6), corners, True)
#
#     cv2.putText(frame, "{:12.4f}".format(fm), (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (150, 150, 150), lineType=cv2.LINE_AA)
#     cv2.imshow('frame', frame)
#
# images = glob.glob(path_raw + '*.jpg')
#
# print('started watch images')
# count = 0
# for fname in images:
#     print("next image")
#     img = cv2.imread(fname)
#     gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
#
#     ret, corners = cv2.findChessboardCorners(gray, (nx, ny), None)
#     # If found, add object points, image points (after refining them)
#     if ret:
#         print('i see')
#         objpoints.append(objp)
#         corners2 = cv2.cornerSubPix(gray, corners, (19, 19), (-1, -1), criteria)
#         imgpoints.append(corners2)
#         # img = cv2.drawChessboardCorners(img, (nx, ny), corners2, ret)
#         # cv2.imshow('img', img)
#
# print('i had saw all images')
# ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
#
# # writing coef into coef.txt
# # (описание коэффициентов - добавить - не будь Тимуром)
# file = open("coef.txt", "w+")
# strr = "3\n3\n"
# for i in range(len(mtx[0])):
#     for j in range(len(mtx)):
#         strr += str(mtx[i, j]) + "\n"
# strr += str(len(dist[0])) + "\n"
# for i in range(len(dist[0])):
#     strr += str(dist[0, i]) + "\n"
# file.write(strr)
# file.close()
#
# h, w = img.shape[:2]
# newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
#
# count = 0
# for fname in images:
#     dst = cv2.undistort(cv2.imread(fname), mtx, dist, None, newcameramtx)
#     x, y, w, h = roi
#     dst = dst[y:y + h, x:x + w]
#     cv2.imwrite(path_calib + "calib" + str(count) + ".jpg", dst)
#     count += 1
#
# while True:
#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         break
#     ret, frame = video_capture.read()
#     # undistortion
#     dst = cv2.undistort(frame, mtx, dist, None, newcameramtx)
#     # mapx, mapy = cv2.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (w, h), 5)
#     # dst = cv2.remap(img, mapx, mapy, cv2.INTER_LINEAR)
#     # crop the image
#     x, y, w, h = roi
#     dst = dst[y:y + h, x:x + w]
#     cv2.imshow('frame', dst)
#
# # Close device
# video_capture.release()
# cv2.destroyAllWindows()
#
