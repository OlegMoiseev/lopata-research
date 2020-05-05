import cv2
from cv.cam_calib import *
from vpython import *
from test_vis import create_scene


class PoseMeasurerFromImage:
    def __init__(self, calib_file_path, marker_side_length=0.07):
        self.cam_calib_w_i = CameraCalibrationWI((9, 6))
        self.cam_calib_w_i.load_camera_calibration(calib_file_path)
        self.marker_dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.marker_side_length = marker_side_length  # in meters

    def get_pose_markers(self, cam_frame):
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(cam_frame, self.marker_dictionary)
        rvecs, tvecs = None, None
        if not ids is None:
            rvecs, tvecs, _objPoints = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_side_length,
                                                                           self.cam_calib_w_i.camera_matrix,
                                                                           self.cam_calib_w_i.distortion_coefficients)
        return corners, ids, rvecs, tvecs

    def draw_markers(self, cam_frame, corners, ids, rvecs, tvecs):
        cam_frame = cv2.aruco.drawDetectedMarkers(cam_frame, corners, ids)
        cam_frame = cv2.aruco.drawAxis(cam_frame, self.cam_calib_w_i.camera_matrix, self.cam_calib_w_i.distortion_coefficients,
                                   rvecs, tvecs, 0.05)
        return cam_frame


class PoseMeasurerFromCam(PoseMeasurerFromImage):
    def __init__(self, calib_file_path, marker_side_length=0.07, camera_num=0):
        super().__init__(calib_file_path, marker_side_length)
        self.camera = cv2.VideoCapture(camera_num)

    def get_pose_markers_from_cam(self):
        _, cam_frame = self.camera.read()
        corners, ids, rvecs, tvecs = super().get_pose_markers(cam_frame)
        return corners, ids, rvecs, tvecs, cam_frame


if __name__ == '__main__':
    PoseMeasurerFromImage = PoseMeasurerFromImage('CamCalib.json')
    scene = create_scene()

    height_img = 640
    width_img = 480

    for i in range(15, 185, 5):
        print(i)
        for j in range(10):
            file_path = 'images/around/' + str(i) + '_' + str(j) + '.jpg'
            frame = cv2.imread(file_path)
            corners, ids, rvecs, tvecs = PoseMeasurerFromImage.get_pose_markers(frame)

            coord = tvecs[0][0]
            sphere(pos=vector(coord[0], coord[1], coord[2]), radius=0.005, color=color.red)

        sleep(0.01)

    # PoseMeasurerFromCam = PoseMeasurerFromCam('CamCalib.json')
    # while True:
    #     try:
    #         corners, ids, rvecs, tvecs, image = PoseMeasurerFromCam.get_pose_markers_from_cam()
    #         if rvecs is not None and tvecs is not None:
    #             image = PoseMeasurerFromCam.draw_markers(image, corners, ids, rvecs, tvecs)
    #             print(ids)
    #
    #         cv2.imshow("Camera", image)
    #     except:
    #         pass

    #     if cv2.waitKey(1) & 0xFF == ord('q'):
    #         break
    # cv2.destroyWindow("Camera")
