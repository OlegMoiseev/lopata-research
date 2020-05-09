from cv.marker_detection import PoseMeasurerFromCam
import cv2
import numpy as np
from help_functions import get_rotation_from_vecs
from scipy.spatial.transform import Rotation as R


class Detector:
    def __init__(self, marker_positions: dict, calib_file_path, marker_side_length=0.07, camera_num=0):
        self.__measurer = PoseMeasurerFromCam(calib_file_path, marker_side_length, camera_num)
        self.__marker_positions = marker_positions

    def get_my_pose_from_all(self, frame):
        corners, ids, rvecs, tvecs = self.__measurer.get_pose_markers(frame)
        my_pose = []
        for id, tvec in zip(ids, tvecs):
            marker_pose = self.__marker_positions[id[0]]
            # print("____________")
            # print(id[0])
            # print(tvec)
            # print(marker_pose)
            # print("____________")
            my_pose.append(tvec - marker_pose)
        return my_pose

    def get_poses_from_cam(self):
        corners, ids, rvecs, tvecs, cam_frame = self.__measurer.get_pose_markers_from_cam()
        my_pose = {}
        if ids is not None:
            for id, tvec, rvec in zip(ids, tvecs, rvecs):
                id, tvec, rvec = id[0], tvec[0], rvec[0]
                marker_pose = self.__marker_positions[id]

                r = R.from_rotvec(rvec)
                yaw = r.as_euler('xyz')[2]

                rot_mat = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                                    [np.sin(yaw), np.cos(yaw), 0],
                                    [0, 0, 1]])
                my_pose[id] = np.matmul(tvec, rot_mat) + marker_pose


        cam_frame = self.__measurer.draw_markers(cam_frame, corners, ids, rvecs, tvecs)
        cv2.imshow("frame", cam_frame)
        return my_pose


if __name__ == '__main__':
    # TODO: get poses from file
    markers_poses = {3: [0, 0, 0],
                     8: [0.3, 0.25, 0]}

    zero_marker_num = 3
    Detector = Detector(markers_poses, "camera_calibration/CamCalib.json")
    while True:
        try:
            my_poses = Detector.get_poses_from_cam()

            print("________________")
            for key in sorted(my_poses):
                print(key, my_poses[key])

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        except:
            pass
