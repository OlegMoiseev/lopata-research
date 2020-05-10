from cv.marker_detection import PoseMeasurerFromCam
import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R
import csv


class Detector:
    def __init__(self, marker_positions: dict, calib_file_path, marker_side_length=0.07, camera_num=0):
        self.__measurer = PoseMeasurerFromCam(calib_file_path, marker_side_length, camera_num)
        self.__marker_positions = marker_positions

    def get_my_pose(self, ids, rvs, tvs):
        my_pose = {}
        if ids is not None:
            for id, tvec, rvec in zip(ids, tvs, rvs):
                id, tvec, rvec = id[0], tvec[0], rvec[0]
                marker_pose = self.__marker_positions[id]

                r = R.from_rotvec(rvec)
                yaw = r.as_euler('xyz')[2]

                rot_mat = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                                    [np.sin(yaw), np.cos(yaw), 0],
                                    [0, 0, 1]])
                my_pose[id] = np.matmul(tvec, rot_mat) + marker_pose
        return my_pose

    def get_my_pose_from_frame(self, frame):
        corners, ids, rvecs, tvecs = self.__measurer.get_pose_markers(frame)

        my_pose = self.get_my_pose(ids, rvecs, tvecs)
        frame = self.__measurer.draw_markers(frame, corners, ids, rvecs, tvecs)
        cv2.imshow("frame", frame)

        return my_pose

    def get_poses_from_cam(self):
        corners, ids, rvecs, tvecs, cam_frame = self.__measurer.get_pose_markers_from_cam()

        my_pose = self.get_my_pose(ids, rvecs, tvecs)
        cam_frame = self.__measurer.draw_markers(cam_frame, corners, ids, rvecs, tvecs)
        cv2.imshow("Camera frame", cam_frame)
        return my_pose


if __name__ == '__main__':
    # TODO: get poses from file
    markers_poses = {3: [0, 0, 0],
                     5: [0.15, 0.1, 0],
                     8: [0.3, 0.25, 0]}
    zero_marker_num = 3
    Detector = Detector(markers_poses, "camera_calibration/CamCalib.json")

    x_file, y_file, z_file = [], [], []
    while True:
        try:
            my_poses = Detector.get_poses_from_cam()
            xs, ys, zs = [], [], []

            print("________________")
            for key in sorted(my_poses):
                # print(key, my_poses[key])
                xs.append(my_poses[key][0])
                ys.append(my_poses[key][1])
                zs.append(my_poses[key][2])

            x_file.append(np.mean(xs))
            y_file.append(np.mean(ys))
            z_file.append(np.mean(zs))

            actual_pose = [x_file[-1], y_file[-1], z_file[-1]]
            print(actual_pose)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        except:
            pass

    with open('../data/hand_around/data_around.csv', 'w', newline='') as csvfile:
        datawriter = csv.writer(csvfile)
        datawriter.writerow(['x', 'y', 'z'])
        for i in range(len(x_file)):
            datawriter.writerow([x_file[i], y_file[i], z_file[i]])