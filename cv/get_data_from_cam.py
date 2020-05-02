import cv2
from marker_detection import PoseMeasurer
import serial
from time import sleep


if __name__ == '__main__':
    ser = serial.Serial(
        port='/dev/ttyUSB0',
        baudrate=9600
    )
    PoseMeasurer = PoseMeasurer('CamCalib.json', 0)
    cv2.namedWindow("Camera", cv2.WINDOW_AUTOSIZE)

    height_img = 640
    width_img = 480
    angle = 15
    ser.write((str(angle) + '\n').encode())

    while True:
        frame, corners, ids, rvecs, tvecs = PoseMeasurer.get_pose_markers()
        new_frame = frame.copy()
        if rvecs is not None and tvecs is not None:
            try:
                frame = PoseMeasurer.draw_markers(frame, corners, ids, rvecs, tvecs)
                if cv2.waitKey(1) & 0xFF == ord(' '):
                    print("Captured with angle " + str(angle))
                    cv2.imwrite('./images/' + str(angle) + '.jpg', new_frame)
                    angle += 5
                    ser.write((str(angle) + '\n').encode())



            except Exception as ex:
                print(ex)

        cv2.imshow("Camera", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cv2.destroyWindow("Camera")
