import pandas as pd
import matplotlib.pyplot as plt
from pykalman import KalmanFilter
from scipy.signal import medfilt, wiener
from mpl_toolkits.mplot3d import Axes3D


def threshold_filter(data, threshold):
    if threshold < 0:
        threshold *= -1
    for i in range(len(data)):
        if -threshold < data[i] < threshold:
            data[i] = 0


if __name__ == '__main__':
    data = pd.read_csv('data/hand_camera/data_random.csv', index_col=False, dtype=float)

    s_x = data['s_x']
    s_y = data['s_y']
    s_z = data['s_z']
    t = data['t']

    kernel_size = 9
    s_x_filt = medfilt(s_x, kernel_size)
    s_y_filt = medfilt(s_y, kernel_size)
    s_z_filt = medfilt(s_z, kernel_size)

    fig = plt.figure(dpi=300)
    plt.subplot(231)
    plt.title("Coordinate X")
    plt.plot(t, s_x_filt, 'r')

    plt.subplot(232)
    plt.title("Coordinate Y")
    plt.plot(t, s_y_filt, 'r')

    plt.subplot(233)
    plt.title("Coordinate Z")
    plt.plot(t, s_z_filt, 'r')

    plt.subplot(234)
    plt.title("Coordinate X, Y")
    plt.plot(t, s_x_filt, 'r')
    plt.plot(t, s_y_filt, 'g')

    f = fig.add_subplot(235, projection='3d')
    f.set_zlim3d(0, 0.5)

    plt.plot(xs=s_x_filt, ys=s_y_filt, zs=s_z_filt)

    plt.show()
