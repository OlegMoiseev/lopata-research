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
    data = pd.read_csv('data/up.csv', index_col=False, dtype=float)

    s_x = data['x']
    s_y = data['y']
    s_z = data['z']
    t = data['t']

    kernel_size = 9
    s_x_filt = medfilt(s_x, kernel_size)
    s_y_filt = medfilt(s_y, kernel_size)
    s_z_filt = medfilt(s_z, kernel_size)

    fig = plt.figure(dpi=300)
    plt.subplot(231)
    plt.title("Coordinate X")
    plt.plot(t, s_x, 'r')

    plt.subplot(232)
    plt.title("Coordinate Y")
    plt.plot(t, s_y, 'r')

    plt.subplot(233)
    plt.title("Coordinate Z")
    plt.plot(t, s_z, 'r')

    plt.subplot(234)
    plt.title("Coordinate X, Y")
    plt.plot(t, s_x, 'r')
    plt.plot(t, s_y, 'g')

    f = fig.add_subplot(235, projection='3d')
    f.set_zlim3d(0, 8)
    # f.set_xlim3d(-0.1, 0.5)
    # f.set_ylim3d(-0.1, 0.5)

    plt.plot(xs=s_x, ys=s_y, zs=s_z)

    plt.show()
