import pandas as pd
import matplotlib.pyplot as plt
from pykalman import KalmanFilter
from scipy.signal import medfilt, wiener


def threshold_filter(data, threshold):
    if threshold < 0:
        threshold *= -1
    for i in range(len(data)):
        if -threshold < data[i] < threshold:
            data[i] = 0


if __name__ == '__main__':
    data = pd.read_csv('data/hand_camera/data_random.csv', index_col=False, dtype=float)

    min_x = 0.00549
    min_y = -0.00116
    a_x = data['a_x'] - min_x
    a_y = data['a_y'] - min_y
    a_z = data['a_z']
    t = data['t']

    threshold_filter(a_x, min_x)
    threshold_filter(a_y, min_y)

    # pair_x = []
    # for i in range(len(a_x)):
    #     pair_x.append([t[i], a_x[i]])

    kernel_size = 79
    a_x_filt = medfilt(a_x, kernel_size)
    a_y_filt = medfilt(a_y, kernel_size)

    plt.figure(dpi=300)
    plt.subplot(221)
    plt.title("Accel X")
    plt.plot(t, a_x, 'b')
    plt.plot(t, a_x_filt, 'r')

    plt.subplot(222)
    plt.title("Accel Y")
    plt.plot(t, a_y, 'b')
    plt.plot(t, a_y_filt, 'r')

    u_x = [a_x[0] * t[0]]
    for i in range(1, len(t)):
        u_x.append(u_x[i-1] + a_x_filt[i]*t[i])

    u_y = [a_y[0] * t[0]]
    for i in range(1, len(t)):
        u_y.append(u_y[i - 1] + a_y_filt[i] * t[i])

    plt.subplot(223)
    plt.title("Speed X, Y")
    plt.plot(t, u_x, 'b')
    plt.plot(t, u_y, 'r')

    s_x = [u_x[0] * t[0]]
    for i in range(1, len(t)):
        s_x.append(s_x[i - 1] + u_x[i] * t[i])

    s_y = [u_y[0] * t[0]]
    for i in range(1, len(t)):
        s_y.append(s_y[i - 1] + u_y[i] * t[i])

    plt.subplot(224)
    plt.title("Coordinate X, Y")
    plt.plot(t, s_x, 'b')
    plt.plot(t, s_y, 'r')

    plt.show()
