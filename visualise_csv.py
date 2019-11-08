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
    data = pd.read_csv('data/f/fanuc_1_square_low.csv', dtype=float)
    med_x = data['a_x'][910:2735].median()
    med_y = data['a_y'][910:2735].median()

    mean_x = data['a_x'][910:2735].mean()
    mean_y = data['a_y'][910:2735].mean()

    min_x = data['a_x'][910:2735].min()
    min_y = data['a_y'][910:2735].min()

    a_x = data['a_x'][910:2735]
    a_y = data['a_y'][910:2735]
    t = data['t'][910:2735]

    # threshold_filter(a_x, min_x/10)
    # threshold_filter(a_y, min_y/10)

    kernel_size = 509
    a_x_filt = wiener(a_x, kernel_size, noise=0.01)
    a_y_filt = medfilt(a_x, kernel_size)

    plt.subplot(221)
    plt.plot(t, a_x, 'b')
    plt.plot(t, a_x_filt, 'r')

    plt.subplot(222)
    plt.plot(t, a_x, 'b')
    plt.plot(t, a_y_filt, 'r')

    # u_x = [a_x_filt[0] * t[0]]
    # for i in range(1, len(t)):
    #     u_x.append(u_x[i-1] + a_x_filt[i]*t[i])
    #
    # u_y = [a_y_filt[0] * t[0]]
    # for i in range(1, len(t)):
    #     u_y.append(u_y[i - 1] + a_y_filt[i] * t[i])
    #
    # plt.subplot(223)
    # plt.plot(t, u_x, 'b')
    # plt.plot(t, u_y, 'r')
    #
    # s_x = [u_x[0] * t[0]]
    # for i in range(1, len(t)):
    #     s_x.append(s_x[i - 1] + u_x[i] * t[i])
    #
    # s_y = [u_y[0] * t[0]]
    # for i in range(1, len(t)):
    #     s_y.append(s_y[i - 1] + u_y[i] * t[i])
    #
    # plt.subplot(224)
    # plt.plot(t, s_x, 'b')
    # plt.plot(t, s_y, 'r')

    plt.show()
