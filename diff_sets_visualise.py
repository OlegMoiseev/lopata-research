import pandas as pd
import matplotlib.pyplot as plt
from sklearn.metrics import mean_squared_error
from scipy.signal import medfilt, wiener


def vis_1():
    fig, sp = plt.subplots(3)

    # for name, color in [('96_32_0', 'black'), ('96_0_0', 'g'), ('16_32_0', 'r'), ('16_0_0', 'b')]:
    # for name, color in [('160_32_0', 'r'), ('160_0_0', 'b'), ('128_0_0', 'black'), ('128_32_0', 'g')]:
    for name, color in [('160_32_225', 'r'), ('160_0_225', 'b'), ('128_0_225', 'black'), ('128_32_225', 'g')]:
        data = pd.read_csv('data/rest/' + name + '.csv', index_col='Index', dtype=float)
        a_x = data['a_x']
        a_y = data['a_y']
        a_z = data['a_z']
        t = data['t']

        sp[0].plot(t, a_x, color)

        sp[1].plot(t, a_y, color)

        sp[2].plot(t, a_z, color)
    plt.show()


def vis_2():
    fig, sp = plt.subplots(8, 2)
    index = 0
    col = 0
    color_filt = 'r'
    mse = {}

    for name, color in [('16_0_0', 'b'), ('16_0_225', 'b'),
                        ('16_32_0', 'b'), ('16_32_225', 'b'),
                        ('96_0_0', 'b'), ('96_0_225', 'b'),
                        ('96_32_0', 'b'), ('96_32_225', 'b'),
                        ('128_0_0', 'b'), ('128_0_225', 'b'),
                        ('128_32_0', 'b'), ('128_32_225', 'b'),
                        ('160_0_0', 'b'), ('160_0_225', 'b'),
                        ('160_32_0', 'b'), ('160_32_225', 'b')]:
        data = pd.read_csv('data/rest/' + name + '.csv', index_col='Index', dtype=float)
        a_x = data['a_x']
        a_x -= a_x.mean()

        t = data['t']
        sp[index, col].set_title(name)

        kernel_size = 19
        a_x_filt = medfilt(a_x, kernel_size)
        err = mean_squared_error(a_x_filt, [0. for _ in range(len(a_x_filt))])
        mse[name] = err
        sp[index, col].plot(t, a_x, color)
        sp[index, col].plot(t, a_x_filt, color_filt)

        index += 1
        if index > 7:
            index = 0
            col = 1

    for i in range(3):
        min_err = min(mse.values())
        to_pop = ''
        for key in mse:
            if mse[key] == min_err:
                print(key, mse[key])
                to_pop = key
                break
        mse.pop(to_pop)

    plt.show()


if __name__ == '__main__':
    vis_2()
