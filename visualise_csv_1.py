import pandas as pd
import matplotlib.pyplot as plt
from test_vis import create_scene
from vpython import *


if __name__ == '__main__':
    data = pd.read_csv('data/hand_around/data_around.csv', index_col=False, dtype=float)

    x = data['x']
    y = data['y']
    z = data['z']
    create_scene()
    for i in range(len(x)):
        sphere(pos=vector(x[i], y[i], z[i]), radius=0.005, color=color.red)

    plt.show()
