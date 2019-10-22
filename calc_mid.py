import pandas as pd


if __name__ == '__main__':
    data = pd.read_csv('data2.csv', index_col=False, dtype=float)

    a_x = data['a_x']
    a_y = data['a_y']
    a_z = data['a_z']
    t = data['t']
    print(a_x.mean(), a_y.mean(), a_z.mean())
    print(a_x.median(), a_y.median(), a_z.median())

