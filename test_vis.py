from vpython import *


def create_scene():
    scene2 = canvas(x=0, y=0, forward=vector(-1, -1, -1), background=color.white)

    x_axis = arrow(canvas=scene2, pos=vector(0, 0, 0), axis=vector(10, 0, 0), shaftwidth=0.1, color=color.red)
    y_axis = arrow(canvas=scene2, pos=vector(0, 0, 0), axis=vector(0, 10, 0), shaftwidth=0.1, color=color.green)
    z_axis = arrow(canvas=scene2, pos=vector(0, 0, 0), axis=vector(0, 0, 10), shaftwidth=0.1, color=color.blue)

    return scene2


if __name__ == '__main__':
    x0 = 1
    y0 = 1
    z0 = 1

    pointer = arrow(pos=vector(0, 0, 0), axis=vector(x0, y0, z0), shaftwidth=0.1)
    roll = 0
    pitch = 0
    yaw = 0

    for i in range(360):
        yaw = i
        yaw = radians(yaw)
        x = cos(yaw) * cos(pitch)
        y = sin(yaw) * cos(pitch)
        z = sin(pitch)
        pointer.axis = vector(x, y, z)
        sleep(0.1)
