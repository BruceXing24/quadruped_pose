import matplotlib.pyplot as plt
from . import pose_calculate as pose

# initial coordinate
B1 = (0.19, 0.111, 0)
B2 = (0.19, -0.111, 0)
B3 = (-0.19, 0.111, 0)
B4 = (-0.19, -0.111, 0)

angle = 0
fig = plt.figure()
ax = plt.axes(projection='3d')
a = 0.1


def show(r, p, y, ax):
    matrix_ABs = pose.get_AB(r, p, y).T  # get every leg AB coordinate

    x1, y1, z1 = matrix_ABs[0, 0], matrix_ABs[0, 1], matrix_ABs[0, 2]
    A1 = (B1[0] - x1, B1[1] - y1, B1[2] - z1)
    ax.plot([B1[0], A1[0]], [B1[1], A1[1]], [B1[2], A1[2]], c='m', linewidth=2)

    x2, y2, z2 = matrix_ABs[1, 0], matrix_ABs[1, 1], matrix_ABs[1, 2]
    A2 = (B2[0] - x2, B2[1] - y2, B2[2] - z2)
    ax.plot([B2[0], A2[0]], [B2[1], A2[1]], [B2[2], A2[2]], c='m', linewidth=2)

    x3, y3, z3 = matrix_ABs[2, 0], matrix_ABs[2, 1], matrix_ABs[2, 2]  # theta6, theta4,theta5
    A3 = (B3[0] - x3, B3[1] - y3, B3[2] - z3)
    ax.plot([B3[0], A3[0]], [B3[1], A3[1]], [B3[2], A3[2]], c='m', linewidth=2)

    x4, y4, z4 = matrix_ABs[3, 0], matrix_ABs[3, 1], matrix_ABs[3, 2]
    A4 = (B4[0] - x4, B4[1] - y4, B4[2] - z4)
    ax.plot([B4[0], A4[0]], [B4[1], A4[1]], [B4[2], A4[2]], c='m', linewidth=2)

    # draw robot body the head is in red
    ax.plot([A1[0], A2[0]], [A1[1], A2[1]], [A1[2], A2[2]], c='r', linewidth=3)
    ax.plot([A2[0], A4[0]], [A2[1], A4[1]], [A2[2], A4[2]], c='b', linewidth=3)
    ax.plot([A3[0], A4[0]], [A3[1], A4[1]], [A3[2], A4[2]], c='b', linewidth=3)
    ax.plot([A3[0], A1[0]], [A3[1], A1[1]], [A3[2], A1[2]], c='b', linewidth=3)

    plt.pause(0.05)
    plt.ioff()
    plt.cla()


if __name__ == "__main__":

    # test 3d graph
    fig = plt.figure()
    ax = plt.axes(projection='3d')
    while True:
        if angle >= 15:
            a = -0.3
        if angle <= -15:
            a = 0.3
        angle += a
        show(0, angle, 0, ax)
