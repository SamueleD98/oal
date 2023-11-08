import numpy as np
from numpy import linalg as LA
from math import *
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from random import random

OBS_TIME = 15
step = pi / 250


def main():
    # TS
    Origin = np.array([-10.991820188409513, 5.2933816172342105, 0])
    Speed = 0.5
    Heading = 2.6927937030769655
    # Origin = np.array([random() * 10, random() * 10, 0])
    # Speed = 4
    # Heading = pi * random() * 2
    Velocity = [Speed * cos(Heading), Speed * sin(Heading)]
    Direction = np.array([Velocity[0], Velocity[1], 1])
    print(Heading, Speed)
    # OS
    OS_Speed = 1

    ax = SetUpPlot(Origin, OS_Speed, Direction)
    test = sqrt(0.5*0.5+1)/(0.5*0.5+1)
    print("test", test, "_", test*test )

    points = getIntersectionTime_new(Origin, Direction, OS_Speed)
    if points:
        ax = PlotPath(ax, np.array([0, 0, 0]), points[0], 'new 1')
        ax = PlotPath(ax, np.array([0, 0, 0]), points[1], 'new 2')
        # ax.set_zlim(min(points)-10, max(points)+10)

        ax.set_zlim(np.min(np.concatenate(points)) - 10, np.max(np.concatenate(points)) + 10)

    # print("old 2: ")
    # points = getIntersectionTime_old2(Origin, Heading, Speed, OS_Speed)
    # ax = PlotPath(ax, np.array([0, 0, 0]), points[0], 'path3_1')
    # ax = PlotPath(ax, np.array([0, 0, 0]), points[1], 'path3_2')

    # Add a legend
    ax.legend()

    # Show the plot
    plt.show()


def getIntersectionTime_old(Origin, Heading, Speed, OS_Speed):
    print("old: ")
    v1 = OS_Speed
    v2 = Speed
    h2 = Heading
    H = Origin[1]
    L = Origin[0]

    k = atan2(-H, L)
    asin_arg = v2 / v1 * sin(h2 + k)
    theta1 = asin(asin_arg) - k
    theta2 = pi - k - asin(asin_arg) - k
    theta2 = theta2 % pi
    t1 = L / (v1 * cos(theta1) - v2 * cos(h2))
    t2 = L / (v1 * cos(theta2) - v2 * cos(h2))

    ipoint = [L + v2 * cos(h2) * t1, H + v2 * sin(h2) * t1, t1]
    print("Intersection at time: ", t1, " (", ipoint[0], ",", ipoint[1], ") with theta: ", theta1 / pi * 180)
    ipoint2 = [L + v2 * cos(h2) * t2, H + v2 * sin(h2) * t2, t2]
    print("Intersection at time: ", t2, " (", ipoint2[0], ",", ipoint2[1], ") with theta: ", theta2 / pi * 180)

    return [ipoint, ipoint2]


def getIntersectionTime_old2(Origin, Heading, Speed, OS_Speed):
    v1 = OS_Speed
    v2 = Speed
    h2 = Heading
    H = Origin[1]
    L = Origin[0]

    k = atan(-H / L)
    if L < 0:
        k = k + pi
    asin_arg = v2 / v1 * sin(h2 + k)
    theta1 = asin(asin_arg) - k
    theta1 = theta1 % pi
    theta2 = pi - k - asin(asin_arg) - k
    theta2 = theta2 % pi
    t1 = L / (v1 * cos(theta1) - v2 * cos(h2))
    t2 = L / (v1 * cos(theta2) - v2 * cos(h2))

    ipoint = [L + v2 * cos(h2) * t1, H + v2 * sin(h2) * t1, t1]
    print("Intersection at time: ", t1, " (", ipoint[0], ",", ipoint[1], ") with theta: ", theta1 / pi * 180)
    ipoint2 = [L + v2 * cos(h2) * t2, H + v2 * sin(h2) * t2, t2]
    print("Intersection at time: ", t2, " (", ipoint[0], ",", ipoint[1], ") with theta: ", theta2 / pi * 180)

    return [ipoint, ipoint2]


def getIntersectionTime_new(P, U, v):
    print("new: ")
    gamma = sqrt(v ** 2 + 1) / (v ** 2 + 1)
    print(v)
    print("gamma: ", gamma)
    print("gamma square: ", gamma ** 2)
    c2 = 1 - gamma ** 2 * np.dot(U, U)
    print(c2)
    print("dot p", np.dot(U, U))
    c1 = -gamma ** 2 * np.dot(U, P)
    c0 = -gamma ** 2 * np.dot(P, P)
    delta = c1 ** 2 - c0 * c2

    t = []
    if abs(c2) > 0.001 and delta >= 0:
        t.append((-c1 + sqrt(delta)) / c2)
        t.append((-c1 - sqrt(delta)) / c2)
    elif abs(c2) <= 0.001 < abs(c1):
        t.append(-c0 / (2 * c1))
    else:
        print("error")

    p = []
    if not t:
        return []
    t1 = t.pop()
    if t1 >= 0:
        p.append(P + U * t1)
    if t:
        t2 = t.pop()
        if t2 >= 0:
            p.append(P + U * t2)

    if c2 != 0:
        if delta < 0:
            print("No intersections")
        elif delta == 0:
            t = (-c1 + sqrt(delta)) / c2
            ipoint = P + U * t
            print("Intersection at time: ", t, " (", ipoint[0], ",", ipoint[1], ") with theta: ",
                  atan2(ipoint[1], ipoint[0]) / pi * 180)
            return [ipoint]
        elif delta > 0:
            t1 = (-c1 + sqrt(delta)) / c2
            t2 = (-c1 - sqrt(delta)) / c2
            ipoint = P + U * t1
            print("Intersection at time: ", t1, " (", ipoint[0], ", ", ipoint[1], ") with theta: ",
                  atan2(ipoint[1], ipoint[0]) / pi * 180)
            ipoint2 = P + U * t2
            print("Intersection at time: ", t2, " (", ipoint2[0], ", ", ipoint2[1], ") with theta: ",
                  atan2(ipoint[1], ipoint[0]) / pi * 180)
            return [ipoint, ipoint2]
    elif c2 == 0:
        if c1 != 0:
            t = -c0 / (2 * c1)
            ipoint = P + U * t
            print("Intersection at time: ", t, " (", ipoint[0], ",", ipoint[1], ") with theta: ",
                  atan2(ipoint[1], ipoint[0]) / pi * 180)
            return [ipoint]
        else:
            print(" wtf, ts and os coincide")

    return []


def SetUpPlot(TS, speed, vel_vect):
    # Create a 3D plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(TS[0], TS[1], TS[2], label='obs origin')
    ax.scatter(0, 0, 0, label='vh origin')

    ax = PlotPath(ax, TS, TS + vel_vect * OBS_TIME, 'obs path')

    line_x, line_y, line_z = zip(TS, TS - vel_vect * OBS_TIME)
    ax.plot(line_x, line_y, line_z, color='c')

    ax = ConePlot(ax, speed)

    # Set labels for the axes
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('t')
    return ax


def PlotPath(ax, p1, p2, name):
    line_x, line_y, line_z = zip(p1, p2)
    ax.plot(line_x, line_y, line_z, label=name)
    return ax


def ConePlot(ax, speed):
    theta = pi / 2 - atan2(1, speed)
    for a in range(0, int(2 * pi / step)):
        # Calculate the current angle
        angle = a * step
        p2 = np.array([0, 0, speed * OBS_TIME])
        p2 = y_rotation(p2, -theta)
        p2 = z_rotation(p2, angle)
        line_x, line_y, line_z = zip(np.array([0, 0, 0]), p2)
        ax.plot(line_x, line_y, line_z, '-r', alpha=0.2)
        line_x, line_y, line_z = zip(np.array([0, 0, 0]), -p2)
        ax.plot(line_x, line_y, line_z, '-', color='grey', alpha=0.2)
    return ax


def x_rotation(vector, theta):
    """Rotates 3-D vector around x-axis"""
    R = np.array([[1, 0, 0], [0, np.cos(theta), -np.sin(theta)], [0, np.sin(theta), np.cos(theta)]])
    return np.dot(R, vector)


def y_rotation(vector, theta):
    """Rotates 3-D vector around y-axis"""
    R = np.array([[np.cos(theta), 0, np.sin(theta)], [0, 1, 0], [-np.sin(theta), 0, np.cos(theta)]])
    return np.dot(R, vector)


def z_rotation(vector, theta):
    """Rotates 3-D vector around z-axis"""
    R = np.array([[np.cos(theta), -np.sin(theta), 0], [np.sin(theta), np.cos(theta), 0], [0, 0, 1]])
    return np.dot(R, vector)


main()
