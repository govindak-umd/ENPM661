# Importing all the libraries

import numpy as np
import copy
import math
import heapq
import time
import matplotlib.pyplot as plt
import cv2
import pygame


# wheel dia = 76mm
# full robot dia = 354 mm
# distance between the wheels = 317.5mm
# clearance =  can be given by the user = 5 mm
# for solving the path using eucledian heuristic

def euclediandistance(a, b):
    x1 = a[0]
    x2 = b[0]

    y1 = a[1]
    y2 = b[1]

    dist = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    return dist


# inding neighbours using the non-holonomic drive conditions
# fucntion finds the immediete neighbours

def findneighbours(X0, Y0, Theta0, UL, UR):

    t = 0
    r = 0.1
    L = 1

    dt = 0.1
    X1 = 0
    Y1 = 0
    dtheta = 0
    Theta0 = 3.14 * Theta0 / 180
    Theta1 = Theta0
    while t < 1:
        t = t + dt
        X0 = X0 + X1
        Y0 = Y0 + Y1
        dx = r * (UL + UR) * math.cos(Theta1) * dt
        dy = r * (UL + UR) * math.sin(Theta1) * dt
        dtheta = (r / L) * (UR - UL) * dt
        X1 = X1 + dx
        Y1 = Y1 + dy
        Theta1 = Theta1 + 0.5 * dtheta
        Xn = X0 + X1
        Yn = Y0 + Y1
        Thetan = 180 * (Theta1) / 3.14
    return Xn, Yn, Thetan


# function to move
def actionmove(curr_node, degree, step_size=1.0):
    x = curr_node[0]
    y = curr_node[1]
    x_new = step_size * np.cos(np.deg2rad(degree)) + x  # - (y-0)*np.sin(np.deg2rad(degree)) + length
    y_new = step_size * np.sin(np.deg2rad(degree)) + y  # + (y-0)*np.cos(np.deg2rad(degree)) + length
    new_node = (round(x_new, 2), round(y_new, 2))
    if 0.00 <= new_node[0] <= 1020.00 and 0.00 <= new_node[1] <= 1020.00:
        return new_node, True
    else:
        return curr_node, False


#        POINTS FOR DRAWING THE GRAPH
# all possible points in integer format

all_possible_int_points = []

# points to draw the map, without considering the radius and clearance of the robot. This works with
# different sets of points as compared to the points created

map_points = []
# SCALING all by 100 times
for i in range(0, 1020):  # 1020 width
    for j in range(0, 1020):  # 1020 width
        all_possible_int_points.append((i, j))  # appending

for pt in all_possible_int_points:
    x = pt[0]
    y = pt[1]
    # circle shaped obstacles
    # circle shaped obstacle at the center of the image
    # for path traversal
    if (x - 510) ** 2 + (y - 510) ** 2 <= 100 ** 2:
        map_points.append((x, y))

    # circle shaped obstacle on top right
    # for path traversal
    if (x - 710) ** 2 + (y - 810) ** 2 <= 100 ** 2:
        map_points.append((x, y))

    # circle shaped obstacle on bottom right
    # for path traversal
    if (x - 710) ** 2 + (y - 210) ** 2 <= 100 ** 2:
        map_points.append((x, y))

    # circle shaped obstacle on bottom left
    # for path traversal
    if (x - 310) ** 2 + (y - 210) ** 2 <= 100 ** 2:
        map_points.append((x, y))

    # borders

    # left vertical border
    if 0 <= x <= 10:
        if 0 <= y <= 1020:
            map_points.append((x, y))

    # right vertical border
    if 1010 <= x <= 1020:
        if 0 <= y <= 1020:
            map_points.append((x, y))
    # bottom horizontal border
    if 10 < x < 1010:
        if 0 <= y <= 10:
            map_points.append((x, y))
    # top horizontal border
    if 10 < x < 1010:
        if 1010 <= y <= 1020:
            map_points.append((x, y))

    # squares

    # left square
    if 35 <= x <= 185:
        if 435 <= y <= 585:
            map_points.append((x, y))

    # right square
    if 835 <= x <= 985:
        if 435 <= y <= 585:
            map_points.append((x, y))

    # top left square
    if 235 <= x <= 385:
        if 735 <= y <= 885:
            map_points.append((x, y))
# defining a blank canvas
new_canvas = np.zeros((1020, 1020, 3), np.uint8)

# for every point that belongs within the obstacle
for c in map_points:  # change the name of the variable l
    x = c[1]
    y = c[0]
    new_canvas[(x, y)] = [20, 125, 150]  # assigning a yellow coloured pixel

# flipping the image for correct orientation
new_canvas = np.flipud(new_canvas)
# showing the obstacle map
cv2.imshow('new_canvas', new_canvas)
cv2.waitKey(0)
cv2.destroyAllWindows()
