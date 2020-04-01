# Importing all the libraries

import numpy as np
import copy
import math
import heapq
import time
import matplotlib.pyplot as plt
import cv2
import pygame

# %%
# wheel dia = 76mm
# full robot dia = 354 mm
# distance between the wheels = 317.5mm
# clearance =  can be given by the user = 5 mm
# for solving the path using eucledian heuristic

# Getting the start time to measure the time taken for solving

####################User Inputs######################

print('                             WELCOME TO THE A * PROGRAMME                             ')
print('THIS PROGRAMME TAKES THE USER INPUTS BELOW AND SOLVES THE OBSTACLE SPACE VERY QUICKLY')
print('time taken (for A* solving) ~ <1 second')
print(
    'To solve the necessary test cases, please know that the start and the goal points here are adjusted based on the clearance and the width of the robot')
print('This avoids the goal or the start getting into the borders or any obstacles')
print('For the turtle bot, this value is fixed at 35')
print('If you wish to change this value, please rerun the code after doing so.')
print('ALSO !! NOTE !!, the coordinates of the obstacle points have been multiplied by a factor of 100')
print(
    'This ensures a clearer graph. Hence, please multiply by a factor of 100, before input. For example: 5.5 will be = 5.5*100 = 550')
print('Range of x coordinates you can enter  =  0 - 1020')
print('Range of y coordinates you can enter  =  0 - 1020')
x_start, y_start, start_orientation = input("Enter the x and y coordinate of the start and the initial orientation here, with a blank in between them (suggested : 10 10 30) : > ").split()
x_goal, y_goal =  input("Enter the x and y coordinate of the goal and the , in the form of blanks (suggested : 1010 1010) : > ").split()
RPM_L, RPM_R = input("Enter the LEFT WHEEL RPM AND RIGHT WHEEL RPM  : > ").split()
clearance = int(input("Enter the clearance of the robot with the nearby obstacles : (suggested value  < 5) : > "))
step_size = int(input("Enter the step (1-10): (suggested values  = 1) : > "))
radius = 0.5
start = (int(x_start) + radius + clearance, int(y_start) + radius + clearance)
goal = (int(x_goal) - radius - clearance, int(y_goal) - radius - clearance)
print('Wheel radius is calculated to be 76mm/2 -> approximating to 0.5')
time_run = int(input(('Please enter the time you wish to choose to run the robot ((suggested  value = 1 second) : ')))
start_orientation = int(start_orientation)
RPM_L = int(RPM_L)
RPM_R = int(RPM_R)
print('Please wait while the map is being generated. Approximate wait time < 5 seconds . ')
# %%
# function to round the point 5

def Round2Point5(num):
    return (round(num * 2) / 2)


def RoundToFifteen(x, base=15):
    return base * round(x / base)


# for solving the path using eucledian heuristic

def EucledianDistance(a, b):
    x1 = a[0]
    x2 = b[0]
    y1 = a[1]
    y2 = b[1]
    dist = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
    dist = Round2Point5(dist)
    return dist


def plot_curve(X0, Y0, Theta0, UL, UR):
    global time_run
    t = 0
    r = 0.5
    L = 3.5
    dt = 0.1
    X1 = 0
    Y1 = 0
    dtheta = 0
    Theta0 = 3.14 * Theta0 / 180
    Theta1 = Theta0
    while t < time_run:
        t = t + dt
        X0 = X0 + X1
        Y0 = Y0 + Y1

        dx = r * (UL + UR) * math.cos(Theta1) * dt
        dy = r * (UL + UR) * math.sin(Theta1) * dt
        dtheta = (r / L) * (UR - UL) * dt

        X1 = X1 + dx
        Y1 = Y1 + dy
        Theta1 = Theta1 + 0.5 * dtheta

        plt.quiver(X0, Y0, X1, Y1, units='xy', scale=1, color='r', width=1, headwidth=1, headlength=0)

        Xn = X0 + X1
        Yn = Y0 + Y1
        Thetan = 180 * (Theta1) / 3.14
    return Xn, Yn, Thetan


# can use manhattan or eucledian heuristic as for caclulation of the value of cost to go to the goal.

#####

# function to move

# Possible motions :

# [0, RPM1]
# [RPM1, 0]
# [RPM1, RPM1]
# [0, RPM2]
# [RPM2, 0]
# [RPM2, RPM2]
# [RPM1, RPM2]
# [RPM2, RPM1]

# inding neighbours using the non-holonomic drive conditions
# fucntion finds the immediete neighbour to which the robot drives to

def ActionMove(curr_node, orientation_facing, RPM_L, RPM_R, d=1.0, L=3.5, step_size=1.0):
    global time_run
    curr_start_x = curr_node[0]
    curr_start_y = curr_node[1]

    r = d / 2
    # radius = dia / 2
    t = 0

    dt = 0.1
    X1 = 0
    Y1 = 0
    dtheta = 0
    # x_new, y_new, new_orientation = GoTo(x,y,degree,RPM_L,RPM_R)

    orientation_facing_RADIANS = 3.14 * orientation_facing / 180

    while t < time_run:
        t = t + dt
        curr_start_x = curr_start_x + X1
        curr_start_y = curr_start_y + Y1
        dx = r * (RPM_L + RPM_R) * math.cos(orientation_facing_RADIANS) * dt
        dy = r * (RPM_L + RPM_R) * math.sin(orientation_facing_RADIANS) * dt
        dtheta = (r / L) * (RPM_R - RPM_L) * dt
        X1 = X1 + dx
        Y1 = Y1 + dy
        orientation_facing_RADIANS = orientation_facing_RADIANS + 0.5 * dtheta
        new_x = curr_start_x + X1
        new_y = curr_start_y + Y1
        new_final_orientation = 180 * (orientation_facing_RADIANS) / 3.14
    # calculating the degrees rotated   from the start to the new orientation and assigning costs accordingly
    degrees_rotated = abs(orientation_facing - new_final_orientation)
    degrees_rotated = abs(degrees_rotated % 360)
    cost = 10 + (10 * degrees_rotated / 360)
    #     new_final_orientation = round(new_final_orientation,2)
    new_final_orientation = RoundToFifteen(new_final_orientation)
    new_node = ((round(new_x, 2), round(new_y, 2)), new_final_orientation, cost)
    same_out = (curr_node, orientation_facing, 100000)
    if 0.00 <= new_node[0][0] <= 1020.00 and 0.00 <= new_node[0][1] <= 1020.00:
        return new_node, True
    else:
        return same_out, False


# %%
######################################     POINTS FOR REMOVING VALUES FROM THE GRAPH

# calculating all the points within the entire canvas

# all possible points in integer format

all_possible_int_points = []

# points to draw the map, without considering the radius and clearance of the robot. This works with
# different sets of points as compared to the points created

untraversable_points = []
# SCALING all by 100 times
for i in range(0, 1021):  # 1020 width
    for j in range(0, 1021):  # 1020 width
        all_possible_int_points.append((i, j))  # appending

for pt in all_possible_int_points:
    x = pt[0]
    y = pt[1]
    # circle shaped obstacles
    # circle shaped obstacle at the center of the image
    # for path traversal
    if (x - 510) ** 2 + (y - 510) ** 2 <= (100 + radius + clearance) ** 2:
        untraversable_points.append((x, y))

    # circle shaped obstacle on top right
    # for path traversal
    if (x - 710) ** 2 + (y - 810) ** 2 <= (100 + radius + clearance) ** 2:
        untraversable_points.append((x, y))

    # circle shaped obstacle on bottom right
    # for path traversal
    if (x - 710) ** 2 + (y - 210) ** 2 <= (100 + radius + clearance) ** 2:
        untraversable_points.append((x, y))

    # circle shaped obstacle on bottom left
    # for path traversal
    if (x - 310) ** 2 + (y - 210) ** 2 <= (100 + radius + clearance) ** 2:
        untraversable_points.append((x, y))

    # borders

    # left vertical border
    if 0 < x < 10 + radius + clearance:
        if 0 < y < 1020:
            untraversable_points.append((x, y))

    # right vertical border
    if 1010 - radius - clearance < x < 1020:
        if 0 < y < 1020:
            untraversable_points.append((x, y))
    # bottom horizontal border
    if 10 < x < 1010:
        if 0 < y < 10 + radius + clearance:
            untraversable_points.append((x, y))
    # top horizontal border
    if 10 < x < 1010:
        if 1010 - radius - clearance < y < 1020:
            untraversable_points.append((x, y))

    # squares

    # left square
    if 35 - radius - clearance <= x <= 185 + radius + clearance:
        if 435 - radius - clearance <= y <= 585 + radius + clearance:
            untraversable_points.append((x, y))

    # right square
    if 835 - radius - clearance <= x <= 985 + radius + clearance:
        if 435 - radius - clearance <= y <= 585 + radius + clearance:
            untraversable_points.append((x, y))

    # top left square
    if 235 - radius - clearance <= x <= 385 + radius + clearance:
        if 735 - radius - clearance <= y <= 885 + radius + clearance:
            untraversable_points.append((x, y))

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
    if 0 < x < 10:
        if 0 < y < 1020:
            map_points.append((x, y))

    # right vertical border
    if 1010 < x < 1020:
        if 0 < y < 1020:
            map_points.append((x, y))
    # bottom horizontal border
    if 10 < x < 1010:
        if 0 < y < 10:
            map_points.append((x, y))
    # top horizontal border
    if 10 < x < 1010:
        if 1010 < y < 1020:
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

cv2.imshow(' < OBSTACLE SPACE MAP > ', new_canvas)
cv2.waitKey(0)
cv2.destroyAllWindows()

print('Please wait while your solution is being calculated . ')
print('Time taken on a high end Personal Computer ~ < 1 second')
print('This will generate a map with solution, backtracked path, quiver plot and an animation too! ')
print('!!! NOTE: Quiver plot will take an additional 10 seconds to calculate ')
##################function to check if any point is within the obstacle space#################
# including the region covered by the robot radius and its clearance

# %%
def checkObstaclespace(point):
    global radius
    global clearance
    x = point[0]
    y = point[1]
    # circle shaped obstacles
    # circle shaped obstacle at the center of the image
    # for path traversal
    if (x - 510) ** 2 + (y - 510) ** 2 <= (100 + radius + clearance) ** 2:
        #         print('THIS POINT IS INSIDE : > circle shaped obstacle at the center')
        return False

    # circle shaped obstacle on top right
    # for path traversal
    elif (x - 710) ** 2 + (y - 810) ** 2 <= (100 + radius + clearance) ** 2:
        #         print('THIS POINT IS INSIDE : > circle shaped obstacle on top right')
        return False

    # circle shaped obstacle on bottom right
    # for path traversal
    elif (x - 710) ** 2 + (y - 210) ** 2 <= (100 + radius + clearance) ** 2:
        #         print('THIS POINT IS INSIDE : > circle shaped obstacle on bottom right')
        return False

    # circle shaped obstacle on bottom left
    # for path traversal
    elif (x - 310) ** 2 + (y - 210) ** 2 <= (100 + radius + clearance) ** 2:
        #         print('THIS POINT IS INSIDE : > circle shaped obstacle on bottom right')
        return False

    # borders

    # left vertical border
    elif 0 < x < 10 + radius + clearance:
        #         print('THIS POINT IS INSIDE : > left vertical border')
        if 0 < y < 1020:
            return False
    # right vertical border
    elif 1010 - radius - clearance < x < 1020:
        #         print('THIS POINT IS INSIDE : > right vertical border')
        if 0 < y < 1020:
            return False
    # bottom horizontal border
    elif 10 < x < 1010 and 0 < y < 10 + radius + clearance:
        if i <= size_x and j <= size_y and i >= 0 and j >= 0:
            #             print('THIS POINT IS INSIDE : > bottom horizontal border')
            return False
    # top horizontal border
    elif 10 < x < 1010 and 1010 - radius - clearance < y < 1020:
        if 735 <= y <= 885:
            #             print('THIS POINT IS INSIDE : > top horizontal border')
            return False

    # squares

    # left square
    elif 35 - radius - clearance <= x <= 185 + radius + clearance and 435 - radius - clearance <= y <= 585 + radius + clearance:
        #             print('THIS POINT IS INSIDE : > left square')
        return False

    # right square
    elif 835 - radius - clearance <= x <= 985 + radius + clearance and 435 - radius - clearance <= y <= 585 + radius + clearance:
        #             print('THIS POINT IS INSIDE : > right square')
        return False
    # top left square
    elif 235 - radius - clearance <= x <= 385 + radius + clearance and 735 - radius - clearance <= y <= 885 + radius + clearance:
        #             print('THIS POINT IS INSIDE : > top left square')
        return False

    else:

        return True


# %%
# showing what every child node of each parent nodes are connected to
list_of_points_for_graph = []
size_x, size_y = 1021, 1021


def generateGraph(point, degree,
                  step_size=1):  # remember that this size_x and size_y are the sizes of the matrix, so not the end coordinates

    global list_of_points_for_graph
    global size_x
    global size_y
    global RPM_R
    global RPM_L

    i = point[0]  # x coordinate
    j = point[1]  # y coordinate

    if i <= size_x and j <= size_y and i >= 0 and j >= 0:

        all_neighbours = {}

        pos1 = ActionMove(point, degree, 0, RPM_L)[0]
        pos2 = ActionMove(point, degree, RPM_L, 0)[0]
        pos3 = ActionMove(point, degree, RPM_L, RPM_L)[0]
        pos4 = ActionMove(point, degree, 0, RPM_R)[0]
        pos5 = ActionMove(point, degree, RPM_R, 0)[0]
        pos6 = ActionMove(point, degree, RPM_R, RPM_R)[0]
        pos7 = ActionMove(point, degree, RPM_L, RPM_R)[0]
        pos8 = ActionMove(point, degree, RPM_R, RPM_L)[0]

        if pos1[0][0] >= 0 and pos1[0][1] >= 0 and pos1[0][0] <= size_x and pos1[0][1] <= size_y:
            all_neighbours[pos1[0]] = (round(pos1[2], 2), pos1[1])

        if pos2[0][0] >= 0 and pos2[0][1] >= 0 and pos2[0][0] <= size_x and pos2[0][1] <= size_y:
            all_neighbours[pos2[0]] = (round(pos2[2], 2), pos2[1])

        if pos3[0][0] >= 0 and pos3[0][1] >= 0 and pos3[0][0] <= size_x and pos3[0][1] <= size_y:
            all_neighbours[pos3[0]] = (round(pos3[2], 2), pos3[1])

        if pos4[0][0] >= 0 and pos4[0][1] >= 0 and pos4[0][0] <= size_x and pos4[0][1] <= size_y:
            all_neighbours[pos1[0]] = (round(pos4[2], 2), pos4[1])

        if pos5[0][0] >= 0 and pos5[0][1] >= 0 and pos5[0][0] <= size_x and pos5[0][1] <= size_y:
            all_neighbours[pos5[0]] = (round(pos5[2], 2), pos5[1])

        if pos6[0][0] >= 0 and pos6[0][1] >= 0 and pos6[0][0] <= size_x and pos6[0][1] <= size_y:
            all_neighbours[pos6[0]] = (round(pos6[2], 2), pos6[1])

        if pos7[0][0] >= 0 and pos7[0][1] >= 0 and pos7[0][0] <= size_x and pos7[0][1] <= size_y:
            all_neighbours[pos7[0]] = (round(pos7[2], 2), pos7[1])

        if pos8[0][0] >= 0 and pos8[0][1] >= 0 and pos8[0][0] <= size_x and pos8[0][1] <= size_y:
            all_neighbours[pos8[0]] = (round(pos8[2], 2), pos8[1])

        return all_neighbours

    else:

        pass


# %%
# function to backtrack so that the path from the goal to the end is obtained

point_and_angle_list = []


def BackTrack(backtrack_dict, goal, start):  # goal is the starting point now and start is the goal point now

    # initializing the backtracked list
    back_track_list = []
    # appending the start variable to the back_track_list list
    back_track_list.append(start)
    # while the goal is not found

    while goal != []:
        # for key and values in the backtracking dictionary
        for k, v in backtracking_dict.items():

            # for the key and values in the values, v
            for k2, v2 in v.items():
                # checking if the first key is the start
                if k == start:

                    # checking if not in the backtrackedlist

                    if v2[0] not in back_track_list:
                        back_track_list.append(start)
                        point_and_angle_list.append((start, v2[1]))
                    # updating the start variable
                    start = v2[0]

                    # checking if it is the goal
                    if v2[0] == goal:
                        goal = []
                        break
                        # returns the backtracked list
    return (back_track_list)


######### < < < < A* algorithm code > > > > #########

# %%
# empty dictionary for backtracking from child to parent upto the start
backtracking = {}
# list of all the visited nodes
visited = set()
global orientation_to_layer
# Mapping of Angles to layers
orientation_to_layer = {0: 0, 15: 1, 30: 2, 45: 3, 60: 4, 75: 5, 90: 6, 105: 7, 120: 8, 135: 9, 150: 10, 165: 11,
                        180: 12,
                        195: 13, 210: 14, 225: 15, 240: 16, 255: 17, 270: 18, 285: 19, 300: 20, 315: 21, 330: 22,
                        345: 23, 360: 24}
# array to store cost from
# creating a 200rows, by 300 column by 12 layers for the various costs and to check if a node is visited
cost_from = np.array(np.ones((2041, 2041, 24)) * np.inf)
# Initializing visited nodes as empty array
V = np.zeros((2041, 2041, 24))
# array to store Heuristic distance
heur = np.array(np.ones((2041, 2041)) * np.inf)
# array to store total cost f
f = np.array(np.ones((2041, 2041, 24)) * np.inf)
# list for Explored nodes
priority_queue = []
# append start point,start orientation of the bot and initialize it's cost to zero
heapq.heappush(priority_queue, (0, start, start_orientation))
# initialize cost  for start node to zero
cost_from[int(2 * start[0])][int(2 * start[1])][orientation_to_layer[start_orientation]] = 0
f[int(2 * start[0])][int(2 * start[1])][orientation_to_layer[start_orientation]] = 0


######### < < < < A* algorithm code > > > > #########

def a_star_Algorithm(start, goal):
    global step_size
    global untraversable_points
    global V
    break_while = 0

    # create a dictionary for backtracked parents
    backtracking = {}

    # check if goal/start in obstacle space

    if goal in untraversable_points or start in untraversable_points:

        print('!!!!!!!!!!GOAL/START IS INSIDE OBSTACLE SPACE!!!!!!!!!!')
        print(
            'Please check if the radius and clearance of the robot was \n taken into consideration and reassign again.')
        backtracking = 0
        rounded_neighbour = 0

        return False

    else:
        while True:
            if break_while == 1:
                break
            _, curr_vert, curr_orient = heapq.heappop(priority_queue)
            # append visited nodes
            visited.add(curr_vert)
            # checking if the neighbour is the goal. If goal found reached
            if ((curr_vert[0] - goal[0]) ** 2 + (curr_vert[1] - goal[1]) ** 2 <= (1.5) ** 2):
                print('^^^^^^^^^^^^^^')
                print('^^^^^^^^^^^^^^')
                print('GOAL REACHED')
                print('^^^^^^^^^^^^^^')
                print('^^^^^^^^^^^^^^')
                print(curr_vert)
                break
            # check whether node is in the obstacle space
            if checkObstaclespace(curr_vert) == True:
                # generate neighbours
                graph = generateGraph(curr_vert, curr_orient)
                graph_list = []
                # put neighbours in a list for easy access
                for key, cost_value in graph.items():
                    graph_list.append((key, cost_value))
                for neighbour, cost in graph_list:
                    this_cost = graph[neighbour][0]
                    breakflag = 0
                    orientation = graph[neighbour][1]
                    orientation = orientation % 360
                    # Round the node
                    rounded_neighbour = (Round2Point5(neighbour[0]), Round2Point5(neighbour[1]))
                    #                     print("cost for this"+str(rounded_neighbour)+str(this_cost))
                    if rounded_neighbour in visited:
                        breakflag = 1
                        # exit if found
                    if breakflag == 1:
                        continue
                    # check if this neighbour is goal
                    if ((rounded_neighbour[0] - goal[0]) ** 2 + (rounded_neighbour[1] - goal[1]) ** 2 <= (1.5) ** 2):
                        print('^^^^^^^^^^^^^^')
                        print('^^^^^^^^^^^^^^')
                        print('GOAL REACHED')
                        print('^^^^^^^^^^^^^^')
                        print('^^^^^^^^^^^^^^')
                        break_while = 1
                        break
                    ##check whether current neighbour node is in the obstacle space
                    if checkObstaclespace(rounded_neighbour) == True:
                        # check if visited
                        if V[int(2 * rounded_neighbour[0])][int(2 * rounded_neighbour[1])][
                            orientation_to_layer[orientation]] == 0:
                            # if not, make it visited
                            V[int(2 * rounded_neighbour[0])][int(2 * rounded_neighbour[1])][
                                orientation_to_layer[orientation]] = 1
                            # calculate cost from
                            cost_from[int(2 * rounded_neighbour[0])][int(2 * rounded_neighbour[1])][
                                orientation_to_layer[int(orientation)]] = (
                                    this_cost + cost_from[int(2 * curr_vert[0])][int(2 * curr_vert[1])][
                                orientation_to_layer[curr_orient]])
                            # calculate cost to go values
                            heur[int(2 * rounded_neighbour[0])][int(2 * rounded_neighbour[1])] = EucledianDistance(
                                rounded_neighbour, goal)
                            # calculate f = g+h
                            f[int(2 * rounded_neighbour[0])][int(2 * rounded_neighbour[1])][
                                orientation_to_layer[orientation]] = \
                                cost_from[int(2 * rounded_neighbour[0])][int(2 * rounded_neighbour[1])][
                                    orientation_to_layer[orientation]] + heur[int(2 * rounded_neighbour[0])][
                                    int(2 * rounded_neighbour[1])]
                            # push to the explored node queue
                            heapq.heappush(priority_queue, (
                                f[int(2 * rounded_neighbour[0])][int(2 * rounded_neighbour[1])][
                                    orientation_to_layer[orientation]], rounded_neighbour, orientation))
                            backtracking[rounded_neighbour] = {}
                            # adding to the backtracking dictionary
                            backtracking[rounded_neighbour][
                                f[int(2 * rounded_neighbour[0])][int(2 * rounded_neighbour[1])][
                                    orientation_to_layer[orientation]]] = (curr_vert, curr_orient)
                        else:
                            # if visited, check cost. if newly genrated neighbour has a lower cost, update it
                            if (f[int(2 * rounded_neighbour[0])][int(2 * rounded_neighbour[1])][
                                orientation_to_layer[orientation]]) > (
                                    f[int(2 * curr_vert[0])][int(2 * curr_vert[1])][orientation_to_layer[curr_orient]]):
                                f[int(2 * rounded_neighbour[0])][int(2 * rounded_neighbour[1])][
                                    orientation_to_layer[orientation]] = (
                                    f[int(2 * curr_vert[0])][int(2 * curr_vert[1])][orientation_to_layer[curr_orient]])
                                backtracking[rounded_neighbour][
                                    f[int(2 * rounded_neighbour[0])][int(2 * rounded_neighbour[1])][
                                        orientation_to_layer[orientation]]] = (curr_vert, curr_orient)

    return (curr_vert, backtracking)
    # return the last parent node and backtracked dictionary


start_time = time.time()
# calling the function to solve the obstacle space using A* Algorithm
new_goal_rounded, backtracking_dict = a_star_Algorithm(start, goal)
print("SOLVED in =", time.time() - start_time)

# Backtracking back to the goal
backtracked_final = BackTrack(backtracking_dict, start, new_goal_rounded)
# printing the nodes from initial to the end
print(backtracked_final)
# %%

# Scale the visited nodes and backtracked nodes for easy visualization
new_list_visited = []
for visited_node in list(visited):
    new_x = visited_node[0] * 2
    new_y = visited_node[1] * 2
    new_list_visited.append((new_x, new_y))

new_backtracked = []
for back in backtracked_final:
    new_x_b = back[0] * 2
    new_y_b = back[1] * 2
    new_backtracked.append((new_x_b, new_y_b))

# #defining a blank canvas
new_canvas = np.zeros((2040, 2040, 3), np.uint8)

# for every point that belongs within the obstacle
for c in map_points:  # change the name of the variable l
    x = 2 * c[1]
    y = 2 * c[0]
    new_canvas[(x, y)] = [255, 255, 255]  # assigning a yellow coloured pixel

# flipping the image for correct orientation
new_canvas = np.flipud(new_canvas)
# making a copy for backtracking purpose
new_canvas_copy_backtrack = new_canvas.copy()
# making a copy for showing the visited nodes on the obstacle space
# can be used for the animation
new_canvas_copy_visited = new_canvas.copy()

# visited path
for visit_path in new_list_visited:
    # print(path)
    x = int(visit_path[0])
    y = int(visit_path[1])
    cv2.circle(new_canvas_copy_visited, (x, 2040 - y), 5, [0, 255, 255], -1)
#     new_canvas_copy_visited[(2040-y,x)]=[255,255,255] #setting every backtracked pixel to white

# showing the final backtracked path
new_visited = cv2.resize(new_canvas_copy_visited, (600, 400))
# showing the image
new_canvas_copy_visited = cv2.resize(new_canvas_copy_visited, (1020, 1020))
cv2.imshow('visited', new_canvas_copy_visited)
# saving the image
# cv2.imwrite('visited_img.jpg',new_visited)
cv2.waitKey(0)
cv2.destroyAllWindows()

# new_backtracked = g
# backtracked path
for path in new_backtracked:
    # print(path)
    x = int(path[0])
    y = int(path[1])
    cv2.circle(new_canvas_copy_backtrack, (x, 2040 - y), 5, [0, 0, 255], -1)
#     new_canvas_copy_backtrack[(2040-y,x)]=[255,255,255] #setting every backtracked pixel to white
# showing the final backtracked path
# new_backtracked = cv2.resize(new_canvas_copy_backtrack,(600,400))

new_canvas_copy_backtrack = cv2.resize(new_canvas_copy_backtrack, (1020, 1020))
# showing the image
cv2.imshow('backtracked image', new_canvas_copy_backtrack)
# saving the image
# cv2.imwrite('backtracked_img.jpg',new_backtracked)
cv2.waitKey(0)
cv2.destroyAllWindows()

# final quiver plot

fig, ax = plt.subplots()
RPM1 = RPM_L
RPM2 = RPM_R

actions = [[0, RPM1],
           [RPM1, 0],
           [RPM1, RPM1],
           [0, RPM2],
           [RPM2, 0],
           [RPM2, RPM2],
           [RPM1, RPM2],
           [RPM2, RPM1]]

count = 0
for point in point_and_angle_list:
    x = point[0][0]
    y = point[0][1]
    theta = point[1]
    for action in actions:
        X1 = plot_curve(x, y, theta, action[0], action[1])
    count += 1
plt.grid()
ax.set_aspect('equal')
plt.title('Vector - Path', fontsize=10)
plt.show()
