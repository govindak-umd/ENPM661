#!/usr/bin/env python
# coding: utf-8

# In[1]:


import numpy as np
import copy
import math
import heapq
import time
import matplotlib.pyplot as plt
x_start, y_start, start_orientation = -420.0,-300.0,30.0
x_goal, y_goal = 450.0,450.0
RPM_L, RPM_R = 1.0,1.0
radius = 1.0
clearance = 1.0
step_size = 1.0
start = (int(x_start) + radius + clearance, int(y_start) + radius + clearance)
goal = (int(x_goal) - radius - clearance, int(y_goal) - radius - clearance)
time_run = 1
start_orientation = int(start_orientation)
RPM_L = int(RPM_L)
RPM_R = int(RPM_R)


# In[2]:


def Round2Point5(num):
    return (round(num * 2.0) / 2.0)


def RoundToFifteen(x, base=15.0):
    return base * round(x / base)


# In[3]:


def EucledianDistance(a, b):
    x1 = a[0]
    x2 = b[0]
    y1 = a[1]
    y2 = b[1]
    dist = math.sqrt((x2 - x1) ** 2.0 + (y2 - y1) ** 2.0)
    dist = Round2Point5(dist)
    return dist


# In[4]:


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

        dx = (r/2) * (UL + UR) * math.cos(Theta1) * dt
        dy = (r/2) * (UL + UR) * math.sin(Theta1) * dt
        dtheta = (r / L) * (UR - UL) * dt

        X1 = X1 + dx
        Y1 = Y1 + dy
        Theta1 = Theta1 + 0.5 * dtheta

        plt.quiver(X0, Y0, X1, Y1, units='xy', scale=1, color='r', width=1, headwidth=1, headlength=0)

        Xn = X0 + X1
        Yn = Y0 + Y1
        Thetan = 180 * (Theta1) / 3.14
    return Xn, Yn, Thetan


# In[5]:


def ActionMove(curr_node, orientation_facing, RPM_L, RPM_R, d=1.0, L=3.5, step_size=1.0):
    time_run = 1
    curr_start_x = curr_node[0]
    curr_start_y = curr_node[1]
    #r = d / 2
    r = 2
    # radius = dia / 2
    t = 0

    dt = 0.1
    X1 = 0
    Y1 = 0
    dtheta = 0
    # x_new, y_new, new_orientation = GoTo(x,y,degree,RPM_L,RPM_R)

    orientation_facing_RADIANS = 3.14 * orientation_facing / 180.0

    while t < time_run:
        t = t + dt
        curr_start_x = curr_start_x + X1
        curr_start_y = curr_start_y + Y1
        dx = (r/2.0) * (RPM_L + RPM_R) * math.cos(orientation_facing_RADIANS) * dt
        dy = (r/2.0) * (RPM_L + RPM_R) * math.sin(orientation_facing_RADIANS) * dt
        dtheta = (r / L) * (RPM_R - RPM_L) * dt
        X1 = X1 + dx
        Y1 = Y1 + dy
        
        orientation_facing_RADIANS = orientation_facing_RADIANS + 0.5 * dtheta
        new_x = curr_start_x + X1
        new_y = curr_start_y + Y1
        new_final_orientation = 180 * (orientation_facing_RADIANS) / 3.14
    # calculating the degrees rotated   from the start to the new orientation and assigning costs accordingly
    degrees_rotated = abs(orientation_facing - new_final_orientation)
    degrees_rotated = abs(degrees_rotated % 360.0)
    cost = 10 + (10 * degrees_rotated / 360.0)
    #     new_final_orientation = round(new_final_orientation,2)
    new_final_orientation = RoundToFifteen(new_final_orientation)
    new_node = ((round(new_x, 2), round(new_y, 2)), new_final_orientation, cost)
    same_out = (curr_node, orientation_facing, 100000)
    if -510.00 <= new_node[0][0] <= 510.00 and -510.00 <= new_node[0][1] <= 510.00:
        return new_node, True
    else:
        return same_out, False


# In[6]:


all_possible_int_points = []

# points to draw the map, without considering the radius and clearance of the robot. This works with
# different sets of points as compared to the points created

untraversable_points = []
# SCALING all by 100 times
for i in range(-510, 511):  # 1020 width
    for j in range(-510, 511):  # 1020 width
        all_possible_int_points.append((i, j))  # appending

for pt in all_possible_int_points:
    x = pt[0]
    y = pt[1]
    # circle shaped obstacles
    # circle shaped obstacle at the center of the image
    # for path traversal
    if (x - 0) ** 2 + (y - 0) ** 2 <= (100 + radius + clearance) ** 2:
        untraversable_points.append((x, y))

    # circle shaped obstacle on top right
    # for path traversal
    if (x + 200) ** 2 + (y +300) ** 2 <= (100 + radius + clearance) ** 2:
        untraversable_points.append((x, y))

    # circle shaped obstacle on bottom right
    # for path traversal
    if (x + 200) ** 2 + (y - 300) ** 2 <= (100 + radius + clearance) ** 2:
        untraversable_points.append((x, y))

    # circle shaped obstacle on bottom left
    # for path traversal
    if (x - 200) ** 2 + (y -300) ** 2 <= (100 + radius + clearance) ** 2:
        untraversable_points.append((x, y))

    # borders

    # left vertical border
    if -510 < x < -500 + radius + clearance:
        if 0 < y < 510:
            untraversable_points.append((x, y))

    # right vertical border
    if 500 - radius - clearance < x < 510:
        if -510 < y < 510:
            untraversable_points.append((x, y))
    # bottom horizontal border
    if -510 < x < 510:
        if -510 < y < -500 + radius + clearance:
            untraversable_points.append((x, y))
    # top horizontal border
    if -510 < x < 510:
        if 500 - radius - clearance < y < 510:
            untraversable_points.append((x, y))

    # squares

    # left square
    if -475 - radius - clearance <= x <= -325 + radius + clearance:
        if -75 - radius - clearance <= y <= 75 + radius + clearance:
            untraversable_points.append((x, y))

    # right square
    if 325 - radius - clearance <= x <= 475 + radius + clearance:
        if -75 - radius - clearance <= y <= 75 + radius + clearance:
            untraversable_points.append((x, y))

    # top left square (inverted to bottom right square)
    if 125- radius - clearance <= x <= 275 + radius + clearance:
        if -375 - radius - clearance <= y <= -225 + radius + clearance:
            untraversable_points.append((x, y))


# In[7]:


def checkObstaclespace(point):
    global radius
    global clearance
    x = point[0]
    y = point[1]
    if (x - 0.0) ** 2.0 + (y - 0.0) ** 2.0 <= (100.0 + radius + clearance) ** 2.0:
        return False

    # circle shaped obstacle on top right
    # for path traversal
    elif (x + 200.0) ** 2.0 + (y +300.0) ** 2.0 <= (100.0 + radius + clearance) ** 2.0:
        return False        

    # circle shaped obstacle on bottom right
    # for path traversal
    elif (x + 200.0) ** 2.0 + (y - 300.0) ** 2.0 <= (100.0 + radius + clearance) ** 2.0:
        return False

    # circle shaped obstacle on bottom left
    # for path traversal
    elif (x - 200.0) ** 2.0 + (y -300.0) ** 2.0 <= (100.0 + radius + clearance) ** 2.0:
        return False

    # borders

    # left vertical border
    elif -510.0 < x < -500.0 + radius + clearance and  0 < y < 510.0:
        return False

    # right vertical border
    elif 500.0 - radius - clearance < x < 510.0 and  -510.0 < y < 510.0:
        
        return False
    # bottom horizontal border
    elif -510.0 < x < 510.0 and  -510.0 < y < -500.0 + radius + clearance:
        return False
    # top horizontal border
    elif -510.0 < x < 510.0 and  500.0 - radius - clearance < y < 510.0:
        
        return False

    # squares

    # left square
    elif -475.0 - radius - clearance <= x <= -325.0 + radius + clearance and  -75.0 - radius - clearance <= y <= 75.0 + radius + clearance:
        
        return False

    # right square
    elif 325.0 - radius - clearance <= x <= 475.0 + radius + clearance and  -75.0 - radius - clearance <= y <= 75.0 + radius + clearance:
        
        return False

    # top left square (inverted to bottom right square)
    elif 125.0-radius - clearance <= x <= 275.0 + radius + clearance and  -375.0 - radius - clearance <= y <= -225.0 + radius + clearance:
        
        return False


    else:

        return True


# In[8]:


def generateGraph(point, degree,
                  step_size=1):  # remember that this size_x and size_y are the sizes of the matrix, so not the end coordinates
    global RPM_R
    global RPM_L

    i = point[0]  # x coordinate
    j = point[1]  # y coordinate

    if i <= 510 and j <= 510 and i >= -510 and j >= -510:

        all_neighbours = {}

        pos1 = ActionMove(point, degree, 0, RPM_L)[0]
        pos2 = ActionMove(point, degree, RPM_L, 0)[0]
        pos3 = ActionMove(point, degree, RPM_L, RPM_L)[0]
        pos4 = ActionMove(point, degree, 0, RPM_R)[0]
        pos5 = ActionMove(point, degree, RPM_R, 0)[0]
        pos6 = ActionMove(point, degree, RPM_R, RPM_R)[0]
        pos7 = ActionMove(point, degree, RPM_L, RPM_R)[0]
        pos8 = ActionMove(point, degree, RPM_R, RPM_L)[0]

        if pos1[0][0] >= -510 and pos1[0][1] >= -510 and pos1[0][0] <= 510 and pos1[0][1] <= 510:
            all_neighbours[pos1[0]] = (round(pos1[2], 2), pos1[1])

        if pos2[0][0] >= -510 and pos2[0][1] >= -510 and pos2[0][0] <= 510 and pos2[0][1] <= 510:
            all_neighbours[pos2[0]] = (round(pos2[2], 2), pos2[1])

        if pos3[0][0] >= -510 and pos3[0][1] >= -510 and pos3[0][0] <= 510 and pos3[0][1] <= 510:
            all_neighbours[pos3[0]] = (round(pos3[2], 2), pos3[1])

        if pos4[0][0] >= -510 and pos4[0][1] >= -510 and pos4[0][0] <= 510 and pos4[0][1] <= 510:
            all_neighbours[pos1[0]] = (round(pos4[2], 2), pos4[1])

        if pos5[0][0] >= -510 and pos5[0][1] >= -510 and pos5[0][0] <= 510 and pos5[0][1] <= 510:
            all_neighbours[pos5[0]] = (round(pos5[2], 2), pos5[1])

        if pos6[0][0] >= -510 and pos6[0][1] >= -510 and pos6[0][0] <= 510 and pos6[0][1] <= 510:
            all_neighbours[pos6[0]] = (round(pos6[2], 2), pos6[1])

        if pos7[0][0] >= -510 and pos7[0][1] >= -510 and pos7[0][0] <= 510 and pos7[0][1] <= 510:
            all_neighbours[pos7[0]] = (round(pos7[2], 2), pos7[1])

        if pos8[0][0] >= -510 and pos8[0][1] >= -510 and pos8[0][0] <= 510 and pos8[0][1] <= 510:
            all_neighbours[pos8[0]] = (round(pos8[2], 2), pos8[1])

        return all_neighbours

    else:

        pass


# In[9]:


def BackTrack(backtrack_dict, goal, start):  # goal is the starting point now and start is the goal point now
    point_and_angle_list = []
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


# In[10]:


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
cost_from[int(2.0 * start[0])][int(2.0 * start[1])][orientation_to_layer[start_orientation]] = 0
f[int(2.0 * start[0])][int(2.0 * start[1])][orientation_to_layer[start_orientation]] = 0


# In[11]:


def a_star_Algorithm(start, goal):
    global step_size
    global untraversable_points
    global V
    break_while = 0

    # create a dictionary for backtracked parents
    backtracking = {}

    # check if goal/start in obstacle space

    if goal in untraversable_points or start in untraversable_points:

        print "!!!!!!!!!!GOAL/START IS INSIDE OBSTACLE SPACE!!!!!!!!!!" 
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
            if ((curr_vert[0] - goal[0]) ** 2.0 + (curr_vert[1] - goal[1]) ** 2.0 <= (1.5) ** 2.0):
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
                    orientation = orientation % 360.0
                    # Round the node
                    rounded_neighbour = (Round2Point5(neighbour[0]), Round2Point5(neighbour[1]))
                    #                     print("cost for this"+str(rounded_neighbour)+str(this_cost))
                    if rounded_neighbour in visited:
                        breakflag = 1
                        # exit if found
                    if breakflag == 1:
                        continue
                    # check if this neighbour is goal
                    if ((rounded_neighbour[0] - goal[0]) ** 2.0 + (rounded_neighbour[1] - goal[1]) ** 2.0 <= (1.5) ** 2.0):
                        break_while = 1
                        print "Goal"
                        break
                    ##check whether current neighbour node is in the obstacle space
                    if checkObstaclespace(rounded_neighbour) == True:
                        # check if visited
                        if V[int(2.0 * rounded_neighbour[0])][int(2.0 * rounded_neighbour[1])][
                            orientation_to_layer[orientation]] == 0:
                            # if not, make it visited
                            V[int(2.0 * rounded_neighbour[0])][int(2.0 * rounded_neighbour[1])][
                                orientation_to_layer[orientation]] = 1
                            # calculate cost from
                            cost_from[int(2.0 * rounded_neighbour[0])][int(2.0 * rounded_neighbour[1])][
                                orientation_to_layer[int(orientation)]] = (
                                    this_cost + cost_from[int(2.0 * curr_vert[0])][int(2.0* curr_vert[1])][
                                orientation_to_layer[curr_orient]])
                            # calculate cost to go values
                            heur[int(2.0 * rounded_neighbour[0])][int(2.0 * rounded_neighbour[1])] = EucledianDistance(
                                rounded_neighbour, goal)
                            # calculate f = g+h
                            f[int(2.0 * rounded_neighbour[0])][int(2.0 * rounded_neighbour[1])][
                                orientation_to_layer[orientation]] = \
                                cost_from[int(2.0 * rounded_neighbour[0])][int(2.0 * rounded_neighbour[1])][
                                    orientation_to_layer[orientation]] + heur[int(2.0 * rounded_neighbour[0])][
                                    int(2.0 * rounded_neighbour[1])]
                            # push to the explored node queue
                            heapq.heappush(priority_queue, (
                                f[int(2.0 * rounded_neighbour[0])][int(2.0 * rounded_neighbour[1])][
                                    orientation_to_layer[orientation]], rounded_neighbour, orientation))
                            backtracking[rounded_neighbour] = {}
                            # adding to the backtracking dictionary
                            backtracking[rounded_neighbour][
                                f[int(2.0 * rounded_neighbour[0])][int(2.0 * rounded_neighbour[1])][
                                    orientation_to_layer[orientation]]] = (curr_vert, curr_orient)
                        else:
                            # if visited, check cost. if newly genrated neighbour has a lower cost, update it
                            if (f[int(2.0 * rounded_neighbour[0])][int(2.0 * rounded_neighbour[1])][
                                orientation_to_layer[orientation]]) > (
                                    f[int(2.0 * curr_vert[0])][int(2.0 * curr_vert[1])][orientation_to_layer[curr_orient]]):
                                f[int(2.0 * rounded_neighbour[0])][int(2.0 * rounded_neighbour[1])][
                                    orientation_to_layer[orientation]] = (
                                    f[int(2.0 * curr_vert[0])][int(2.0 * curr_vert[1])][orientation_to_layer[curr_orient]])
                                backtracking[rounded_neighbour][
                                    f[int(2.0 * rounded_neighbour[0])][int(2.0 * rounded_neighbour[1])][
                                        orientation_to_layer[orientation]]] = (curr_vert, curr_orient)

    return (curr_vert, backtracking)


# In[12]:


new_goal_rounded, backtracking_dict = a_star_Algorithm(start, goal)


# In[13]:


backtracked_final = BackTrack(backtracking_dict, start, new_goal_rounded)


# In[14]:


ros_visited_points = []
for visit in list(visited):
    ros_visited_points.append((visit[0]/100.0,visit[1]/100.0))


# In[15]:


ros_backtracked_points = []
for backtr in backtracked_final:
    ros_backtracked_points.append((backtr[0]/100.0,backtr[1]/100.0))
ros_backtracked_points = ros_backtracked_points[::-1]

