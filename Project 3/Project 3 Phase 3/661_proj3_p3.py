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

#Getting the start time to measure the time taken for solving
    
 ####################User Inputs######################   


x_start= int(input("Enter the x coordinate of the start:  "))
y_start= int(input("Enter the y coordinate of the start:  "))

start_orientation = int(input("Enter the Orientation at start (enter in multiples of 30 degreees and less that 360 degrees), :  "))
x_goal= int(input("Enter the x coordinate of the goal:  "))
y_goal= int(input("Enter the y coordinate of the goal:  "))

RPM_R = int(input("Enter the RIGHT WHEEL RPM  : > "))
RPM_L = int(input("Enter the LEFT WHEEL RPM : > "))

radius= int(input("Enter the radius of the robot:  "))
clearance= int(input("Enter the clearance of the robot: "))
step_size= int(input("Enter the step (1-10): "))
start = (x_start,y_start)
goal = (x_goal,y_goal)


#######################################################

# %% 
#function to round the point 5

def Round2Point5(num):
    return (round(num * 2) / 2)


#for solving the path using eucledian heuristic
    
def EucledianDistance(a,b):  
    x1 = a[0]
    x2 = b[0]
    y1 = a[1]
    y2 = b[1]
    dist = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)  
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

# findneighbours(10,10,60,3,5)
#%% 
#can use manhattan or eucledian heuristic as for caclulation of the value of cost to go to the goal.

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

def ActionMove(curr_node, degree, RPM_L, RPM_R, step_size=1.0):
    x = curr_node[0]
    y = curr_node[1]
    x_new, y_new, new_orientation = findneighbours(x,y,degree,RPM_L,RPM_R)
    
    #calculating the degrees rotated   from the start to the new orientation and assigning costs accordingly
    degrees_rotated = abs(degree - new_orientation)
    
    if 0<=degrees_rotated<30:
        cost = 1
    elif 30<=degrees_rotated<60:
        cost = 1.4
    elif 60<=degrees_rotated<=90:
        cost = 1.8
        
    new_node = (round(x_new, 2), round(y_new, 2),new_orientation,cost)
    if 0.00 <= new_node[0] <= 1020.00 and 0.00 <= new_node[1] <= 1020.00:
        return new_node, True
    else:
        return curr_node, False
        
#%%        
######################################     POINTS FOR REMOVING VALUES FROM THE GRAPH
        
#calculating all the points within the entire canvas
        
# all possible points in integer format

all_possible_int_points = []

# points to draw the map, without considering the radius and clearance of the robot. This works with
# different sets of points as compared to the points created

untraversable_points = []
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
    if (x - 510) ** 2 + (y - 510) ** 2 <= 100 ** 2 + radius + clearance:
        untraversable_points.append((x, y))

    # circle shaped obstacle on top right
    # for path traversal
    if (x - 710) ** 2 + (y - 810) ** 2 <= 100 ** 2 + radius + clearance:
        untraversable_points.append((x, y))

    # circle shaped obstacle on bottom right
    # for path traversal
    if (x - 710) ** 2 + (y - 210) ** 2 <= 100 ** 2 + radius + clearance:
        untraversable_points.append((x, y))

    # circle shaped obstacle on bottom left
    # for path traversal
    if (x - 310) ** 2 + (y - 210) ** 2 <= 100 ** 2 + radius + clearance:
        untraversable_points.append((x, y))

    # borders

    # left vertical border
    if 0 <= x <= 10+ radius + clearance:
        if 0 <= y <= 1020:
            untraversable_points.append((x, y))

    # right vertical border
    if 1010 - radius - clearance<= x <= 1020:
        if 0 <= y <= 1020:
            untraversable_points.append((x, y))
    # bottom horizontal border
    if 10 < x < 1010:
        if 0 <= y <= 10+ radius + clearance:
            untraversable_points.append((x, y))
    # top horizontal border
    if 10 < x < 1010:
        if 1010 - radius - clearance<= y <= 1020:
            untraversable_points.append((x, y))

    # squares

    # left square
    if 35- radius - clearance <= x <= 185 + radius + clearance :
        if 435- radius - clearance <= y <= 585 + radius + clearance :
            untraversable_points.append((x, y))

    # right square
    if 835- radius - clearance <= x <= 985+ radius + clearance:
        if 435- radius - clearance <= y <= 585 + radius + clearance:
            untraversable_points.append((x, y))

    # top left square
    if 235 - radius - clearance <= x <= 385 + radius + clearance:
        if 735- radius - clearance <= y <= 885 + radius + clearance:
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

#%%     
#showing what every child node of each parent nodes are connected to
list_of_points_for_graph = []
size_x, size_y = 1021,1021

def generateGraph(point, degree, step_size = 1): #remember that this size_x and size_y are the sizes of the matrix, so not the end coordinates
    

    global list_of_points_for_graph
    global size_x
    global size_y
    global RPM_R
    global RPM_L
    
    i = point[0] #x coordinate
    j = point[1] #y coordinate
    
    if i <=size_x and j<=size_y and i>=0 and j>=0:
        
        cost_values = {}
        
        pos1 = ActionMove(point,degree,0,RPM_L,step_size = 1.0)[0]
        pos2 = ActionMove(point,degree,RPM_L,0,step_size = 1.0)[0]
        pos3 = ActionMove(point,degree,RPM_L,RPM_L,step_size = 1.0)[0]
        
        pos4 = ActionMove(point,degree,0,RPM_R,step_size = 1.0)[0]
        pos5 = ActionMove(point,degree,RPM_R,0,step_size = 1.0)[0]
        pos6 = ActionMove(point,degree,RPM_R,RPM_R,step_size = 1.0)[0]
        
        pos7 = ActionMove(point,degree,RPM_L,RPM_R,step_size = 1.0)[0]
        pos8 = ActionMove(point,degree,RPM_R,RPM_L,step_size = 1.0)[0]
        
        print(pos1)
        if pos1[0]>=0 and pos1[1]>=0 and pos1[0]<=size_x and pos1[1]<=size_y:
            new_position = pos1
            
        if pos2[0]>=0 and pos2[1]>=0 and pos2[0]<=size_x and pos2[1]<=size_y:
            new_position = pos2
            
        if pos3[0]>=0 and pos3[1]>=0 and pos3[0]<=size_x and pos3[1]<=size_y:
            new_position = pos3
        
        if pos4[0]>=0 and pos4[1]>=0 and pos4[0]<=size_x and pos4[1]<=size_y:
            new_position = pos4
            
        if pos5[0]>=0 and pos5[1]>=0 and pos5[0]<=size_x and pos5[1]<=size_y:
            new_position = pos5
            
        if pos6[0]>=0 and pos6[1]>=0 and pos6[0]<=size_x and pos6[1]<=size_y:
            new_position = pos6
            
        if pos7[0]>=0 and pos7[1]>=0 and pos7[0]<=size_x and pos7[1]<=size_y:
            new_position = pos7
            
        if pos8[0]>=0 and pos8[1]>=0 and pos8[0]<=size_x and pos8[1]<=size_y:
            new_position = pos8
            
        return new_position
    
    else:
        
        pass

# generateGraph((5,5),0)
# function to backtrack so that the path from the goal to the end is obtained
#%%            
def BackTrack(backtrack_dict,goal,start):#goal is the starting point now and start is the goal point now
    
    #initializing the backtracked list
    back_track_list = []
    #appending the start variable to the back_track_list list
    back_track_list.append(start)
    #while the goal is not found
    
    while goal!=[]:
        #for key and values in the backtracking dictionary 
        for k,v in backtracking_dict.items():
            
            #for the key and values in the values, v
            for k2,v2 in v.items():
                
                #checking if the first key is the start
                if k==start:
                    
                    #checking if not in the backtrackedlist
                    
                    if v2 not in back_track_list:
                        back_track_list.append(start)
                    #updating the start variable
                    start=v2
                    
                    #checking if it is the goal
                    if v2==goal:
                        goal=[]
                        break      
    #returns the backtracked list
    return(back_track_list)

#empty dictionary for backtracking from child to parent upto the start
backtracking = {}
#list of all the visited nodes
visited = set()
global orientation_to_layer
#Mapping of Angles to layers
orientation_to_layer={0:0,30:1,60:2,90:3,120:4,150:5,180:6,210:7,240:8,270:9,300:10,330:11, 360:0}
#array to store cost from
# creating a 200rows, by 300 column by 12 layers for the various costs and to check if a node is visited
cost_from=np.array(np.ones((600,400,12)) * np.inf)
#Initializing visited nodes as empty array
V=np.zeros((600,400,12))
# array to store Heuristic distance
heur=np.array(np.ones((600,400)) * np.inf)
#array to store total cost f
f=np.array(np.ones((600,400,12)) * np.inf)
#list for Explored nodes
priority_queue=[]
# append start point,start orientation of the bot and initialize it's cost to zero
heapq.heappush(priority_queue,(0,start,start_orientation))
#initialize cost  for start node to zero
cost_from[int(2*start[0])][int(2*start[1])][orientation_to_layer[start_orientation]]=0
f[int(2*start[0])][int(2*start[1])][orientation_to_layer[start_orientation]]=0

#%%     
######### < < < < A* algorithm code > > > > #########

def a_star_Algorithm(start,goal):
    
    global step_size
   
    global list_of_obstacle_points
    global V
    break_while=0
  
    #create a dictionary for backtracked parents
    backtracking = {}
    
    #check if goal/start in obstacle space
  
    if goal in list_of_obstacle_points or start in list_of_obstacle_points:
        
        print('!!!!!!!!!!GOAL/START IS INSIDE OBSTACLE SPACE!!!!!!!!!!')
        backtracking=0
        rounded_neighbour=0
        
    else:
        while True:
            if break_while==1:
                break
            _,curr_vert,curr_orient=heapq.heappop(priority_queue)
            #append visited nodes
            visited.add(curr_vert)
            #checking if the neighbour is the goal. If goal found reached
            if ((curr_vert[0]-goal[0])**2 + (curr_vert[1]-goal[1])**2 <= (1.5)**2):
                print('^^^^^^^^^^^^^^')
                print('^^^^^^^^^^^^^^')
                print('GOAL REACHED')
                print('^^^^^^^^^^^^^^')
                print('^^^^^^^^^^^^^^')
                print(curr_vert)
                break
            #check whether node is in the obstacle space
            if checkObstaclespace(curr_vert)==True:
                #generate neighbours
                graph = generateGraph(curr_vert,601,401,curr_orient,step_size)
                graph_list = []
                #put neighbours in a list for easy access
                for key,cost_value in graph.items():
                    graph_list.append((key,cost_value))  
                for neighbour,cost in graph_list:
                    this_cost = graph[neighbour][0]
                    breakflag = 0
                    orientation = graph[neighbour][1]
                    orientation = orientation % 360
                    #Round the node
                    rounded_neighbour = (Round2Point5(neighbour[0]),Round2Point5(neighbour[1]))
                    if rounded_neighbour in visited:
                        breakflag=1 
                    #exit if found
                    if breakflag==1:
                        continue
                    #check if this neighbour is goal
                    if ((rounded_neighbour[0]-goal[0])**2 + (rounded_neighbour[1]-goal[1])**2 <= (1.5)**2):
                        print('^^^^^^^^^^^^^^')
                        print('^^^^^^^^^^^^^^')
                        print('GOAL REACHED')
                        print('^^^^^^^^^^^^^^')
                        print('^^^^^^^^^^^^^^')
                        break_while = 1
                        break
                    ##check whether current neighbour node is in the obstacle space
                    if checkObstaclespace(rounded_neighbour)==True:
                        #check if visited
                        if V[int(2*rounded_neighbour[0])][int(2*rounded_neighbour[1])][orientation_to_layer[orientation]]==0:
                            #if not, make it visited
                            V[int(2*rounded_neighbour[0])][int(2*rounded_neighbour[1])][orientation_to_layer[orientation]]=1
                            #calculate cost from
                            cost_from[int(2*rounded_neighbour[0])][int(2*rounded_neighbour[1])][orientation_to_layer[int(orientation)]]= (step_size +cost_from[int(2*curr_vert[0])][int(2*curr_vert[1])][orientation_to_layer[curr_orient]])
                            #calculate cost to go values
                            heur[int(2*rounded_neighbour[0])][int(2*rounded_neighbour[1])] = find_manhattan_distance(rounded_neighbour,goal)
                            #calculate f = g+h
                            f[int(2*rounded_neighbour[0])][int(2*rounded_neighbour[1])][orientation_to_layer[orientation]]=cost_from[int(2*rounded_neighbour[0])][int(2*rounded_neighbour[1])][orientation_to_layer[orientation]] + heur[int(2*rounded_neighbour[0])][int(2*rounded_neighbour[1])]
                            #push to the explored node queue
                            heapq.heappush(priority_queue,(f[int(2*rounded_neighbour[0])][int(2*rounded_neighbour[1])][orientation_to_layer[orientation]] ,rounded_neighbour,orientation))
                            backtracking[rounded_neighbour]={}
                            #adding to the backtracking dictionary
                            backtracking[rounded_neighbour][f[int(2*rounded_neighbour[0])][int(2*rounded_neighbour[1])][orientation_to_layer[orientation]]]=curr_vert
                        else:
                            #if visited, check cost. if newly genrated neighbour has a lower cost, update it
                            if(f[int(2*rounded_neighbour[0])][int(2*rounded_neighbour[1])][orientation_to_layer[orientation]])>(f[int(2*curr_vert[0])][int(2*curr_vert[1])][orientation_to_layer[curr_orient]]+step_size):
                                f[int(2*rounded_neighbour[0])][int(2*rounded_neighbour[1])][orientation_to_layer[orientation]]=(f[int(2*curr_vert[0])][int(2*curr_vert[1])][orientation_to_layer[curr_orient]]+step_size)
                                backtracking[rounded_neighbour][f[int(2*rounded_neighbour[0])][int(2*rounded_neighbour[1])][orientation_to_layer[orientation]]]=curr_vert
                    
    return(curr_vert,backtracking)                             
    #return the last parent node and backtracked dictionary
        
start_time = time.time()
#calling the function to solve the obstacle space using A* Algorithm
new_goal_rounded,backtracking_dict= a_star_Algorithm(start,goal)

print("SOLVED in =",time.time()-start_time)
#Backtracking back to the goal
backtracked_final = BackTrack(backtracking_dict,start,new_goal_rounded)
#printing the nodes from initial to the end
print(backtracked_final)


#Scale the visited nodes and backtracked nodes for easy visualization
new_list_visited = []
for visited_node in list(visited):
    new_x = visited_node[0]*2
    new_y = visited_node[1]*2
    new_list_visited.append((new_x,new_y))
    
new_backtracked = []
for back in backtracked_final:
    new_x_b = back[0]*2
    new_y_b = back[1]*2
    new_backtracked.append((new_x_b,new_y_b))
    
# #defining a blank canvas
new_canvas = np.zeros((400,600,3),np.uint8) 

#for every point that belongs within the obstacle
for c in map_points: #change the name of the variable l
    x = 2*c[1]
    y = 2*c[0]
    new_canvas[(x,y)]=[255,0,255] #assigning a yellow coloured pixel
    
#flipping the image for correct orientation
new_canvas = np.flipud(new_canvas)
#making a copy for backtracking purpose
new_canvas_copy_backtrack = new_canvas.copy()
#making a copy for showing the visited nodes on the obstacle space
#can be used for the animation
new_canvas_copy_visited = new_canvas.copy()

#showing the obstacle map

cv2.imshow('new_canvas',new_canvas)
cv2.waitKey(0)
cv2.destroyAllWindows()



#visited path
for visit_path in new_list_visited:
    
    #print(path)
    x = int(visit_path[0])
    y = int(visit_path[1])
    new_canvas_copy_visited[(400-y,x)]=[255,255,255] #setting every backtracked pixel to white
    
#showing the final backtracked path
#new_visited = cv2.resize(new_canvas_copy_visited,(600,400))
#showing the image
cv2.imshow('visited',new_canvas_copy_visited)
#saving the image
#cv2.imwrite('visited_img.jpg',new_visited)
cv2.waitKey(0)
cv2.destroyAllWindows()

#backtracked path
for path in new_backtracked:
    
    #print(path)
    x = int(path[0])
    y = int(path[1])
    new_canvas_copy_backtrack[(400-y,x)]=[255,255,255] #setting every backtracked pixel to white
#showing the final backtracked path
#new_backtracked = cv2.resize(new_canvas_copy_backtrack,(600,400))

#showing the image
cv2.imshow('backtracked',new_canvas_copy_backtrack)
#saving the image
#cv2.imwrite('backtracked_img.jpg',new_backtracked)
cv2.waitKey(0)
cv2.destroyAllWindows()


# # # In[ ]:

#showing animation
pygame.init()

display_width = 600
display_height = 400

gameDisplay = pygame.display.set_mode((display_width,display_height),pygame.FULLSCREEN)
pygame.display.set_caption('Covered Nodes- Animation')

black = (0,0,0)
white = (0,255,255)
#new = np.array(new_canvas_copy_visited)
surf = pygame.surfarray.make_surface(new_canvas_copy_visited)

clock = pygame.time.Clock()

done = False
while not done:
    
    for event in pygame.event.get(): 
        
        if event.type == pygame.QUIT:  
            done = True   
 
    gameDisplay.fill(black)
    for path in new_list_visited:
        if path not in new_canvas_copy_visited:
            pygame.time.wait(1)
            x = path[0]
            y = abs(400-path[1])
            pygame.draw.rect(gameDisplay, white, [x,y,1,1])
            pygame.display.flip()
            
    for path in new_backtracked:
        
        pygame.time.wait(5)
        x = path[0]
        y = abs(400-path[1])
        pygame.draw.rect(gameDisplay, (0,0,255), [x,y,1,1])
        pygame.display.flip()

    done = True
pygame.quit()