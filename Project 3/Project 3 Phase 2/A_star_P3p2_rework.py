#!/usr/bin/env python
# coding: utf-8

# # Project 3 | Phase 2 | ENPM 661 | Planning for Autonomous Robots |

#SUBMISSION : March 20, 2020

#GITHUB: https://github.com/govindak-umd/ENPM661/tree/master/Project%203/Project%203%20Phase%202
#YOUTUBE: https://www.youtube.com/watch?v=VBzvzb2vaYg&feature=emb_logo

# A* Algorithm for Rigid Robot with Obstacles | Govind Ajith Kumar & Rajeshwar NS

#Importing all the libraries
import numpy as np
import copy
import math
import heapq
import time
import matplotlib.pyplot as plt
import cv2
import pygame

#Getting the start time to measure the time taken for solving

start_time = time.time()

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

#for solving the path using manhattan heuristic
    
def find_manhattan_distance(curr_position,goal):
    manhattan_distance = (abs(curr_position[1] - goal[1]) + abs(curr_position[0] - goal[0]))
    return (manhattan_distance)


#can use manhattan or eucledian heuristic as for caclulation of the value of cost to go to the goal.

#####               PRESET VALUES
    
step_count=1 #steps taken by the robot. Do vary as per the need
size_x = 301 #size of the x axis of the map
size_y = 201 #size of the y axis of the map
radius = 1 #radius of the robot, just for the purpose of creating the map of the obstacle space
clearance = 1 #clearance of the robot, just for the purpose of creating the map of the obstacle space

#####

#function to move any direction based on the degree
#step_size by default is taken as 1.0

def ActionMove(curr_node,degree,step_size=1.0):
    x = curr_node[0]
    y = curr_node[1]
    x_new = (step_size)*np.cos(np.deg2rad(degree)) + x# - (y-0)*np.sin(np.deg2rad(degree)) + length
    y_new = (step_size)*np.sin(np.deg2rad(degree)) + y# + (y-0)*np.cos(np.deg2rad(degree)) + length
    new_node = (round(x_new,2),round(y_new,2))
    if new_node[0]>=0.00 and new_node[0]<=300.00 and new_node[1]>=0.00 and new_node[1]<=200.00:
        return(new_node,True)
    else:
        return(curr_node,False)
    
        
        
######################################     POINTS FOR REMOVING VALUES FROM THE GRAPH
        
#calculating all the points within the entire canvas
        
all_possible_points = []
for i in range(0,601): #300 width with a gap of 0.5
    for j in range(0,401): #200 width with a gap of 0.5
        all_possible_points.append((Round2Point5(i/2),Round2Point5(j/2))) #appending
        
#all possible points in the obstacle space
list_of_obstacle_points=[]

for pt in all_possible_points:
    
    x = pt[0]
    y = pt[1]
    
#circle shaped obstacle
    #for path traversal
    if((x-225)**2 + (y-150)**2 <= (25+radius+clearance)**2):
        list_of_obstacle_points.append((x,y))

#ellipse shaped obstacle
    #for path traversal
    if(((x-150)**2)/(40+radius+clearance)**2 +((y-100)**2)/(20+radius+clearance)**2 <= 1):
        list_of_obstacle_points.append((x,y))

#complex polygon shaped obstacle
    #for path traversal
    if y>=120+radius+clearance and y<=185+radius+clearance and x>=20+radius+clearance and x<=100+radius+clearance:
        if y-(13*x)<=-140+radius+clearance:
            if y-x>=100-(radius+clearance) or y+(6/5)*x>=210-(radius+clearance):
                if y-(6/5)*x>=30-(radius+clearance):
                    if y+(7/5)*x<=290+(radius+clearance):
                        list_of_obstacle_points.append((x,y))

#rectangle slanted shaped obstacle
    #for path traversal
    if y-(8/5)*x>=-122-(radius+clearance) and y+(38/65)*x<=(1254/13)+(radius+clearance) and y-(9/5)*x<=13+(radius+clearance) and y+(37/65)*x>=(1093/13)-(radius+clearance):
        list_of_obstacle_points.append((x,y))

#rhombus shaped obstacle
    #for path traversal
    if y-(3/5)*x<=-95+(radius+clearance) and y+(3/5)*x<=175+(radius+clearance) and y-(3/5)*x>=-125-(radius+clearance) and y+(3/5)*x>=145-(radius+clearance):
        list_of_obstacle_points.append((x,y))

######################################        POINTS FOR DRAWING THE GRAPH
        
#all possible points in integer format
        
all_possible_int_points = []

#points to draw the map, without considering the radius and clearance of the robot. This works with
#different sets of poinst as compared to the points created 

map_points = []

for i in range(0,301): #300 width
    for j in range(0,201): #200 width
        all_possible_int_points.append((i,j)) #appending

for pt in all_possible_int_points:
    x = pt[0]
    y = pt[1]
#circle shaped obstacle
    #for path traversal
    if((x-225)**2 + (y-150)**2 <= (25)**2):
        map_points.append((x,y))

#ellipse shaped obstacle
    #for path traversal
    #for map
    if(((x-150)**2)/40**2 +((y-100)**2)/20**2 <= 1):
        map_points.append((x,y))

#complex polygon shaped obstacle
    #for path traversal
    if y>=120 and y<=185 and x>=20 and x<=100:
        if y-(13*x)<=-140:
            if y-x>=100 or y+(6/5)*x>=210:
                if y-(6/5)*x>=30:
                    if y+(7/5)*x<=290:
                        map_points.append((x,y))

#rectangle slanted shaped obstacle
    #for path traversal
    if y-(8/5)*x>=-122 and y+(38/65)*x<=(1254/13) and y-(9/5)*x<=13 and y+(37/65)*x>=(1093/13):
        map_points.append((x,y))

#rhombus shaped obstacle
    #for path traversal
    if y-(3/5)*x<=-95 and y+(3/5)*x<=175 and y-(3/5)*x>=-125 and y+(3/5)*x>=145:
        map_points.append((x,y))

#sorting out the points
list_of_obstacle_points.sort()

#function to check if any point is within the obstacle space
#including the region covered by the robot radius and its clearance

def checkObstaclespace(point):
    
    test = []
    x = point[0]
    y = point[1]
    #circle shaped obstacle
    #for path traversal
    
    if((x-225)**2 + (y-150)**2 <= (25+radius+clearance)**2):
        return False
        
#ellipse shaped obstacle
    #for path traversal
    elif(((x-150)**2)/(40+radius+clearance)**2 +((y-100)**2)/(20+radius+clearance)**2 <= 1):
        return False

#complex polygon shaped obstacle
    #for path traversal
    elif y>=120+radius+clearance and y<=185+radius+clearance and x>=20+radius+clearance and x<=100+radius+clearance:
        if y-(13*x)<=-140+radius+clearance:
            if y-x>=100-(radius+clearance) or y+(6/5)*x>=210-(radius+clearance):
                if y-(6/5)*x>=30-(radius+clearance):
                    if y+(7/5)*x<=290+(radius+clearance):
                        return False
#rectangle slanted shaped obstacle
    #for path traversal
    elif y-(8/5)*x>=-122-(radius+clearance) and y+(38/65)*x<=(1254/13)+(radius+clearance) and y-(9/5)*x<=13+(radius+clearance) and y+(37/65)*x>=(1093/13)-(radius+clearance):
        return False

#rhombus shaped obstacle
    #for path traversal
    elif y-(3/5)*x<=-95+(radius+clearance) and y+(3/5)*x<=175+(radius+clearance) and y-(3/5)*x>=-125-(radius+clearance) and y+(3/5)*x>=145-(radius+clearance):
        return False
    
    else:
        #If the object is not in any obstacle space
        return True

        
#Taking inputs
#sample inputs shown below
#test_case : 
    #start = (50,30)
    #orientation = 60
    #goal = (150,150)
    #radius = 1
    #clearance =1
    
    
x_start= int(input("Enter the x coordinate of the start:  "))
y_start= int(input("Enter the y coordinate of the start:  "))
orientation = int(input("Enter the Orientation at start (enter in multiples of 30 degreees and less that 360 degrees), :  "))
x_goal= int(input("Enter the x coordinate of the goal:  "))
y_goal= int(input("Enter the y coordinate of the goal:  "))
radius= int(input("Enter the radius of the robot:  "))
clearance= int(input("Enter the clearance of the robot: "))
start = (x_start,y_start)
goal = (x_goal,y_goal)

#showing what every child node of each parent nodes are connected to
list_of_points_for_graph = []

def generateGraph(point,size_x,size_y): #remember that this size_x and size_y are the sizes of the matrix, so not the end coordinates
    
    global step_count
    global orientation
    global list_of_points_for_graph
    
    i = point[0] #x coordinate
    j = point[1] #y coordinate
    
    if i <=size_x and j<=size_y and i>=0 and j>=0:
        
        cost_values = {}
        
        pos0 = ActionMove(point,orientation+0)[0]#0
        if pos0[0]>=0 and pos0[1]>=0 and pos0[0]<=size_x and pos0[1]<=size_y:
            cost_values[pos0] = (step_count,orientation)
            
        pos30 = ActionMove(point,orientation+30)[0]#30
        if pos30[0]>=0 and pos30[1]>=0 and pos30[0]<=size_x and pos30[1]<=size_y: 
            cost_values[pos30] = (1.4,orientation+30)
            
        pos60 = ActionMove(point,orientation+60)[0]#60
        if pos60[0]>=0 and pos60[1]>=0 and pos60[0]<=size_x and pos60[1]<=size_y:
            cost_values[pos60] = (1.7,orientation+60)
            
        pos_minus60 = ActionMove(point,orientation-60)[0]#-60
        if pos_minus60[0]>=0 and pos_minus60[1]>=0 and pos_minus60[0]<=size_x and pos_minus60[1]<=size_y:
            cost_values[pos_minus60] = (1.7,orientation-60)
            
        pos_minus30 = ActionMove(point,orientation-30)[0]#-30
        if pos_minus30[0]>=0 and pos_minus30[1]>=0 and pos_minus30[0]<=size_x and pos_minus30[1]<=size_y:
            cost_values[pos_minus30] = (1.4,orientation-30)
            
        cost_values_copy = cost_values.copy()
        
        for k,v in cost_values_copy.items():
            if k==point:
                del cost_values[k]
        return(cost_values)
    
    else:
        
        pass

# function to backtrack so that the path from the goal to the end is obtained
        
def BackTrack(backtrack_dict,goal,start):#goal is the starting point now and start is the goal point now
    
    #initializing the backtracked list
    back_track_list = []
    #appending the start variable to the back_track_list list
    back_track_list.append(start)
    #while the goal is not found
    
    while goal!=[]:
        #for key and values in the backtracking dictionary 
        for k,v in backtracking.items():
            
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

#empty dictionary with all the distances
all_distance = {}
#empty dictionary with all the distances
open_list = {}
#empty dictionary for backtracking from child to parent upto the start
backtracking = {}
#list of all the visited nodes
# creating a 200rows, by 300 column by 12 layers
rows = 401
columns = 601
layers =12 
V=np.zeros((rows,columns,layers))
visited = []
#if visualization is a problem, stack the layers and work from there
#variable to exit out of the while loop in the dijkstraAlgorithm function
check=0
######### < < < < A* algorithm code > > > > #########

def a_star_Algorithm(start,goal):
    
    global orientation
    global list_of_obstacle_points
    # global step_count
    global backtracking
    #adding the global variables
    global check
    global visited
    global all_distance
    #when the function starts
    
    if goal in list_of_obstacle_points or start in list_of_obstacle_points:
        
        print('!!!!!!!!!!GOAL/START IS INSIDE OBSTACLE SPACE!!!!!!!!!!')
        all_distance=0
        backtracking=0
        rounded_neighbour=0
        
    else:
        
        all_distance[start]=0
        #appending the start node to the list of visited nodes
        #setting all nodes as infinity distance away
        #starting the priority queue with the start node
        priority_queue = [(0,start,orientation)]
        #checking the length of the priority queue
        #and, inserting the while loop exit condition
        
        while len(priority_queue)>0 and check!=[]:
            
            curr_dist,curr_vert,orientation = heapq.heappop(priority_queue)
            print('###########################################################################################')
            print('                                     PARENT CHANGE')
            print('curr_vert > ',curr_vert)
            # print('curr_dist > ',curr_dist)
            print('ORIENTATION CHANGED TO >',orientation)
           
            
            if checkObstaclespace(curr_vert)==True:
                graph = generateGraph(curr_vert,301,201)
                print('graph is> ',graph)

                for vertex,edge in graph.items():
                    all_distance[vertex]=math.inf
                graph_list = []
                
                for key,cost_value in graph.items():
                    graph_list.append((key,cost_value))
                    
                #checking the value of the current distance and 
                if curr_dist>all_distance[curr_vert]:
                    continue
                
                for neighbour,cost in graph_list:
                    this_cost = graph[neighbour][0]
                    curr_dist = 0
                    distance = curr_dist + this_cost + find_manhattan_distance(neighbour,goal)
                    print('-------------COST CALCULATION-----------')
                    print('curr_dist is',curr_dist)
                    print('MANHATTAN DISTANCE for',neighbour,'to the goal',goal,' IS > ',find_manhattan_distance(neighbour,goal))
                    print('this cost is',this_cost)
                    print('----------------------------------------')
                    # print('angle is > ',graph[neighbour][1],'cost is>'graph[neighbour][0])
                    #checking when the variable <distance> that was calculated is lesse
                    #than the neighbouring cost
                    
                    if distance < all_distance[neighbour]:
                        
                        rounded_neighbour = (Round2Point5(neighbour[0]),Round2Point5(neighbour[1]))

                        try:
                            
                            orientation = ((orientation) % 360)
                            #maintaining a dictionary of all the angles and their layer value                                
                            orientation_to_layer={0:0,30:1,60:2,90:3,120:4,150:5,180:6,210:7,240:8,270:9,300:10,330:11, 360:0}
    #                         print('rounded neighbour > ',rounded_neighbour,'with orientation', orientation)
                            # print('inside try blck',rounded_neighbour[0])
                            if rounded_neighbour not in visited:
                                if V[int(2*rounded_neighbour[0])][int(2*rounded_neighbour[1])][orientation_to_layer[orientation]]==0:
                                    V[int(2*rounded_neighbour[0])][int(2*rounded_neighbour[1])][orientation_to_layer[orientation]]=1
                                    # print('inside ZERO')
                                    visited.append(rounded_neighbour)
                                    backtracking[rounded_neighbour]={}
                                    #adding to the backtracking dictionary
                                    backtracking[rounded_neighbour][distance]=curr_vert
                                    all_distance[rounded_neighbour]=distance
                                    #pushing from the priority queue
                                    #rounded neighbour
                                    orientation = graph[neighbour][1]
                                    heapq.heappush(priority_queue, (distance, rounded_neighbour,orientation))
                                    #checking of the neighbour is not added to the visited
                                    #checks what node to go to next      
                                    #appending to the visited list

                                    #keeping the angles between 0 and 360 degree
                                    #checking if the neighbour is the goal
                                    if ((rounded_neighbour[0]-goal[0])**2 + (rounded_neighbour[1]-goal[1])**2 <= (1.5)**2):
                                        print('^^^^^^^^^^^^^^')
                                        print('^^^^^^^^^^^^^^')
                                        print('GOAL REACHED')
                                        print('^^^^^^^^^^^^^^')
                                        print('^^^^^^^^^^^^^^')
                                        #changing check variable for the exit condition
                                        check=[]
                                        #breaking out of the loop
                                        break
                                    else:

                                        pass
                        except:
                            pass
    return(all_distance,backtracking,rounded_neighbour)     

#calling the function to solve the obstacle space using A* Algorithm
all_distances,backtracking,new_goal_rounded= a_star_Algorithm(start,goal)


# In[ ]:

#Backtracking back to the goal
backtracked_final = BackTrack(backtracking,start,new_goal_rounded)

#printing the nodes from initial to the end
print(backtracked_final)


#function generates every child to every parent that has been backtracked here
def generateChilds(backtrack):
    
    parents2children = {}
    for parent in backtrack:
        child = generateGraph(parent,301,201)
        
        all_children_here = set()
        for key,value in child.items():
            all_children_here.add(key)
        parents2children[parent] = all_children_here
        
    return(parents2children)

#calling the function
branched_parents = generateChilds(backtracked_final)

#printing the path traversed on MATPLOTLIB

for i in range(1,len(backtracked_final)-1):
    
    x = backtracked_final[i][0]
    y = backtracked_final[i][1]
    x2 = backtracked_final[i+1][0]
    y2 = backtracked_final[i+1][1]
    plt.plot([x,x2],[y,y2])

#saving the figure
plt.savefig('Path traversed.png', bbox_inches='tight')

print("Total Time Taken : ",time.time() - start_time, "seconds")



#defining a blank canvas
new_canvas = np.zeros((201,301,3),np.uint8) 

#for every point that belongs within the obstacle
for c in map_points: #change the name of the variable l
    x = c[1]
    y = c[0]
    new_canvas[(x,y)]=[0,255,255] #assigning a yellow coloured pixel
    
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


#backtracked path
for path in backtracked_final:
    
    #print(path)
    x = int(path[0])
    y = int(path[1])
    new_canvas_copy_backtrack[(200-y,x)]=[255,0,0] #setting every backtracked pixel to white
#showing the final backtracked path
new_backtracked = cv2.resize(new_canvas_copy_backtrack,(600,400))

#showing the image
cv2.imshow('backtracked',new_backtracked)
#saving the image
cv2.imwrite('backtracked_img.jpg',new_backtracked)
cv2.waitKey(0)
cv2.destroyAllWindows()


#visited path
for path in visited:
    
    #print(path)
    x = int(path[0])
    y = int(path[1])
    new_canvas_copy_visited[(200-y,x)]=[255,0,0] #setting every backtracked pixel to white
    
#showing the final backtracked path
new_visited = cv2.resize(new_canvas_copy_visited,(600,400))
#showing the image
cv2.imshow('visited',new_visited)
#saving the image
cv2.imwrite('visited_img.jpg',new_visited)
cv2.waitKey(0)
cv2.destroyAllWindows()


# In[ ]:

#showing animation
pygame.init()

display_width = 300
display_height = 200

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
    for path in visited:
        if path not in new_canvas_copy_visited:
            pygame.time.wait(7)
            x = path[0]
            y = abs(200-path[1])
            pygame.draw.rect(gameDisplay, white, [x,y,1,1])
            pygame.display.flip()
            
    for path in backtracked_final:
        
        pygame.time.wait(5)
        x = path[0]
        y = abs(200-path[1])
        pygame.draw.rect(gameDisplay, (0,0,255), [x,y,1,1])
        pygame.display.flip()

    done = True
pygame.quit()


# In[ ]:

# GRAPH 1 : MATPLOTLIB PLOTTED
# SHOWS THE VECTORS AT EVERY CHILD NODE
# AND CONNECTION BETWEEN EVERY PARENT AND THEIR CHILD NODES

# NOTE: This is a zoomed OUT version
X_origin = np.array((0))#starting parent
Y_origin= np.array((0))#starting parent
U_firstnode = np.array((50))#child
V_firstnode= np.array((30))#child

fig, ax = plt.subplots()
plt.quiver(X_origin, Y_origin, U_firstnode, V_firstnode,units='xy' ,scale=1,color= 'r',headwidth = 1,headlength=0)

all_parents = []

for parent, children in branched_parents.items():
    
    parent_x = parent[0]
    parent_y = parent[1]
    X_p = np.array((parent_x))#starting parent
    Y_p= np.array((parent_y))#starting parent
    plt.scatter(X_p, Y_p, c='g')
    all_parents.append(parent)#for later connecting of parents
    
    for i in children:
        
        child_x = i[0]
        child_y = i[1]
        
        X_c = np.array((child_x))#starting child
        Y_c= np.array((child_y))#starting child
        plt.quiver(X_p, Y_p, X_c, Y_c,units='xy' ,scale=6)
    
plt.grid()

ax.set_aspect('equal')
#changing xlim and ylim for zoomed version
plt.xlim(0,300)
plt.ylim(0,200)
#Inserting figure title
plt.title('Parent and children vector map',fontsize=20)
#saving the figure
plt.savefig('Parent-child_vector_map.png', bbox_inches='tight')

plt.show()


# In[ ]:

# GRAPH 2 : MATPLOTLIB PLOTTED
# SHOWS THE VECTORS AT EVERY CHILD NODE
# AND CONNECTION BETWEEN EVERY PARENT AND THEIR CHILD NODES

# NOTE: This is a zoomed IN version

X_origin = np.array((0))#starting parent
Y_origin= np.array((0))#starting parent
U_firstnode = np.array((50))#child
V_firstnode= np.array((30))#child

fig, ax = plt.subplots()
plt.quiver(X_origin, Y_origin, U_firstnode, V_firstnode,units='xy' ,scale=1,color= 'r',headwidth = 1,headlength=0)

all_parents = []
for parent, children in branched_parents.items():
    parent_x = parent[0]
    parent_y = parent[1]
    X_p = np.array((parent_x))#starting parent
    Y_p= np.array((parent_y))#starting parent
    plt.scatter(X_p, Y_p, c='b')
    all_parents.append(parent)#for later connecting of parents
    for i in children:
        child_x = i[0]
        child_y = i[1]
        
        X_c = np.array((child_x))#starting child
        Y_c= np.array((child_y))#starting child
        plt.quiver(X_p, Y_p, X_c, Y_c,units='xy' ,scale=6)
    
plt.grid()

ax.set_aspect('equal')
#changing xlim and ylim for zoomed version
plt.xlim(40,120)
plt.ylim(25,50)
#Inserting figure title
plt.title('Parent and children vector zoomed',fontsize=20)
#saving the figure
plt.savefig('zoomed_vector_map.png', bbox_inches='tight')

plt.show()

#THE END
# In[ ]:




