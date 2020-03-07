#!/usr/bin/env python
# coding: utf-8

# # Project 2 | ENPM 661 | Planning for Autonomous Robots |
# 
# 
# 
# Dijkstra Algorithm for Rigid Robot with Obstacles | Govind Ajith Kumar & Rajeshwar NS

# In[1]:


# In[2]:


#Importing all the libraries
import numpy as np
import math
import heapq
import time
import cv2
import pygame


# In[3]:


#Getting the start time to measure the time taken for solving
start_time = time.time()


# In[4]:


#moving up thorugh the coordinate points on the cartersian plane
def ActionMoveUp(curr_node):
    x = curr_node[0]
    y = curr_node[1]
    curr_node = (x,y)
    new_node=()
    new_node_y = curr_node[0]
    new_node_y-=1
    new_node = (new_node_y,curr_node[1])
    if new_node[0]>=0 and new_node[1]>=0:
        return(new_node,True)
    else:
        return(curr_node,False)
#moving left thorugh the coordinate points on the cartersian plane      
def ActionMoveLeft(curr_node):
    x = curr_node[0]
    y = curr_node[1]
    curr_node = (x,y)
    new_node=()
    new_node_x = curr_node[1]
    new_node_x-=1
    new_node = (curr_node[0],new_node_x)
    if new_node[0]>=0 and new_node[1]>=0:
        return(new_node,True)
    else:
        return(curr_node,False)
#moving right thorugh the coordinate points on the cartersian plane      
def ActionMoveRight(curr_node):
    x = curr_node[0]
    y = curr_node[1]
    curr_node = (x,y)
    new_node=()
    new_node_x = curr_node[1]
    new_node_x+=1
    new_node = (curr_node[0],new_node_x)
    if new_node[0]>=0 and new_node[1]>=0:
        return(new_node,True)
    else:
        return(curr_node,False)
#moving down thorugh the coordinate points on the cartersian plane      
def ActionMoveDown(curr_node):
    x = curr_node[0]
    y = curr_node[1]
    curr_node = (x,y)
    new_node=()
    new_node_y = curr_node[0]
    new_node_y+=1
    new_node = (new_node_y,curr_node[1])
    if new_node[0]>=0 and new_node[1]>=0:
        return(new_node,True)
    else:
        return(curr_node,False)
#moving diagonally up and left thorugh the coordinate points on the cartersian plane  
def ActionMoveTL(curr_node):
    x = curr_node[0]
    y = curr_node[1]
    curr_node = (x,y)
    new_node=()
    new_node_y = curr_node[0]
    new_node_y-=1
    new_node_x = curr_node[1]
    new_node_x-=1
    new_node = (new_node_y,new_node_x)
    if new_node[0]>=0 and new_node[1]>=0:
        return(new_node,True)
    else:
        return(curr_node,False)
#moving diagonally up and right thorugh the coordinate points on the cartersian plane  
def ActionMoveTR(curr_node):
    x = curr_node[0]
    y = curr_node[1]
    curr_node = (x,y)
    new_node=()
    new_node_y = curr_node[0]
    new_node_y-=1
    new_node_x = curr_node[1]
    new_node_x+=1
    new_node = (new_node_y,new_node_x)
    if new_node[0]>=0 and new_node[1]>=0:
        return(new_node,True)
    else:
        return(curr_node,False)
#moving diagonally down and left thorugh the coordinate points on the cartersian plane  
def ActionMoveDL(curr_node):
    x = curr_node[0]
    y = curr_node[1]
    curr_node = (x,y)
    new_node=()
    new_node_y = curr_node[0]
    new_node_y+=1
    new_node_x = curr_node[1]
    new_node_x-=1
    new_node = (new_node_y,new_node_x)
    if new_node[0]>=0 and new_node[1]>=0:
        return(new_node,True)
    else:
        return(curr_node,False)
#moving diagonally up and right thorugh the coordinate points on the cartersian plane  
def ActionMoveDR(curr_node):
    x = curr_node[0]
    y = curr_node[1]
    curr_node = (x,y)
    new_node=()
    new_node_y = curr_node[0]
    new_node_y+=1
    new_node_x = curr_node[1]
    new_node_x+=1
    new_node = (new_node_y,new_node_x)
    if new_node[0]>=0 and new_node[1]>=0:
        return(new_node,True)
    else:
        return(curr_node,False)


# In[5]:


#getting a list of all coordinate points on the obstacle space in a list
all_points = []
for i in range(0,301): #300 width
    for j in range(201): #200 width
        all_points.append((i,j)) #appending to the list


# In[6]:


#showing what all child nodes each parent nodes are connected to
def generateGraph(start,size_x,size_y): #remember that this size_x and size_y are the sizes of the matrix, so not the end coordinates
    i = start[0] #x coordinate
    j = start[1] #y coordinate
    if i <size_x and j<size_y:
        graph={}
        #for the origin
        if i==0 and j==0: 
            graph[(i,j)]={(i+1,j+1),(i+1,j),(i,j+1)}
        #for the last point
        elif i==size_x-1 and j==size_y-1:
            graph[(i,j)]={(i-1,j),(i-1,j-1),(i,j-1)}
        #when Y =0
        elif i==size_x-1 and j ==0:
            graph[(i,j)]={(i-1,j),(i-1,j+1),(i,j+1)}
        #when X = 0
        elif j==size_y-1 and i ==0:
            graph[(i,j)]={(i,j-1),(i+1,j-1),(i+1,j)}
        #special case for points along the borders
        elif i == 0 and j!=0 and j!=size_y-1:
            graph[(i,j)]={(i,j-1),(i,j+1),(i+1,j-1),(i+1,j),(i+1,j+1)}
        #special case for points along the borders
        elif i == size_x-1 and j!=0 and j!=size_y-1:
            graph[(i,j)]={(i,j-1),(i,j+1),(i-1,j-1),(i-1,j),(i-1,j+1)}
        #special case for points along the borders
        elif j == 0 and i!=0 and i!=size_x-1:
            graph[(i,j)]={(i-1,j),(i+1,j),(i+1,j+1),(i,j+1),(i-1,j+1)}
        #special case for points along the borders
        elif j == size_y-1 and i!=0 and i!=size_x-1:
            graph[(i,j)]={(i-1,j),(i+1,j),(i+1,j-1),(i,j-1),(i-1,j-1)} 
        #for all other points
        else: 
            graph[(i,j)]={(i-1,j),(i-1,j+1),(i-1,j-1),(i+1,j-1),(i+1,j),(i+1,j+1),(i,j-1),(i,j+1)}
        #return the graph
        return(graph)
    else:
        pass


# In[7]:


#cost calculation of every point
def costCalculationatAllNodes(graph,start):
    new_dic={}#dictionary of key with node and value with the cost
    for key,value in graph.items():
        new_dic[key]={}
        for neighbour in value:
            #all possible movements from the parent
            R = ActionMoveRight(key)
            L = ActionMoveLeft(key)
            U =ActionMoveUp(key)
            D = ActionMoveDown(key)
            TL = ActionMoveTL(key)
            TR =ActionMoveTR(key)
            DL = ActionMoveDL(key)
            DR = ActionMoveDR(key)
            #checking if the point is 
            #Right away from the parent node
            #or, Left away from the parent node
            #or, above the parent node
            #or, under the parent node
            if (neighbour==R[0]) or (neighbour==L[0]) or (neighbour==U[0]) or (neighbour==D[0]):
                new_dic[key][neighbour]=1 #Assigning cost of 1 in this case
            #checking if the point is 
            #TOP RIGHT away from the parent node
            #or, TOP LEFT away from the parent node
            #or, DOWN RIGHT the parent node
            #or, DOWN LEFT the parent node
            elif (neighbour==TL[0]) or (neighbour==TR[0]) or (neighbour==DL[0]) or (neighbour==DR[0]):
                new_dic[key][neighbour]=1.414 #Assigning cost of 1.414 in this case
    return(new_dic)


# In[8]:


#empty dictionary with all the distances
all_distance = {}
#empty dictionary for backtracking from child to parent upto the start
backtracking = {}
#list of all the visited nodes
visited = []
#variable to exit out of the while loop in the dijkstraAlgorithm function
check=0
def dijkstraAlgorithm(graph,start):
    #adding the global variables
    global check
    global visited
    #when the function starts
    all_distance[start]=0
    #appending the start node to the list of visited nodes
    visited.append(start)
    #setting all nodes as infinity distance away
    for vertex,edge in graph.items():
        all_distance[vertex]=math.inf
    #starting the priority queue with the start node
    priority_queue = [(0,start)]
    #checking the length of the priority queue
    #and, inserting the while loop exit condition
    while len(priority_queue)>0 and check!=[]:
        #popping the current distance and the currenyt vertex 
        #from the priority queue
        curr_dist,curr_vert = heapq.heappop(priority_queue)
        #checking the value of the current distance and 
        if curr_dist>all_distance[curr_vert]:
            continue
        for neighbour,cost in graph[curr_vert].items():
            #Updating the cost
            distance = curr_dist + cost 
            #checking when the variable <distance> that was calculated is lesse
            #than the neighbouring cost
            if distance < all_distance[neighbour]:
                backtracking[neighbour]={}
                #adding to the backtracking dictionary
                backtracking[neighbour][distance]=curr_vert
                all_distance[neighbour]=distance
                #pushing from the priority queue
                heapq.heappush(priority_queue, (distance, neighbour))
                #checking of the neighbour is not added to the visited
                #checks what node to go to next
                if neighbour not in visited:
                    #appending to the visited list
                    visited.append(neighbour)
                    #checking if the neighbour is the goal
                    if neighbour==goal:
                        print('GOAL REACHED')
                        #changing check variable for the exit condition
                        check=[]
                        #breaking out of the loop
                        break
    #returning all_distance, visited list and backtracked dictionary
    return(all_distance,visited,backtracking)     


# In[9]:


# function to backtrack
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


# In[10]:


#Main function that solves the Djkstra algorithm and finds the obstacles
#Arguments:
# Maximum size of the x axis: Maximum_size_x,
# Maximum size of the y axis: Maximum_size_y,  
# start coordinate
# goal coordinate
#Next two arguments are exclusive to the rigid robot case : 
# radius of the rigid robot
# clearence needed to be given to the path between the robot and the obstacle
def RigidRobotdijkstra(Maximum_size_x,Maximum_size_y,start,goal,radius,clearence):
    #appending x to include the ending coordinates
    Maximum_size_x+=1 
    #appending y to include the ending coordinates
    Maximum_size_y+=1
    #getting a list of all coordinate points on the obstacle space in a list
    all_points = []
    for i in range(0,301): #300 width
        for j in range(201): #200 width
            all_points.append((i,j)) #appending to the list
    print('Length ofall_points')
    print(len(all_points))
    #points that are in the path, including the obstacle
    list_of_all_points=[]
    #empty list to store points that are in the obstacle
    map_points = [] #points that are in the shapes | obstacles
    #for every such point
    #every equation for '#formap' appends to a list that has points which only contains the 
    #physical obstacles
    #every equation for '#for path traversal' appends to a list that has points
    #which contains the points after taking the radius and the clearence into
    #consideration as well
    for c in all_points:
        x = c[0]
        y = c[1]
        
    #circle shaped obstacle
    
        #for map
        if((x-225)**2 + (y-150)**2 <= (25)**2):
            map_points.append((x,y))
            
        #for path traversal
        if((x-225)**2 + (y-150)**2 <= (25+radius+clearence)**2):
            list_of_all_points.append((x,y))
            
    #ellipse shaped obstacle
    
        #for map
        if(((x-150)**2)/40**2 +((y-100)**2)/20**2 <= 1):
            map_points.append((x,y))
            
        #for path traversal
        if(((x-150)**2)/(40+radius+clearence)**2 +((y-100)**2)/(20+radius+clearence)**2 <= 1):
            list_of_all_points.append((x,y))
            
    #complex polygon shaped obstacle
    
        #for map
        if y>=120 and y<=185 and x>=20 and x<=100:
            if y-(13*x)<=-140:
                if y-x>=100 or y+(6/5)*x>=210:
                    if y-(6/5)*x>=30:
                        if y+(7/5)*x<=290:
                            map_points.append((x,y))
                            
        #for path traversal
        if y>=120-(radius+clearence) and y<=185+radius+clearence and x>=20-(radius+clearence) and x<=100+radius+clearence:
            if y-(13*x)<=-140+radius+clearence:
                if y-x>=100-(radius+clearence) or y+(6/5)*x>=210-(radius+clearence):
                    if y-(6/5)*x>=30-(radius+clearence):
                        if y+(7/5)*x<=290+(radius+clearence):
                            list_of_all_points.append((x,y))
                            
    #rectangle slanted shaped obstacle
    
        #for map
        if y-(8/5)*x>=-122 and y+(38/65)*x<=(1254/13) and y-(9/5)*x<=13 and y+(37/65)*x>=(1093/13):
            map_points.append((x,y))
            
        #for path traversal
        if y-(8/5)*x>=-122-(radius+clearence) and y+(38/65)*x<=(1254/13)+(radius+clearence) and y-(9/5)*x<=13+(radius+clearence) and y+(37/65)*x>=(1093/13)-(radius+clearence):
            list_of_all_points.append((x,y))
            
    #rhombus shaped obstacle
    
        #for map
        if y-(3/5)*x<=-95 and y+(3/5)*x<=175 and y-(3/5)*x>=-125 and y+(3/5)*x>=145:
            map_points.append((x,y))
            
        #for path traversal
        if y-(3/5)*x<=-95+(radius+clearence) and y+(3/5)*x<=175+(radius+clearence) and y-(3/5)*x>=-125-(radius+clearence) and y+(3/5)*x>=145-(radius+clearence):
            list_of_all_points.append((x,y))
            
    #checking if the GOAL entered is within these points
    
    if goal in list_of_all_points:
        print('THE GOAL ENTERED IS WITHIN THE OBSTACLE. PLEASE RESTART AND RE-RUN')
        print('!!!!!!!!!!STOP NOW!!!!!!!!!!!!!')
    #checking the length of all the points within the obstacles itself
    print(' Length of map_points with the shape is : ')
    print(len(map_points))

    #checking the length of all the points that are UNTRAVERSABLE
    print(' Length of list_of_all_points with the shape is : ')
    print(len(list_of_all_points))
    
    #generating the base graph of all the coordinates
    base_graph = {}
    for i in range(Maximum_size_x-1,-1,-1):
        for j in range(Maximum_size_y-1,-1,-1):
            graph = generateGraph((i,j),Maximum_size_x,Maximum_size_y)
            base_graph[(i,j)]=graph[(i,j)]
    
    #checking the length of this graph
    print('Length of base_graph BEFORE removing')
    print(len(base_graph))
    
    
    #removing all the coordinates that are within the points in the obtsacle and all that
    #are connected to it as well
    for key,value in base_graph.items():
        value_copy = value.copy()
        for coordinates in value_copy:
            if coordinates in list_of_all_points:
                value.remove(coordinates) 
    base_graph_copy=base_graph.copy()
    for key,value in base_graph_copy.items():
        if key in list_of_all_points:
            del base_graph[key]
    
    #checking this length of all the points again
    #but, now with updated coordinates.
    #SHOULD be lesser than before, because total possible
    #traversable coordinates have reduced
    print('Length of base_graph AFTER removing')
    print(len(base_graph))
    #checking all the costs
    costs_calculated = costCalculationatAllNodes(base_graph,start)
    actual_graph = costs_calculated
    #empty dictionary with all the distances
    all_distance = {}
    #empty dictionary for backtracking from child to parent upto the start
    backtracking = {}
    #list of all the visited nodes
    visited = []
    #variable to exit out of the while loop in the dijkstraAlgorithm function
    #returning all the essential lists after calculating using
    #dijkstraAlgorithm
    all_distance,visited,backtracking= dijkstraAlgorithm(actual_graph,start) #can alter the start here
    #creating a copy so that the dictionary can be modified
    all_distance_copy = all_distance.copy()
    for k,v in all_distance_copy.items():
        if all_distance_copy[k] == math.inf:
            del all_distance[k]
    #returning all_distance, backtracking and list_of_all_points
    return(all_distance,visited,backtracking,map_points)


# In[11]:


#Taking inputs
x_start= int(input("Enter the x coordinate of the start:  "))
y_start= int(input("Enter the y coordinate of the start:  "))
x_goal= int(input("Enter the x coordinate of the goal:  "))
y_goal= int(input("Enter the y coordinate of the goal:  "))
radius= int(input("Enter the radius of the robot:  "))
clearence= int(input("Enter the clearance of the robot: "))


# In[12]:


#have to be user defined inputs

start = (x_start,y_start) #5,5
goal =  (x_goal,y_goal)   #295,195
Maximum_size_x = 300
Maximum_size_y = 200
all_distance,visited,backtrack,listofallpointsformap= RigidRobotdijkstra(Maximum_size_x,Maximum_size_y,start,goal,radius,clearence) #l is the list of points in the obstacle

# :: :: :: :: NOTE :: :: :: :: 
#a = all the distances of every node from every other node
#v = list of all visited nodes
#b = dictionary of all backtracked elements


# In[13]:


#Backtracking 
backtracked_final = BackTrack(backtrack,start,goal)
print(backtracked_final)
#printing the final time for completion
print("Total Time Taken : ",time.time() - start_time, "seconds")


# In[14]:


#defining a blank canvas
new_canvas = np.zeros((201,301,3),np.uint8) 
#for every point that belongs within the obstacle
for c in listofallpointsformap: #change the name of the variable l
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
            #pygame.time.wait(1)
            #time.sleep(0.000005)
            x = path[0]
            y = abs(200-path[1])
            #gameDisplay.blit(surf, (x,y))
            #gameDisplay.fill(white)
            pygame.draw.rect(gameDisplay, white, [x,y,1,1])
            #pygame.time.wait(1)
            pygame.display.flip()
    for path in backtracked_final:
        
        pygame.time.wait(5)
        #time.sleep(0.00005)
        x = path[0]
        y = abs(200-path[1])
        #gameDisplay.blit(surf, (x,y))
        #gameDisplay.fill(white)
        pygame.draw.rect(gameDisplay, (0,0,255), [x,y,1,1])
        #pygame.time.wait(1)
        pygame.display.flip()
        
    #pygame.time.wait(14)
    done = True
pygame.quit()


# In[ ]:


#visited path
for path in visited:
    #print(path)
    x = path[0]
    y = path[1]
    new_canvas_copy_backtrack[(200-y,x)]=[255,0,0] #setting every backtracked pixel to white
#showing the final backtracked path
new_backtracked = cv2.resize(new_canvas_copy_backtrack,(600,400))
cv2.imshow('visited',new_backtracked)
cv2.waitKey(0)
cv2.destroyAllWindows()


# In[ ]:


#backtracked path
for path in backtracked_final:
    x = path[0]
    y = path[1]
    new_canvas_copy_backtrack[(200-y,x)]=[0,255,0] #setting every backtracked pixel to green
#showing the final backtracked path
new_backtracked = cv2.resize(new_canvas_copy_backtrack,(600,400))
cv2.imshow('new_backtracked',new_backtracked)
cv2.waitKey(0)
cv2.destroyAllWindows()
