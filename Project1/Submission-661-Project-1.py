#!/usr/bin/env python
# coding: utf-8

# # Govind Ajith Kumar | ENPM661 | Project 1


# github repo: https://github.com/govindak-umd/ENPM661/tree/master/Project1


#Importing all the libraries
import numpy as np
import copy
import ast #for converting strings of lists, to pure lists

def PrintMatrix(k):#converting the input list to a 3 by 3 matrix
    list_of_num=np.array([k]) #to change format num from 0 to 8 and position from 0 to 9
    if len(list_of_num) ==9: #checking if there are 9 elements
        for i in range(0,len(list_of_num),3):
            k.append(list_of_num[i]) #appending to the first row
            k.append(list_of_num[i+1]) #appending to the second row
            k.append(list_of_num[i+2]) #appending to the third row
            i+=1 #incrementing counter
    k=np.reshape(k,(3,3)) #reshaping to a 3 by 3 matrix
    return(k) #outputting the 3 by 3 matrix


def BlankTileLocation(curr_node): #to find the position of zero in the matrix
    curr_node=np.reshape(curr_node,(3,3))
    for i in range(0,3):
        for j in range(0,3):
            if curr_node[i][j]==0:
                row=i#saving the row
                col=j#saving the column
    return([row,col]) #returns the row and column number of zero


def ActionMoveLeft(curr_node): #To move left
    r=BlankTileLocation(curr_node)[0]#row of zero
    c=BlankTileLocation(curr_node)[1]#column of zero
    if c!=0:
        curr_node[r][c-1],curr_node[r][c] = curr_node[r][c],curr_node[r][c-1]#swapping elements
        return(curr_node,True)#returning the current node and True, if Left is possible
    else:
        return(curr_node,False)#returning the current node and False, if Left is IMpossible


def ActionMoveRight(curr_node):#To move right
    r=BlankTileLocation(curr_node)[0]#row of zero
    c=BlankTileLocation(curr_node)[1]#column of zero
    if c!=2:
        curr_node[r][c+1],curr_node[r][c] = curr_node[r][c],curr_node[r][c+1] #swapping elements
        return(curr_node,True)#returning the current node and True, if Right is possible
    else:
        return(curr_node,False)#returning the current node and False, if Right is IMpossible

def ActionMoveUp(curr_node):#To move up
    r=BlankTileLocation(curr_node)[0]#row of zero
    c=BlankTileLocation(curr_node)[1]#column of zero
    if r!=0:
        curr_node[r-1][c],curr_node[r][c] = curr_node[r][c],curr_node[r-1][c]#swapping elements
        return(curr_node,True)#returning the current node and True, if Up is possible
    else:
        return(curr_node,False)#returning the current node and False, if Up is IMpossible

def ActionMoveDown(curr_node):#To move down
    r=BlankTileLocation(curr_node)[0]#row of zero
    c=BlankTileLocation(curr_node)[1]#column of zero
    if r!=2:
        curr_node[r+1][c],curr_node[r][c] = curr_node[r][c],curr_node[r+1][c]#swapping elements
        return(curr_node,True)#returning the current node and True, if Down is possible
    else:
        return(curr_node,False)#returning the current node and False, if Down is IMpossible
    
#self made function to convert a 3 by 3 to a full list, to append into the global list
def Mat2List(m):
    p=[]
    for i in m[0].tolist():
            p = p +i
    return p

all_inversions = [] #empty list for the function below
def check_solvability(s): #User defined Function to check the solvability of the given 8 puzzle problem 
    global all_inversions #defining a global variable
    for i in s:
        if i!=0:
            count=0
            s = s[s.index(i):] 
            for num in s:
                if num < i and num!=0:
                    count+=1
            all_inversions.append(count) #adding elements to all_inversions list
    counts=0
    for val in all_inversions:
        counts=counts + val
    if counts%2==0:
        
        return (True) #returning True, if solvable
    else:
        return (False) #returning False, if unsolvable


#Enter the start and the goal positions here, in the form of a list
#For example:
#[[2,4,3
#7,8,0
#6,1,5]]
#will be entered as : >>>> [2,4,3,7,8,0,6,1,5]
# The same is done with the desired goal posiiton

#test cases as per rubric
# s = [4,1,0,6,3,2,7,5,8]
# s = [2,5,3,1,0,6,4,7,8]
# s = [8,6,7,2,5,4,3,0,1]
s = [6,4,7,8,5,0,3,2,1]
# s = [1,4,0,6,3,2,7,5,8] 
# s = [8,7,6,2,5,4,3,0,1]
g = [1,2,3,4,5,6,7,8,0] #goal position

global_node=[] #All the nodes traversed throughout the journey by the Blank Tile
parent_node = [] #Immedietely previous node. Child node of 'layer 1' is the Parent Node of 'layer 2'
back_track = {} #Backtracking dictionary
i=0
parent_node.append([s])
goal_found = 0


val = check_solvability(s) #variable that checks the solvability of the matrix


#The function to check when the goal is reached. 
#Function also returns the path all the way from the beginning to the end, in the form of a matrix.
def generate_path(s,g): 
    
    global_node.append(s)
    global val #calling the global val variable that checks the solvability of the matrix
    global goal_found #calling the global goal_node variable
    
    if s==g: #checking if the starting and the ending points are the same
        
        print('GOAL IS REACHED BY DEFAULT!!')
        print('If not intentional, please try again?')
  
    elif s!=g:
        
        if (val == False):
                
                print('This puzzle cannot be solved into that desired goal, since its UNSOLVABLE!')
                
        else:
            
            count=1
            
            for c in parent_node: #for every element in the parent node
                
                if goal_found==[]:  #checking if thegoal is reached, when the list is empty
                    break
                    
                else:
                    
                    # print('The parent node is :>',c) #printing out the parent node
                    parent_node.append([]) #appending an empty list to the parent node, so that the nodes are filled in subsequently
                    print('The count is  >>> ', count)
                    for i in c: #for ever element, inside every list of lists in the parent node

                        l_copy=i.copy() #making a copy of the matrix, to move left
                        l=ActionMoveLeft(PrintMatrix(l_copy))  #moving left
                        l=Mat2List(l) #converting the output matrix to a list format, to be appended to the global node and the parent node mentioned above
                        u_copy=i.copy() #making a copy of the matrix, to move up
                        u=ActionMoveUp(PrintMatrix(u_copy)) #moving up
                        u=Mat2List(u)#converting the output matrix to a list format, to be appended to the global node and the parent node mentioned above
                        r_copy=i.copy() #making a copy of the matrix, to move right
                        r=ActionMoveRight(PrintMatrix(r_copy)) #moving right
                        r=Mat2List(r)#converting the output matrix to a list format, to be appended to the global node and the parent node mentioned above
                        d_copy=i.copy() #making a copy of the matrix, to move down
                        d=ActionMoveDown(PrintMatrix(d_copy)) #moving down
                        d=Mat2List(d)#converting the output matrix to a list format, to be appended to the global node and the parent node mentioned above
                        back_track[str(l_copy)] = [] #Adding a new empty value for the new Parent Node Key
                        if l not in global_node : #checking if the value is already traversed
                            back_track[str(l_copy)].append(l)#adding the Dictionary values of the respective PARENT NODE
                        if l_copy!=u_copy: #checking the equality of the duplicates, i.e: copied values
                            back_track[str(u_copy)] = []#Adding a new empty value for the new Parent Node Key
                        if u not in global_node : #checking if the value is already traversed
                            back_track[str(u_copy)].append(u)#adding the Dictionary values of the respective PARENT NODE
                        if u_copy!=r_copy: #checking the equality of the duplicates, i.e: copied values
                            back_track[str(r_copy)] = []#Adding a new empty value for the new Parent Node Key
                        if r not in global_node : #checking if the value is already traversed
                            back_track[str(r_copy)].append(r)#adding the Dictionary values of the respective PARENT NODE
                        if r_copy!=d_copy: #checking the equality of the duplicates, i.e: copied values
                            back_track[str(d_copy)] = []#Adding a new empty value for the new Parent Node Key
                        if d not in global_node : #checking if the value is already traversed
                            back_track[str(d_copy)].append(d)#adding the Dictionary values of the respective PARENT NODE
                            
                        if l not in global_node : #checking if the value is already traversed
                            global_node.append(l) #appending to the global node 
                            parent_node[count].append(l)#appending to the parent node 
                        if u not in global_node: #checking if the value is already traversed
                            global_node.append(u)#appending to the global node 
                            parent_node[count].append(u)#appending to the parent node 
                        if r not in global_node: #checking if the value is already traversed
                            global_node.append(r)#appending to the global node 
                            parent_node[count].append(r)#appending to the parent node 
                        if d not in global_node: #checking if the value is already traversed
                            global_node.append(d)#appending to the global node 
                            parent_node[count].append(d)#appending to the parent node

                    for val in global_node: #checking if any of the matrices in THIS loop is a goal
                        if val==g: #Boolean checking if case is True
                            print('GOAL REACHED!!!') #printing the goal reached notification
                            goal_found=[] #reinitializing the goal_found to an empty list

                count+=1 #incrementing the count variable


generate_path(s,g) #calling the function with test cases, mentioned above


#reversing and backtracking
val_unchanged = g #just for appending at the end
val = g #val variable, with value equal to the list of the goal matrix
goal = s#goal variable, with value equal to the list of the sOriginal starting node
back_track_list=[]#empty list where all the elements in the path of the back tracking rests
while val!=goal:
    for keys, values in back_track.items():    # for name, age in dictionary.iteritems():  (for Python 2.x)
        while val in values:
            keys= ast.literal_eval(keys) #for converting strings of lists, to pure lists
            val = keys #previous key and values of the dictionary are interchanged
            back_track_list.append(val) #appending the values to the backtracking list
back_track_list=back_track_list[::-1] #reversing the list
back_track_list.append(val_unchanged) #appending the start point to the list, to show complete paths of traversal


#Making the transpose of all the matrices, to run them in the programme
transposed_list = [] #empty transposed list
for i in back_track_list:
    m = PrintMatrix(i) 
    m = m.transpose()#doing np transform
    new=[]
    for i in m:
        for d in i:
            new.append(d)
    transposed_list.append(new) #appending to the blank list


#Contains the path from the start to the goal, in theat order
# Open the file for writing nodePath.txt 
F = open('nodePath.txt', 'w')
# List of numbers
for c in transposed_list:
    for i in c:
        F.write(str(i)+' ')
    F.write('\n')
# Close the file
F.close() 



#Contains all the Paths ever traversed by the blank tile, here it is 0
# Open the file for writing nodeInfo.txt
F = open('Nodes.txt', 'w')
# Writing all the global nodes to a list
for i in range(len(global_node)):
#     F.write(str(i))
    F.write('\t')
    F.write(str(global_node[i]))
    F.write('\n')
# Close the file
F.close() 


F = open('NodesInfo.txt', 'w')
# Writing all the global nodes to a list
c=0
for r in range(len(parent_node)):
    l = len(parent_node[r])
    for i in range(l):
        c=c+1
        F.write(str(r)+' \t'+str(c-1))
        F.write('\n')
F.write('\n')
# Close the file
F.close() 
c=0


#Just for visulaization of the forward path
print('START')
print('\n')
for i in back_track_list:
    print(PrintMatrix(i))
    print('\n')
print('GOAL')


# In[20]:


#Just for visulaization of the backwards path |BACKTRACKING
print('GOAL')
print('\n')
back_track_list=back_track_list[::-1]
for i in back_track_list:
    print(PrintMatrix(i))
    print('\n')
print('START')


