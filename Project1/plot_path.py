import numpy as np

def print_matrix(state):
    counter = 0
    for row in range(0, len(state), 3):
        if counter == 0 :
            print("-------------")
        for element in range(counter, len(state), 3):
            if element <= counter:
                print("|", end=" ")
            print(int(state[element]), "|", end=" ")
        counter = counter +1
        print("\n-------------")

fname = 'nodePath.txt'
data = np.loadtxt(fname)
if len(data[1]) is not 9:
    print("Format of the text file is incorrect, retry ")
else:
    for i in range(0, len(data)):
        if i == 0:
            print("Start Node")
        elif i == len(data)-1:
            print("Achieved Goal Node")
        else:
            print("Step ",i)
        print_matrix(data[i])
        print()
        print()