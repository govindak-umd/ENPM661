import matplotlib.pyplot as plt
import numpy as np
import math

fig, ax = plt.subplots()
XL=[]
def plot_curve(X0,Y0,Theta0,UL,UR):
    t=0
    r=0.5
    L=4
    dt=0.1
    X1=0
    Y1=0
    dtheta=0
    Theta0=3.14*Theta0/180
    Theta1=Theta0
    while t<1:
        t=t+dt
        X0=X0+X1
        Y0=Y0+Y1

        dx=r*(UL+UR)*math.cos(Theta1)*dt
        dy=r*(UL+UR)*math.sin(Theta1)*dt
        dtheta=(r/L)*(UR-UL)*dt

        X1=X1+dx
        Y1=Y1+dy
        Theta1=Theta1+0.5*dtheta

        plt.quiver(X0, Y0, X1, Y1,units='xy' ,scale=1,color= 'r',width =0.2, headwidth = 1,headlength=0)

        Xn=X0+X1
        Yn=Y0+Y1
        Thetan=180*(Theta1)/3.14
    return Xn,Yn,Thetan

#left_RPM = 5
#right_RPM = 10


actions=[[0,5],[5,0],[5,5],[0,10],[10,0],[10,10],[5,10],[10,5]]
# actions = [[0,100],[0,100],[50,100],[100,100],[50,50],[100,100],[100,50],[50,0],[100,0]]
        
for action in actions:
   X1= plot_curve(0,0,0, action[0],action[1]) # (0,0,45) hypothetical start configuration
   #find_neighbours(0, 0, 0, action[0], action[1])
   # print(X1)
   plt.scatter(X1[0],X1[1])
   # for action in actions:
   #      X2=plot_curve(X1[0],X1[1],X1[2], action[0],action[1])
   #      # print('   ',X2)
   #      plt.scatter(X2[0], X2[1])
   #      for action in actions:
   #          X3 = plot_curve(X2[0], X2[1], X2[2], action[0], action[1])
   #          plt.scatter(X3[0], X3[1])
        #print(X2)
        #plt.scatter(X2[0], X2[1])
plt.grid()

ax.set_aspect('equal')

plt.xlim(-30,30)
plt.ylim(-30,30)

plt.title('How to plot a vector in matplotlib ?',fontsize=10)

plt.show()
    
