#This code generates the coordinates for each point generated by MatrixMaker
#   !!This code only works for one shape of path, seg=5!!
#   Written by Michael Buzzy 2019 April 26th

import numpy as np
import os
import matplotlib.pyplot as plt
#width, the number of vertical sections. length, horizontal
w = 5
l = 8
#number of segments per sections
seg = 5

#distance between points
a=1
b=1
c=1/3
d=1/3
e=1/3
f=1/3

# X,Y coordinates. Set to a size to accomidate p the total number of points
p = (w+1)*(l+1)+w*l*(seg-1)
x = np.zeros(p,dtype=float)
y = np.zeros(p,dtype=float)

#Loops through every point. Rx and Ry are the running x and y positions
#n=0 is assumed to be the origin
n=0
Rx=0
Ry=0
for i in range(0,l):
    x[n]= Rx
    y[n]= Ry
    n = n+1
    for j in range(0,w):
        Rx = Rx + c
        Ry = Ry + d
        x[n]= Rx
        y[n]= Ry
        n = n+1

        Rx = Rx + e
        x[n]= Rx
        y[n]= Ry
        n = n+1

        Ry = Ry + f
        x[n]= Rx
        y[n]= Ry
        n = n+1

        Rx = Rx - e
        x[n]= Rx
        y[n]= Ry
        n = n+1

        Rx = Rx - c
        Ry = Ry + d
        x[n]= Rx
        y[n]= Ry
        n = n+1
    Ry=0
    Rx= Rx + a

#clean up for the last row since the numbering pattern breaks
Ry = 0
for i in range(0,w+1):
    x[n]= Rx
    y[n]= Ry
    Ry=Ry+b
    n=n+1

#display results
os.remove("coordinates")
np.savetxt("coordinates",[x,y],fmt="%.2f",delimiter=' ')
print(x)
print(y)
plt.scatter(x,y)
plt.show()
