# MatrixMaker creates an array that describes the allowed paths a robot
#   can travel given any sized area
#
# Written by Michael Buzzy 2019 April 25th

#numpy is for matrix
import numpy as np
import os

#width, the number of vertical sections. length, horizontal
w = 5
l = 8
#number of segments per sections
seg = 5

#distance between points
d=1

# Ma, our matrix. Set to a size to accomidate p the total number of points
p = (w+1)*(l+1)+w*l*(seg-1)
m = np.empty([p,p],dtype=int)

#create links between vertical bidirectional paths

n=0
for i in range(0,l):
    for j in range(0,w):
        m[n,n+seg] = d
        m[n+seg,n] = d
        n = n + seg
    n = n + 1
#this handles the last row since the numbering becomes incosistent
for i in range(0,w):
    m[n,n+1] = d
    m[n+1,n] = d
    n=n+1


#create horizontal links between bidirectional paths
n=0
for i in range(0,w):
    n=seg*i
    for j in range(1,l):
        m[n,n+1+w*seg]=d
        m[n+1+w*seg,n]=d
        n=n+1+w*seg

#this handles the last row since the numbering becomes inconsistent
n=(1+w*seg)*l
h=(1+w*seg)*(l-1)
for i in range(0,w):
    m[n,h] = d
    m[h,n] = d
    n = n+1
    h = h+seg




#create all directional paths
n=0
for i in range(0,l):
    for j in range(0,seg*w):
        m[n+1,n]=d
        n = n + 1
    n=n+1

#display results
os.remove("matrix")
np.savetxt("matrix",m,fmt="%i",delimiter=' ')
