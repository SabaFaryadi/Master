import numpy as np
import os


def MatrixMaker(w,l,seg,d):
    # M, our matrix. Set to a size to accomidate p the total number of points
    p = w*l*seg
    m = np.zeros([p, p], dtype=int)

    # create links between vertical bidirectional paths

    n = 0
    for j in range(0,w):
        for i in range(0, l):
            for j in range(0, seg-1):
                m[n, n + 1] = d
                n = n + 1
            n = n + 1

    for q in range(1,(w*l)+1):
        r=(4*q)-1
        m[r,r-3]=d
    s=0
    for section in range(1,(l*(w-1))+1):

        m[section+s,section+s+((l*seg)-1)]=d
        m[section+s + ((l * seg) - 1),section+s] = d
        s=s+3
    c=0
    for counter in range(2,(l*(w-1))+2):
        m[counter + c, counter + c + ((l * seg) + 1)] = d
        m[counter + c + ((l * seg) +1), counter + c] = d
        c = c + 3
    s=0
    for section in range(1,(w*(l-1))+1):
        m[3*section + s,3* section + s + 1] = d
        m[3*section + s + 1,3* section + s] = d
        s = s + 5
    c = 0
    for counter in range(2, (w * (l- 1)) + 2):
        m[counter + c, counter + c +3] = d
        m[counter + c + 3, counter + c] = d
        c = c + 7
    return m
if __name__ == '__main__':
    Graph=MatrixMaker(3,2,4,1)
    print(Graph)

