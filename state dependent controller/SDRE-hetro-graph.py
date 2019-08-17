import numpy as np
import os


def MatrixMaker(w,l,d):
    # M, our matrix. Set to a size to accomidate p the total number of points
    p = w*l
    m = np.zeros([p, p], dtype=int)

    # create links between vertical bidirectional paths

    n = 0
    for j in range(0,w):
        for i in range(0, l-1):

            m[n, n + 1] =m[n+ 1, n ] = d
            n = n + 1
        n = n + 1
    k=0
    for j in range(0,w-1):
        for i in range(0, l):
            m[k,k+l]=m[k+l,k]=d
            k=k+1

    return m
if __name__ == '__main__':
    Graph=MatrixMaker(3,6,1)
    print(Graph)

