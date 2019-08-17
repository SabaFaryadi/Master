import os, sys
import time
from collections import defaultdict, deque
import numpy as np
import matplotlib.pyplot as plt
import math

def CordinateGenerator(w,l):
    # l= The number of horizontal nodes
    # w= The number of vertical nodes
    a=100 #distance between each pair of horizontal nodes
    b=100 #distance between each pair of vertical nodes
    p=w*l
    x = np.zeros(p, dtype=float)
    y = np.zeros(p, dtype=float)

    Rx=100
    Ry=0
    n=0

    for i in range(0, w):
        for j in range(0, l):
            if j == 0:
                Rx = Rx
            else:
                Rx = Rx + a
            x[n] = Rx
            y[n] = Ry
            n = n + 1
        Rx =100
        Ry = (i + 1) * (b)
    return x, y

if __name__ == '__main__':
    # Generate all nodes and edges
    X, Y = CordinateGenerator(3,4)
    plt.scatter(X, Y)
    i = 0
    for p in range(0, len(X)):
        plt.annotate(i, xy=(X[p], Y[p]), color='k', size=10)
        i = i + 1
    print(p)
    plt.show()