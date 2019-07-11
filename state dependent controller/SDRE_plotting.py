import os, sys
import time
from collections import defaultdict, deque
import numpy as np
import matplotlib.pyplot as plt
import math

def CordinateGenerator(w,l,seg):
    a=150
    b=100
    c=100
    d=100
    p=w*l*seg
    x = np.zeros(p, dtype=float)
    y = np.zeros(p, dtype=float)

    Rx=0
    Ry=0
    n=0

    for i in range(0, w):
        for j in range(0, l):
            if j == 0:
                Rx = Rx
            else:
                Rx = Rx + (d)
            x[n] = Rx
            y[n] = Ry
            n = n + 1

            Ry = Ry + b
            x[n] = Rx
            y[n] = Ry
            n = n + 1

            Rx = Rx + a
            x[n] = Rx
            y[n] = Ry
            n = n + 1

            Ry = Ry - b
            x[n] = Rx
            y[n] = Ry
            n = n + 1
        Rx = 0
        Ry = (i + 1) * (b + c)
    return x, y

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

    for k in range(0,w):
        for g in range(1,l):
            m[(4*g-1)+4*k*l,(4*g-1)+4*k*l+1]=d
            m[(4*g - 1) + 4 * k*l+1, (4 * g - 1) + 4 * k*l ] = d

    for k in range(0,w):
        for g in range(1,l):
            m[(4*g-2)+4*k*l,(4*g-2)+4*k*l+3]=d
            m[(4*g - 2) + 4 * k*l+3, (4 * g - 2) + 4 * k*l ] = d
    return m

class Graph(object):
    def __init__(self):
        self.nodes = set()
        self.edges = defaultdict(list)
        self.distances = {}

    def add_node(self, value):
        self.nodes.add(value)

    def add_edge(self, from_node, to_node, distance):
        self.edges[from_node].append(to_node)
        self.distances[(from_node, to_node)] = distance

if __name__ == '__main__':
    # Generate all nodes and edges
    graph = Graph()
    points=MatrixMaker(2,4,4,1)
    X,Y=CordinateGenerator(2,4,4)

    rowNumber = 0
    columnNumber = 0
    priorityValue = dict()
    for row in points:
        graph.add_node(rowNumber)
        rowNumber = rowNumber + 1
    n = rowNumber
    print(n)
    for i in range(0, n):
        for j in range(0, n):
            if points[i, j] != 0:
                graph.add_edge(i, j, points[i, j])
    for (i,j)in graph.distances:
        plt.plot([X[i],X[j]],[Y[i],Y[j]])
    i=0
    for p in range(0, len(X)):
        plt.annotate(i, xy=(X[p], Y[p]), color='k', size=10)
        i = i + 1
    print(graph.distances)
    plt.show()