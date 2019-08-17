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
    points=MatrixMaker(9,9,1)
    X,Y=CordinateGenerator(9,9)
    plt.plot(X, Y, 'cs', markersize=10)
    # plt.scatter(X,Y)
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
        plt.plot([X[i],X[j]],[Y[i],Y[j]],color='r')
    i=0
    for p in range(0, len(X)):
        plt.annotate(i, xy=(X[p], Y[p]), color='k', size=10)
        i = i + 1
    print(graph.distances)
    plt.xlabel('X Position',size=12)
    plt.ylabel('Y Position', size=12)
    plt.show()