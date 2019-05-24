#A state dependent controller for create2
# Adjust your ROOMBA_PORT if necessary
# python SDRE-Roomba-Controller.py
# the distance (diameter) between the two wheels is 258mm
# keep in mind that the robot's physical diameter is larger ~
# 0.5*258 == 129mm radius

import os, sys
import create
import time
from collections import defaultdict, deque
import numpy as np
import matplotlib.pyplot as plt
import math
import controlpy

'''ROOMBA_PORT = "/dev/ttyUSB0"

robot = create.Create(ROOMBA_PORT, BAUD_RATE=115200)
robot.toSafeMode()
MAX_FORWARD = 50  # in cm per second
MAX_ROTATION = 200  # in cm per second
SPEED_INC = 10  # increment in percent
# start 50% speed


lb_left = robot.senseFunc(create.LIGHTBUMP_LEFT)
lb_front_left = robot.senseFunc(create.LIGHTBUMP_FRONT_LEFT)
lb_center_left = robot.senseFunc(create.LIGHTBUMP_CENTER_LEFT)
lb_center_right = robot.senseFunc(create.LIGHTBUMP_CENTER_RIGHT)
lb_front_right = robot.senseFunc(create.LIGHTBUMP_FRONT_RIGHT)
lb_right = robot.senseFunc(create.LIGHTBUMP_RIGHT)'''
# dist_fun = robot.senseFunc(create.DISTANCE)

# Generating plots
# w=width, the number of vertical sections
# l=length, horizontal
# seg=number of segments per sections
# d=distance between points
def CordinateGenerator(w,l,seg):
    # distance between points
    a = 1
    b = 1
    c = 1 / 3
    d = 1 / 3
    e = 1 / 3
    f = 1 / 3

    # X,Y coordinates. Set to a size to accomidate p the total number of points
    p = (w + 1) * (l + 1) + w * l * (seg - 1)
    x = np.zeros(p, dtype=float)
    y = np.zeros(p, dtype=float)

    # Loops through every point. Rx and Ry are the running x and y positions
    # n=0 is assumed to be the origin
    n = 0
    Rx = 0
    Ry = 0
    for i in range(0, l):
        x[n] = Rx
        y[n] = Ry
        n = n + 1
        for j in range(0, w):
            Rx = Rx + c
            Ry = Ry + d
            x[n] = Rx
            y[n] = Ry
            n = n + 1

            Rx = Rx + e
            x[n] = Rx
            y[n] = Ry
            n = n + 1

            Ry = Ry + f
            x[n] = Rx
            y[n] = Ry
            n = n + 1

            Rx = Rx - e
            x[n] = Rx
            y[n] = Ry
            n = n + 1

            Rx = Rx - c
            Ry = Ry + d
            x[n] = Rx
            y[n] = Ry
            n = n + 1
        Ry = 0
        Rx = Rx + a

    # clean up for the last row since the numbering pattern breaks
    Ry = 0
    for i in range(0, w + 1):
        x[n] = Rx
        y[n] = Ry
        Ry = Ry + b
        n = n + 1
    return x,y
def MatrixMaker(w,l,seg,d):
    # M, our matrix. Set to a size to accomidate p the total number of points
    p = (w + 1) * (l + 1) + w * l * (seg - 1)
    m = np.zeros([p, p], dtype=int)

    # create links between vertical bidirectional paths

    n = 0
    for i in range(0, l):
        for j in range(0, w):
            m[n, n + seg] = d
            m[n + seg, n] = d
            n = n + seg
        n = n + 1
    # this handles the last row since the numbering becomes incosistent
    for i in range(0, w):
        m[n, n + 1] = d
        m[n + 1, n] = d
        n = n + 1

    # create horizontal links between bidirectional paths
    n = 0
    for i in range(0, w):
        n = seg * i
        for j in range(1, l):
            m[n, n + 1 + w * seg] = d
            m[n + 1 + w * seg, n] = d
            n = n + 1 + w * seg

    # this handles the last row since the numbering becomes inconsistent
    n = (1 + w * seg) * l
    h = (1 + w * seg) * (l - 1)
    for i in range(0, w):
        m[n, h] = d
        m[h, n] = d
        n = n + 1
        h = h + seg

    # create all directional paths
    n = 0
    for i in range(0, l):
        for j in range(0, seg * w):
            m[n + 1, n] = d
            n = n + 1
        n = n + 1
    return m

# Generating a directed graph for plots
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
# Dijkstra algorithm used in shortest path function
def dijkstra(graph, initial):
    visited = {initial: 0}
    path = {}
    nodes = set(graph.nodes)

    while nodes:
        min_node = None
        for node in nodes:
            if node in visited:
                if min_node is None:
                    min_node = node
                elif visited[node] < visited[min_node]:
                    min_node = node
        if min_node is None:
            break

        nodes.remove(min_node)
        current_weight = visited[min_node]

        for edge in graph.edges[min_node]:
            try:
                weight = current_weight + graph.distances[(min_node, edge)]
            except:
                continue
            if edge not in visited or weight < visited[edge]:
                visited[edge] = weight
                path[edge] = min_node

    return visited, path

# Function to find the shortest distance between every pair of nodes
def shortest_path(graph, origin, destination):
    visited, paths = dijkstra(graph, origin)
    full_path = deque()
    _destination = paths[destination]

    while _destination != origin:
        full_path.appendleft(_destination)
        _destination = paths[_destination]

    full_path.appendleft(origin)
    full_path.append(destination)

    return visited[destination], list(full_path)
# Discrete Voronoie partitioning to divide all nodes between robots based on shortest distances
# When all robots have the same distances to one node, that node is assigned to the robot with smaller index
def Voronoi(n,position_Robot1,position_Robot2):
    subnodes_Robot1 = list()
    subnodes_Robot2 = list()
    for j in range(0, n):
        if j == position_Robot1 or j == position_Robot2:
            continue
        else:
            Robot1_distance = (shortest_path(Graph, position_Robot1, j))
            Robot2_distance = (shortest_path(Graph, position_Robot2, j))
            if (Robot1_distance[0]) <=(Robot2_distance[0]):
                subnodes_Robot1.append(j)

            elif (Robot2_distance[0])<(Robot1_distance[0]):
                subnodes_Robot2.append(j)

    return subnodes_Robot1,subnodes_Robot2
# Calculate cost function for each node
def cost_function(cost,distance):
    totalCost=cost*distance
    return totalCost
# Find next point for robot to move
# The best next point for robot to move is the node where has smallest cost
def finding_nextPoint(subnodes_Robot,position_Robot,n):
    initialCost_Robot=list()
    for vertex in subnodes_Robot:
        distance = (shortest_path(Graph, position_Robot, vertex))
        initialCost_Robot.append(cost_function(priorityValue[vertex],distance[0]))
    initialCost_Robot=sum(initialCost_Robot)
    totalCost_voronoi_Robot=dict()
    Robot_neighbors=list()
    for node in range(0,n):#If there is an edge between two nodes, those are neighbors
        if points[position_Robot, node]!=0:
            Robot_neighbors.append(node)
    print ('neighbors',Robot_neighbors)
    subnodes_Robot.append(position_Robot)
    for neighbor in Robot_neighbors:
        costNeighbor_Robot = list()
        for vertex in subnodes_Robot:
            if neighbor==vertex:continue
            else:
                distance = (shortest_path(Graph, neighbor, vertex))
                costNeighbor_Robot.append(cost_function(priorityValue[vertex], distance[0]))
        totalCost_voronoi_Robot[neighbor]=sum(costNeighbor_Robot)

        next_position=min(totalCost_voronoi_Robot.items(), key=lambda x: x[1])[0]
    if initialCost_Robot<=totalCost_voronoi_Robot[next_position]:
        nextBest_position=position_Robot
    else:
        nextBest_position=next_position
    return nextBest_position,initialCost_Robot
# sTATE DEPENDENT RICCATI EQUATION TO CONTROL THE ROBOT TO MOVE ALONG THE LINE
def SDRE(X_path,Y_path):
    delta_t = 0.01
    t = 10
    x_initial = np.zeros((len(X_path)-1, (int(t / delta_t)) - 1))
    y_initial = np.zeros((len(X_path)-1, (int(t / delta_t)) - 1))
    tetha = np.zeros((len(X_path)-1, (int(t / delta_t)) - 1))
    zd1 = np.zeros((len(X_path) - 1, (int(t / delta_t)) - 1))
    zd2 = np.zeros((len(X_path) - 1, (int(t / delta_t)) - 1))
    wd1 = np.zeros((len(X_path) - 1, (int(t / delta_t)) - 1))
    yd1 = np.zeros((len(X_path) - 1, (int(t / delta_t)) - 1))
    yd2 = np.zeros((len(X_path) - 1, (int(t / delta_t)) - 1))
    v = np.zeros((len(X_path) - 1, (int(t / delta_t)) - 1))
    w = np.zeros((len(X_path) - 1, (int(t / delta_t)) - 1))
    for j in range(0,len(X_path)-1):
        x=X_path[j]
        y=Y_path[j]
        x_final = X_path[j + 1]
        y_final = Y_path[j + 1]

        if x!=x_final or y!=y_final:

            x_initial[j, 0] = x
            y_initial[j, 0] = y

            tetha[j,0]=math.pi/2

            zd1[j,0]=x_initial[j,0]
            zd2[j,0]=(x_final-x_initial[j,0])/t

            m=(y_final-y_initial[j,0])/(x_final-x_initial[j,0])
            n=y_initial[j,0]-m*x_initial[j,0]


            wd1[j,0]=math.atan(m)


            yd1[j,0]=(m*x_initial[j,0])+n
            yd2[j,0]=m*(x_final-x_initial[j,0])/t

            Q1=np.array([[1,0,0,-1,0,0,0,0],[0,1,0,0,0,-1,0,0],[0,0,1,0,0,0,0,-1]])
            Q2=Q1.transpose()
            I3=np.eye(3,dtype=int)
            Q3=np.dot(Q2,I3)
            Q=np.dot(Q3,Q1)

            for time in range(0,(int(t/delta_t))-2):
                A=np.array([[0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0],[0,0,0,0,1,0,0,0],
                            [0,0,0,0,0,0,0,0],[0,0,0,0,0,0,1,0],[0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0]])
                B=np.array([[math.cos(tetha[j,time]),math.sin(tetha[j,time]),0 ,0 ,0 ,0 ,0 ,0],[0 ,0 ,1 ,0 ,0 ,0 ,0 ,0]])
                A = A - 0.05 * np.eye(8, dtype=int)
                R = 0.0001 * np.eye(2, dtype=int)
                K,S,E = controlpy.synthesis.controller_lqr(A, B.transpose(), Q, R)
                u=np.array([x_initial[j,time],y_initial[j,time],tetha[j,time],zd1[j,time],zd2[j,time],yd1[j,time],yd2[j,time],wd1[j,time]])
                U=np.dot(-K,u.transpose())

                v[j,time]=U[0]#Linear velocity
                w[j,time]=U[1]#Angular velocity

                px,py,th=robot.getPose() # Get current position of the robot

                x_initial[j,time+1]=px
                y_initial[j,time+1]=py
                tetha[j,time+1]=th
                zd1[j,time+1]=zd1[j,time]+delta_t*zd2[j,time]
                zd2[j,time+1]=zd2[j,time]
                yd1[j,time+1]=yd1[j,time]+delta_t*yd2[j,time]
                yd2[j,time+1]=yd2[j,time]
                wd1[j,time+1]=wd1[j,time]
    return x_initial,y_initial,v,w

# Converting linear and angular velocities to the left and right velocities
def Inverse_Kinematics(V,W,d):
    V_right=V+W*(d/2)
    V_left=V-W*(d/2)
    return V_right,V_left

if __name__ == '__main__':
    # Generate all nodes and edges
    graph = Graph()
    points=MatrixMaker(3,3,5,1)
    x_cordinates,y_cordinates=CordinateGenerator(3,3,5)
    #Get the location of robots and regions of interests
    position_Robot1 = input('Enter Roomba A position:')
    position_Robot2 = input('Enter Roomba B position:')
    goal1 = input('What is the first region of interest?')
    goal2 = input('What is the second region of interest?')
    position_Robot1 = int(position_Robot1)
    position_Robot2 = int(position_Robot2)
    goal1 = int(goal1)
    goal2 = int(goal2)

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

