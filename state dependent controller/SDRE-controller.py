#A state dependent controller for create2
# Adjust your ROOMBA_PORT if necessary
# python SDRE-Roomba-Controller.py
# the distance (diameter) between the two wheels is 258mm
# keep in mind that the robot's physical diameter is larger ~
# 0.5*258 == 129mm radius

import os, sys
#import create
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
    a=150
    b=100
    c=100
    d=100
    p=w*l*seg
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
        Rx = 100
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
    robotSpeed=[20,20]
    #robotCharge=[100,100]
    for j in range(0, n):
        if j == position_Robot1 or j == position_Robot2:
            continue
        else:
            Robot1_distance = (shortest_path(graph, position_Robot1, j))
                #print('robot1 to node', j, '=', (Robot1_distance[0]))

            Robot2_distance = (shortest_path(graph, position_Robot2, j))
                #print('robot2 to node', j, '=', Robot2_distance[0])
            t1=(Robot1_distance[0])/robotSpeed[0]
            t2=(Robot2_distance[0])/robotSpeed[1]

            if t1 <=t2:

                #if ((Robot1_distance[0])/robotSpeed[0])<robotCharge[0]:
                subnodes_Robot1.append(j)
            #elif ((Robot1_distance[0])/robotSpeed[0])<robotCharge[1]:
                    #subnodes_Robot2.append(j)
            elif t2<t1:
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
        distance = (shortest_path(graph, position_Robot, vertex))
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
                distance = (shortest_path(graph, neighbor, vertex))
                costNeighbor_Robot.append(cost_function(priorityValue[vertex], distance[0]))
        totalCost_voronoi_Robot[neighbor]=sum(costNeighbor_Robot)

        next_position=min(totalCost_voronoi_Robot.items(), key=lambda x: x[1])[0]
    if initialCost_Robot<=totalCost_voronoi_Robot[next_position]:
        nextBest_position=position_Robot
    else:
        nextBest_position=next_position
    return nextBest_position,initialCost_Robot
# STATE DEPENDENT RICCATI EQUATION TO CONTROL THE ROBOT TO MOVE ALONG THE LINE
def SDRE(X_path,Y_path):
    delta_t =0.1
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
            if x==x_final:
                x_final=x_final+0.001

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
                R = 0.01* np.eye(2, dtype=int)
                K,S,E = controlpy.synthesis.controller_lqr(A, B.transpose(), Q, R)
                u=np.array([x_initial[j,time],y_initial[j,time],tetha[j,time],zd1[j,time],zd2[j,time],yd1[j,time],yd2[j,time],wd1[j,time]])
                U=np.dot(-K,u.transpose())

                v[j,time]=U[0]#Linear velocity
                w[j,time]=U[1]#Angular velocity

                x_initial[j, time + 1] = x_initial[j, time] + delta_t * (U[0] * math.cos(tetha[j, time]))
                y_initial[j, time + 1] = y_initial[j, time] + delta_t * (U[0] * math.sin(tetha[j, time]))
                tetha[j, time + 1] = tetha[j, time] + delta_t * U[1]
                zd1[j, time + 1] = zd1[j, time] + delta_t * zd2[j, time]
                zd2[j, time + 1] = zd2[j, time]
                yd1[j, time + 1] = yd1[j, time] + delta_t * yd2[j, time]
                yd2[j, time + 1] = yd2[j, time]
                wd1[j, time + 1] = wd1[j, time]


                '''px,py,th=robot.getPose() # Get current position of the robot
                x_initial[j,time+1]=px
                y_initial[j,time+1]=py
                tetha[j,time+1]=th
                zd1[j,time+1]=zd1[j,time]+delta_t*zd2[j,time]
                zd2[j,time+1]=zd2[j,time]
                yd1[j,time+1]=yd1[j,time]+delta_t*yd2[j,time]
                yd2[j,time+1]=yd2[j,time]
                wd1[j,time+1]=wd1[j,time]'''
    return x_initial,y_initial,v,w

# Converting linear and angular velocities to the left and right velocities
def Inverse_Kinematics(V,W,d):
    V_right=V+W*(d/2)
    V_left=V-W*(d/2)
    return V_right,V_left

if __name__ == '__main__':
    # Generate all nodes and edges
    graph = Graph()
    points = MatrixMaker(5, 4, 4, 1)
    X, Y = CordinateGenerator(5, 4, 4)
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
        priorityValue[i] = 0.01
#Set values to use in cost function
    priorityValue[goal1] = 10000000
    priorityValue[goal2] = 100000

    step = list()
    Cost = list()
    path_Robot1 = list()
    path_Robot2 = list()
    i = 0
    for p in range(1, 22):

        if p == 1:# finding first partitioning befor robots move
            voronoiSubsets = Voronoi(n, position_Robot1, position_Robot2)
            initial_subnodes_Robot1 = voronoiSubsets[0]
            initial_subnodes_Robot2 = voronoiSubsets[1]
            X_Robot1 = list()
            Y_Robot1 = list()
            for n in initial_subnodes_Robot1:
                X_Robot1.append(X[n])
                Y_Robot1.append(Y[n])
            X_Robot2 = list()
            Y_Robot2 = list()
            for n in initial_subnodes_Robot2:
                X_Robot2.append(X[n])
                Y_Robot2.append(Y[n])
            #Plot first partitioning
            plt.plot(X_Robot1, Y_Robot1, 'rs', markersize=12, label='Robot1_first partition')
            plt.plot(X_Robot2, Y_Robot2, 'bs', markersize=12, label='Robot2_first partition')
            plt.plot(X[position_Robot1], Y[position_Robot1], 'cs', markersize=20)
            plt.plot(X[position_Robot2], Y[position_Robot2], 'cs', markersize=20)

            for x in range(0, len(X)):
                if x == position_Robot1:
                    plt.annotate('RoombaA', xy=(X[position_Robot1], Y[position_Robot1]), size=10)
                    plt.annotate(i, xy=(X[x], Y[x]), color='c', size=1)
                elif x == position_Robot2:
                    plt.annotate('RoombaB', xy=(X[position_Robot2], Y[position_Robot2]), size=10)
                    plt.annotate(i, xy=(X[x], Y[x]), color='c', size=1)

                elif x == goal1:
                    plt.annotate('goal1', xy=(X[goal1], Y[goal1]), size=12)
                    plt.annotate(i, xy=(X[x], Y[x]), color='w', size=1)
                elif x == goal2:
                    plt.annotate('goal2', xy=(X[goal2], Y[goal2]), size=12)
                    plt.annotate(i, xy=(X[x], Y[x]), color='w', size=1)

                else:
                    plt.annotate(i, xy=(X[x], Y[x]))
                i = i + 1
            plt.legend(bbox_to_anchor=(1, 1), loc=2, borderaxespad=0.)
            #plt.show()

        '''else:
            voronoiSubsets = Voronoi(n, nextBest_position1[0], nextBest_position2[0])
        subnodes_Robot1 = voronoiSubsets[0]
        subnodes_Robot2 = voronoiSubsets[1]
        if p == 1:
            nextBest_position1 = finding_nextPoint(subnodes_Robot1, position_Robot1, n)
            nextBest_position2 = finding_nextPoint(subnodes_Robot2, position_Robot2, n)

            path_Robot1.append(nextBest_position1[0])

            path_Robot2.append(nextBest_position2[0])



        else:
            if nextBest_position1[0] == goal2 or nextBest_position2[0] == goal2:
                print('stop')
            else:
                nextBest_position1 = finding_nextPoint(subnodes_Robot1, nextBest_position1[0], n)
                nextBest_position2 = finding_nextPoint(subnodes_Robot2, nextBest_position2[0], n)
                priorityValue[nextBest_position1[0]] = 0.01
                priorityValue[nextBest_position2[0]] = 0.01

        print('step:', p)
        print('nextBest_position1:', '', nextBest_position1[0], ',', nextBest_position1[1])
        print('nextBest_position2:', '', nextBest_position2[0], ',', nextBest_position2[1])
        totalCost = nextBest_position1[1] + nextBest_position2[1]
        Cost.append(totalCost)
        step.append(p)

        if path_Robot1[-1] != nextBest_position1[0]:
            path_Robot1.append(nextBest_position1[0])

        if path_Robot2[-1] != nextBest_position2[0]:
            path_Robot2.append(nextBest_position2[0])

    X_subnodes_Robot1 = list()
    Y_subnodes_Robot1 = list()
    for n in subnodes_Robot1:
        X_subnodes_Robot1.append(X[n])
        Y_subnodes_Robot1.append(Y[n])
    X_subnodes_Robot2 = list()
    Y_subnodes_Robot2 = list()
    for n in subnodes_Robot2:
        X_subnodes_Robot2.append(X[n])
        Y_subnodes_Robot2.append(Y[n])

    print(path_Robot1)
    print(path_Robot2)
    i = 0
    for x in range(0, len(X)):
        if x == position_Robot1:
            plt.annotate('RoombaA', xy=(X[position_Robot1], Y[position_Robot1]), size=12)
            plt.annotate(i, xy=(X[x], Y[x]), color='c', size=1)
        elif x == position_Robot2:
            plt.annotate('RoombaB', xy=(X[position_Robot2], Y[position_Robot2]), size=12)
            plt.annotate(i, xy=(X[x], Y[x]), color='c', size=1)
            plt.annotate(i, xy=(X[x], Y[x]), color='c', size=1)
        elif x == goal1:
            plt.annotate('goal1', xy=(X[goal1], Y[goal1]), size=12)
            plt.annotate(i, xy=(X[x], Y[x]), color='w', size=1)
        elif x == goal2:
            plt.annotate('goal2', xy=(X[goal2], Y[goal2]), size=12)
            plt.annotate(i, xy=(X[x], Y[x]), color='w', size=1)
        else:
            plt.annotate(i, xy=(X[x], Y[x]))
        i = i + 1

    plt.plot(X_subnodes_Robot1, Y_subnodes_Robot1, 'rs', markersize=12, label='Robot1_partition')
    plt.plot(X_subnodes_Robot2, Y_subnodes_Robot2, 'bs', markersize=12, label='Robot2_partition')

    plt.plot(X[position_Robot1], Y[position_Robot1], 'cs', markersize=20)
    plt.plot(X[position_Robot2], Y[position_Robot2], 'cs', markersize=20)'''
    position_Robot1=17
    position_Robot2=0
    path_Robot2 = [1,2,5,6,9,10,27,24,25,26]
    path_Robot1 = [18,21,36,37,38,39]
    X_path1 = list()
    Y_path1 = list()
    X_path1.append(X[position_Robot1])
    Y_path1.append(Y[position_Robot1])
    for element in path_Robot1:
        X_path1.append(X[element])
        Y_path1.append(Y[element])
    #plt.plot(X_path1, Y_path1, 'b--', label='Robot1_path')

    X_path2 = list()
    Y_path2 = list()
    X_path2.append(X[position_Robot2])
    Y_path2.append(Y[position_Robot2])
    for element in path_Robot2:
        X_path2.append(X[element])
        Y_path2.append(Y[element])
    #plt.plot(X_path2, Y_path2, 'r--', label='Robot2_path')

    trajectory_1 = SDRE(X_path1,Y_path1)
    trajectory_2 = SDRE(X_path2, Y_path2)
    #plt.legend(bbox_to_anchor=(1.01, 1), loc=2, borderaxespad=0.)
    plt.scatter(trajectory_1[0], trajectory_1[1],s=0.05,color='k')
    plt.scatter(trajectory_2[0], trajectory_2[1],s=0.05,color='k')
    print(trajectory_1[2][0])
    plt.xlim(0,1000)
    plt.ylim(0,900)
    plt.show()