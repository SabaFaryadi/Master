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
from scipy import integrate
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
def CordinateGenerator(w,l):
    # l= The number of horizontal nodes
    # w= The number of vertical nodes
    a=60 #distance between each pair of horizontal nodes
    b=60 #distance between each pair of vertical nodes
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
    m = np.zeros([p, p], dtype=float)

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
def Voronoi(n,position_Robot1,position_Robot2,graph1,graph2):
    subnodes_Robot1 = list()
    subnodes_Robot2 = list()

    for j in range(0, n):
        if j == position_Robot1 or j == position_Robot2:
            continue
        else:
            Robot1_distance = (shortest_path(graph1, position_Robot1, j))
            Robot2_distance = (shortest_path(graph2, position_Robot2, j))

            if Robot1_distance <=Robot2_distance:
                subnodes_Robot1.append(j)
            elif Robot2_distance<Robot1_distance:
                subnodes_Robot2.append(j)

    return subnodes_Robot1,subnodes_Robot2
def cost_function(cost,distance):
    totalCost=cost*distance
    return totalCost
# Find next point for robot to move
# The best next point for robot to move is the node where has smallest cost
def finding_nextPoint(subnodes_Robot,position_Robot,n,graph,point_Robot):
    initialCost_Robot=list()
    for vertex in subnodes_Robot:
        distance = (shortest_path(graph, position_Robot, vertex))
        initialCost_Robot.append(cost_function(priorityValue[vertex],distance[0]))
    initialCost_Robot=sum(initialCost_Robot)
    totalCost_voronoi_Robot=dict()
    Robot_neighbors=list()
    for node in range(0,n):#If there is an edge between two nodes, those are neighbors
        if point_Robot[position_Robot, node]!=0:
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

#defining saturation function to make limitation for velocity
def velocitySaturation(Vi,Vlim):
    if Vi <= -Vlim:
        SaturatedV=-Vlim
    elif Vi >= Vlim:
        SaturatedV = Vlim
    elif Vi <= Vlim and Vi >= -Vlim:
        SaturatedV=Vi
    return SaturatedV


def SDRE(X_path,Y_path,Vsm,dt,Total_t):
    Vlim = Vsm
    delta_t = dt
    t = Total_t
    x_initial = np.zeros((len(X_path)-1, (int(t / delta_t)) - 1))
    y_initial = np.zeros((len(X_path)-1, (int(t / delta_t)) - 1))
    tetha = np.zeros((len(X_path)-1, (int(t / delta_t)) - 1))
    zd1 = np.zeros((len(X_path) - 1, (int(t / delta_t)) - 1))
    zd2 = np.zeros((len(X_path) - 1, (int(t / delta_t)) - 1))
    wd1 = np.zeros((len(X_path) - 1, (int(t / delta_t)) - 1))
    yd1 = np.zeros((len(X_path) - 1, (int(t / delta_t)) - 1))
    yd2 = np.zeros((len(X_path) - 1, (int(t / delta_t)) - 1))
    v = np.zeros((len(X_path) - 1, (int(t / delta_t)) - 1))
    Vs= np.zeros((len(X_path) - 1, (int(t / delta_t)) - 1))
    Vi = np.zeros((len(X_path) - 1, (int(t / delta_t)) - 1))
    Ud= np.zeros((len(X_path) - 1, (int(t / delta_t)) - 1))
    w = np.zeros((len(X_path) - 1, (int(t / delta_t)) - 1))
    jad= np.zeros((len(X_path) - 1, (int(t / delta_t)) - 1))
    jad2 = np.zeros((len(X_path) - 1, (int(t / delta_t)) - 1))
    J= np.zeros((len(X_path) - 1, (int(t / delta_t)) - 1))
    Tt=np.zeros((len(X_path) - 1, (int(t / delta_t)) - 1))

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

            tetha[j,0]=math.atan((y_final-y)/(x_final-x))

            zd1[j,0]=x_initial[j,0]
            zd2[j,0]=(x_final-x_initial[j,0])/t


            m=(y_final-y_initial[j,0])/(x_final-x_initial[j,0])
            n=y_initial[j,0]-m*x_initial[j,0]

            wd1[j,0]=math.atan(m)


            yd1[j,0]=(m*x_initial[j,0])+n
            yd2[j,0]=m*(x_final-x_initial[j,0])/t

            Q1=np.array([[1,0,0,-1,0,0,0,0,0],[0,1,0,0,0,-1,0,0,0],[0,0,1,0,0,0,0,-1,0]])
            Q2=Q1.transpose()
            I3=np.eye(3,dtype=int)
            Q3=np.dot(Q2,I3)
            Q=np.dot(Q3,Q1)
            Vi[j,0]=0

            for step in range(0,(int(t/delta_t)-2)):

                if Vi[j,step]==0:
                    Vs[j,step]=1
                else:
                    Vs[j, step]=velocitySaturation(Vi[j,step],Vlim)/Vi[j,step]
                A=np.array([[0,0,0,0,0,0,0,0,math.cos(tetha[j,step])*Vs[j, step]],[0,0,0,0,0,0,0,0,math.sin(tetha[j,step])*Vs[j, step]],
                            [0,0,0,0,0,0,0,0,0],[0,0,0,0,1,0,0,0,0],[0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,1,0,0],[0,0,0,0,0,0,0,0,0],
                            [0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0]])
                B=np.array([[0,0 ,0 ,0 ,0 ,0 ,0,0,1],[0,0,1,0,0,0,0,0,0]])
                A= A - 0.5 * np.eye(9, dtype=int)
                R = 0.01* np.eye(2, dtype=int)
                K,S,E = controlpy.synthesis.controller_lqr(A, B.transpose(), Q, R)
                u=np.array([x_initial[j,step],y_initial[j,step],tetha[j,step],zd1[j,step],zd2[j,step],yd1[j,step],yd2[j,step],wd1[j,step],Vi[j,step]])
                Ud[j,step]=velocitySaturation(Vi[j,step],Vlim)
                U=np.dot(-K,u.transpose())

                jad[j, step] =np.dot(u,(np.dot(Q,u.transpose())))
                jad2[j, step] =np.dot(U.transpose(),(np.dot(R,U)))

                J[j,step]=(math.exp(-2*0.5*delta_t*(step-1)))*(np.dot(u,(np.dot(Q,u.transpose())))+np.dot(U.transpose(),(np.dot(R,U))))

                v[j,step]=U[0]#Linear velocity
                w[j,step]=U[1]#Angular velocity

                x_initial[j, step + 1] = x_initial[j, step] + Vs[j,step] * delta_t * (Vi[j,step] * math.cos(tetha[j, step]))
                y_initial[j, step + 1] = y_initial[j, step] + Vs[j,step] *delta_t * (Vi[j,step] * math.sin(tetha[j, step]))
                tetha[j, step + 1] = tetha[j, step] + delta_t * w[j,step]
                Vi[j,step+1]=Vi[j,step]+delta_t*v[j,step]
                zd1[j, step + 1] = zd1[j, step] + delta_t * zd2[j, step]
                zd2[j, step + 1] = zd2[j, step]
                yd1[j, step + 1] = yd1[j, step] + delta_t * yd2[j, step]
                yd2[j, step + 1] = yd2[j, step]
                wd1[j, step + 1] = wd1[j, step]

                Tt[j,step]=delta_t*(step-1)

    return x_initial,y_initial,Ud,w,J,Tt,Ud

# Converting linear and angular velocities to the left and right velocities
def Inverse_Kinematics(V,W,d):
    V_right=V+W*(d/2)
    V_left=V-W*(d/2)
    return V_right,V_left

if __name__ == '__main__':
    # Generate all nodes and edges
    w=6
    l=15
    X, Y = CordinateGenerator(w, l)
    path=[0,1]
    X_path= list()
    Y_path= list()
    priorityValue = dict()
    Vs1=9.3
    Vs2 =14
    T=0.08
    dtime=6
    position_Robot1 = 7
    position_Robot2 = 8
    for element in path:
        X_path.append(X[element])
        Y_path.append(Y[element])

    weight_Robot1=SDRE(X_path,Y_path,Vs1,T,dtime)
    weight_Robot2=SDRE(X_path,Y_path,Vs2,T,dtime)

    Time_Robot1= weight_Robot1[5][0]
    JJ_Robot1 = weight_Robot1[4][0]
    H_Robot1 = list()
    for t in range(0, len(Time_Robot1)):
        H_Robot1.append(np.trapz(JJ_Robot1[0:t], x=Time_Robot1[0:t]))

    Time_Robot2 = weight_Robot2[5][0]
    JJ_Robot2 = weight_Robot2[4][0]
    H_Robot2 = list()
    for t in range(0, len(Time_Robot2)):
        H_Robot2.append(np.trapz(JJ_Robot2[0:t], x=Time_Robot2[0:t]))

    graph_Robot1 = Graph()
    points_Robot1 = MatrixMaker(w, l,max(H_Robot1))
    rowNumber = 0
    columnNumber = 0
    for row in points_Robot1:
        graph_Robot1.add_node(rowNumber)
        rowNumber = rowNumber + 1
    n_Robot1 = rowNumber
    print(n_Robot1)
    for i in range(0, n_Robot1):
        for j in range(0, n_Robot1):
            if points_Robot1[i, j] != 0:
                graph_Robot1.add_edge(i, j, points_Robot1[i, j])

    graph_Robot2 = Graph()
    points_Robot2 = MatrixMaker(w, l, max(H_Robot2))
    rowNumber = 0
    columnNumber = 0
    for row in points_Robot2:
        graph_Robot2.add_node(rowNumber)
        rowNumber = rowNumber + 1
    n_Robot2 = rowNumber
    #print(n_Robot2)
    for i in range(0, n_Robot2):
        for j in range(0, n_Robot2):
            if points_Robot2[i, j] != 0:
                graph_Robot2.add_edge(i, j, points_Robot2[i, j])
        priorityValue[i] =1

    '''priorityValue[56]=10
    priorityValue[47]=priorityValue[55]=priorityValue[57]=priorityValue[65]=40
    priorityValue[54]=priorityValue[46]=priorityValue[38]=priorityValue[48]=priorityValue[58]=priorityValue[66]=priorityValue[74]=priorityValue[64]=0.1

    priorityValue[34] =1000
    priorityValue[33]=priorityValue[43]=priorityValue[35]=priorityValue[25]=1
    priorityValue[42] = priorityValue[44] = priorityValue[24] = priorityValue[26] =0.1'''

    priorityValue[32]=500
    #priorityValue[56]=20
    priorityValue[17]=priorityValue[47]=priorityValue[31]=priorityValue[33]=150
    #priorityValue[71]=priorityValue[41]=priorityValue[57]=priorityValue[55]=15
    priorityValue[16]=priorityValue[46]=priorityValue[18]=priorityValue[48]=100
    #priorityValue[42]=priorityValue[72]=priorityValue[40]=priorityValue[70]=10
    priorityValue[34]=priorityValue[30]=priorityValue[0]=priorityValue[62]=50
    #priorityValue[26]=priorityValue[86]=priorityValue[54]=priorityValue[58]=5
    '''priorityValue[47]=500
    priorityValue[46] =priorityValue[48]=priorityValue[32]=priorityValue[62]=100
    priorityValue[31] =priorityValue[61]=priorityValue[33]=priorityValue[63]=50
    priorityValue[45] =priorityValue[49]=priorityValue[17]=priorityValue[77]=10'''
    priorityValue[69] =10
    priorityValue[54] =priorityValue[84] =priorityValue[68] =priorityValue[70] =5
    priorityValue[84] =priorityValue[82] =priorityValue[54] =priorityValue[52] =2

    print('max cost1=',max(H_Robot1))
    print('max cost2=', max(H_Robot2))
    step = list()
    Cost = list()
    path_Robot1 = list()
    path_Robot2 = list()
    i = 0
    # Finding first partitioning based on Voronoi algorithm
    for p in range(1, 15):

        if p == 1:
            voronoiSubsets = Voronoi(n_Robot1, position_Robot1, position_Robot2, graph_Robot1, graph_Robot2)
            initial_subnodes_Robot1 = voronoiSubsets[0]
            initial_subnodes_Robot2 = voronoiSubsets[1]
            #Plotting first Voronoi-partitioning
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

                #elif x == goal1:
                   # plt.annotate('goal1', xy=(X[goal1], Y[goal1]), size=12)
                   # plt.annotate(i, xy=(X[x], Y[x]), color='w', size=1)
                #elif x == goal2:
                    #plt.annotate('goal2', xy=(X[goal2], Y[goal2]), size=12)
                   # plt.annotate(i, xy=(X[x], Y[x]), color='w', size=1)
                else:
                    plt.annotate(i, xy=(X[x], Y[x]))
                i = i + 1
            plt.legend(bbox_to_anchor=(1, 1), loc=2, borderaxespad=0.)
            plt.show()

        else:
            voronoiSubsets = Voronoi(n_Robot1, nextBest_position1, nextBest_position2,graph_Robot1,graph_Robot2)
        subnodes_Robot1 = voronoiSubsets[0]
        subnodes_Robot2 = voronoiSubsets[1]
        if p == 1:
            next_point1=finding_nextPoint(subnodes_Robot1, position_Robot1, n_Robot1,graph_Robot1,points_Robot1)
            next_point2=finding_nextPoint(subnodes_Robot2, position_Robot2, n_Robot1,graph_Robot2,points_Robot2)
            nextBest_position1 = next_point1[0]
            nextBest_position2 = next_point2[0]

        #else:
            #if nextBest_position1== goal2 or nextBest_position2== goal2:
                #print('stooooooppppppp')
                #break
        else:
            next_point1 = finding_nextPoint(subnodes_Robot1, nextBest_position1, n_Robot1,graph_Robot1,points_Robot1)
            next_point2 = finding_nextPoint(subnodes_Robot2, nextBest_position2, n_Robot1,graph_Robot2,points_Robot2)
            nextBest_position1 = next_point1[0]
            nextBest_position2 = next_point2[0]
            #priorityValue[nextBest_position1] = 0.01
            #priorityValue[nextBest_position2] = 0.01

        print('step:', p)
        print('nextBest_position1:', '', nextBest_position1, ',', next_point1[1])
        print('nextBest_position2:', '', nextBest_position2, ',', next_point2[1])
        totalCost = next_point1[1] + next_point2[1]
        Cost.append(totalCost)
        step.append(p)
        # if nextBest_position1[0] not in path_Robot1:
        path_Robot1.append(nextBest_position1)

        # if nextBest_position2[0] not in path_Robot2:
        path_Robot2.append(nextBest_position2)

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
    # img = plt.imread('farm2.png')
    # fig, ax = plt.subplots()
    # ax.imshow(img, extent=[120, 570, -50, 350])
    for x in range(0, len(X)):
        if x == position_Robot1:
            plt.annotate('RoombaA', xy=(X[position_Robot1], Y[position_Robot1]), size=12)
            plt.annotate(i, xy=(X[x], Y[x]), color='c', size=1)
        elif x == position_Robot2:
            plt.annotate('RoombaB', xy=(X[position_Robot2], Y[position_Robot2]), size=12)
            plt.annotate(i, xy=(X[x], Y[x]), color='c', size=1)
            plt.annotate(i, xy=(X[x], Y[x]), color='c', size=1)
        #elif x == goal1:
            #plt.annotate('goal1', xy=(X[goal1], Y[goal1]), size=12)
            #plt.annotate(i, xy=(X[x], Y[x]), color='w', size=1)
        #elif x == goal2:
            #plt.annotate('goal2', xy=(X[goal2], Y[goal2]), size=12)
            #plt.annotate(i, xy=(X[x], Y[x]), color='w', size=1)
        else:
            plt.annotate(i, xy=(X[x], Y[x]))
        i = i + 1

    plt.plot(X_subnodes_Robot1, Y_subnodes_Robot1, 'rs', markersize=12, label='Robot1_partition')
    plt.plot(X_subnodes_Robot2, Y_subnodes_Robot2, 'bs', markersize=12, label='Robot2_partition')

    plt.plot(X[position_Robot1], Y[position_Robot1], 'cs', markersize=20)
    plt.plot(X[position_Robot2], Y[position_Robot2], 'cs', markersize=20)

    X_path1 = list()
    Y_path1 = list()
    X_path1.append(X[position_Robot1])
    Y_path1.append(Y[position_Robot1])
    for element in path_Robot1:
        X_path1.append(X[element])
        Y_path1.append(Y[element])
    #plt.plot(X_path1, Y_path1, 'r', label='Robot1_path')
    X_path2 = list()
    Y_path2 = list()
    X_path2.append(X[position_Robot2])
    Y_path2.append(Y[position_Robot2])
    for element in path_Robot2:
        X_path2.append(X[element])
        Y_path2.append(Y[element])
    trajectory_1 = SDRE(X_path1, Y_path1,Vs1,T,dtime)
    #print('time1:', time.time() - TimerStart)

    TimerStart = time.time()
    trajectory_2 = SDRE(X_path2, Y_path2,Vs2,T,dtime)
    #print('time2:', time.time() - TimerStart)

    # plt.legend(bbox_to_anchor=(1.01, 1), loc=2, borderaxespad=0.)
    plt.scatter(trajectory_1[0], trajectory_1[1], color='k', s=0.08)
    plt.scatter(trajectory_2[0], trajectory_2[1], color='k', s=0.08)

    plt.legend(bbox_to_anchor=(1.01, 1), loc=2, borderaxespad=0.)
    plt.xlim(10,1000)
    plt.show()
    #plt.plot(step, Cost)
    #plt.xlabel('Iterations')
    #plt.ylabel('Total Cost')
    #plt.show()
    #plt.plot(trajectory_1[6][0])
    #plt.plot(trajectory_2[6][0])
    #plt.legend(bbox_to_anchor=(1.01, 1), loc=2, borderaxespad=0.)
    #plt.show()
    print(max(H_Robot1))
    print(max(H_Robot2))
    '''print('weight=', max(H_Robot2))
    for i in range(0, n_Robot2):
        for j in range(0, n_Robot2):
            try:
                print('From', i, 'to', j, '=', shortest_path(graph_Robot2, i, j))
            except:break
    #plt.plot(X, Y, 'bs', markersize=10)
    path_Robot1=[0,1]
    X_path1 = list()
    Y_path1 = list()
    for element in path_Robot1:
        X_path1.append(X[element])
        Y_path1.append(Y[element])

    TimerStart = time.time()
    trajectory_1 = SDRE(X_path1,Y_path1)
    print('time1:',time.time() - TimerStart)

    plt.scatter(trajectory_1[0], trajectory_1[1],color='k',s=0.1)


    #print(trajectory_1[2][0])
    #print(trajectory_1[4][0])
    plt.show()
    Time_list=trajectory_1[5][0]
    JJ=trajectory_1[4][0]
    H = list()
    for t in range(0, len(Time_list)):

        H.append( np.trapz(JJ[0:t], x=Time_list[0:t]))
    print(np.shape(H) )
    print(np.shape(Time_list))
    plt.plot(Time_list,H)
    print (max(H))
    plt.show()
    plt.show()
    plt.plot(trajectory_1[2][0])
    plt.show()
    plt.plot(trajectory_1[4][0])'''
    #plt.show()
