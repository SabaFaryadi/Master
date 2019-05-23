from collections import defaultdict, deque
import numpy as np
import matplotlib.pyplot as plt
import math
import controlpy
import matplotlib.patches as mpatches

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

def Voronoi(n,position_Robot1,position_Robot2):
    subnodes_Robot1 = list()
    subnodes_Robot2 = list()
    for j in range(0, n):
        if j == position_Robot1 or j == position_Robot2:
            continue
        else:
            Robot1_distance = (shortest_path(Graph, position_Robot1, j))
                #print('robot1 to node', j, '=', (Robot1_distance[0]))

            Robot2_distance = (shortest_path(Graph, position_Robot2, j))
            if (Robot1_distance[0]) <=(Robot2_distance[0]):
                subnodes_Robot1.append(j)

            elif (Robot2_distance[0])<(Robot1_distance[0]):
                subnodes_Robot2.append(j)

    return subnodes_Robot1,subnodes_Robot2

def cost_function(cost,distance):
    totalCost=cost*distance
    return totalCost

def finding_nextPoint(subnodes_Robot,position_Robot,n):
    initialCost_Robot=list()
    for vertex in subnodes_Robot:
        distance = (shortest_path(Graph, position_Robot, vertex))
        initialCost_Robot.append(cost_function(priorityValue[vertex],distance[0]))
    initialCost_Robot=sum(initialCost_Robot)
    totalCost_voronoi_Robot=dict()
    Robot_neighbors=list()
    for node in range(0,n):
        if points[position_Robot, node]!=0:
            Robot_neighbors.append(node)
    #print(Robot_neighbors)
    print ('neighbors',Robot_neighbors)
    subnodes_Robot.append(position_Robot)
    for neighbor in Robot_neighbors:
        costNeighbor_Robot = list()
        for vertex in subnodes_Robot:
            if neighbor==vertex:continue
            else:
                distance = (shortest_path(Graph, neighbor, vertex))
                costNeighbor_Robot.append(cost_function(priorityValue[vertex], distance[0]))
        #print(costNeighbor_Robot)
        totalCost_voronoi_Robot[neighbor]=sum(costNeighbor_Robot)

        next_position=min(totalCost_voronoi_Robot.items(), key=lambda x: x[1])[0]
    #print(initialCost_Robot, totalCost_voronoi_Robot[next_position])
    if initialCost_Robot<=totalCost_voronoi_Robot[next_position]:
        nextBest_position=position_Robot
    else:
        nextBest_position=next_position
    return nextBest_position,initialCost_Robot

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
                #print((K))
                u=np.array([x_initial[j,time],y_initial[j,time],tetha[j,time],zd1[j,time],zd2[j,time],yd1[j,time],yd2[j,time],wd1[j,time]])
                #print(type(u))
                U=np.dot(-K,u.transpose())

                v[j,time]=U[0]
                w[j,time]=U[1]

                x_initial[j,time+1]=x_initial[j,time]+delta_t*(U[0]*math.cos(tetha[j,time]))
                y_initial[j,time+1]=y_initial[j,time]+delta_t*(U[0]*math.sin(tetha[j,time]))
                tetha[j,time+1]=tetha[j,time]+delta_t*U[1]
                zd1[j,time+1]=zd1[j,time]+delta_t*zd2[j,time]
                zd2[j,time+1]=zd2[j,time]
                yd1[j,time+1]=yd1[j,time]+delta_t*yd2[j,time]
                yd2[j,time+1]=yd2[j,time]
                wd1[j,time+1]=wd1[j,time]
    return x_initial,y_initial,v,w
if __name__ == '__main__':
    Graph = Graph()
    graph = np.zeros((214, 214))
    for j in range(0, 214):
        if j % 26 == 0:
            for p in range(0, 5):
                graph[j + p][j + p + 1] = 2.1
                try:
                    graph[j + p][j + 2 * p + 6] = 0.65
                except:
                    continue
        if j % 26 == 1:
            for p in range(0, 5):
                graph[j + p][j + p - 1] = 2.1
                try:
                    graph[j + p][j + p + 26] = 2.1
                    graph[j + p][j + p - 26] = 2.1
                except:
                    continue
        if j % 26 == 6:
            for p in range(0, 5):
                try:
                    graph[j + 2 * p][j + 2 * p + 10] = 1.25
                except:
                    continue
        if j % 26 == 7:
            for p in range(0, 5):
                try:
                    graph[j + 2 * p][j + 2 * p - (p + 6)] = 0.65
                except:
                    continue

        if j % 26 == 16:
            for p in range(0, 5):
                graph[j + 2 * p][j + 2 * p + 1] = 0.9

        if j % 26 == 17:
            for p in range(0, 5):
                for p in range(0, 5):
                    try:
                        graph[j + 2 * p][j + 2 * p - 10] = 1.25
                    except:
                        continue
    graph[1][189] = 0
    graph[2][190] = 0
    graph[3][191] = 0
    graph[4][192] = 0
    graph[5][193] = 0
    graph[0][26] = graph[26][0] = 1.75
    graph[52][26] = graph[26][52] = 1.75
    graph[52][78] = graph[78][52] = 1.75
    graph[78][104] = graph[104][78] = 1.75
    graph[130][104] = graph[104][130] = 1.75
    graph[130][156] = graph[156][130] = 1.75
    graph[182][156] = graph[156][182] = 1.75
    graph[208][156] = graph[208][182] = 1.75
    points = graph
    # print(points)
    position_Robot1 = input('Enter Roomba A position:')
    position_Robot2 = input('Enter Roomba B position:')
    goal1 = input('What is the first region of interest?')
    goal2 = input('What is the second region of interest?')
    position_Robot1 = int(position_Robot1)
    position_Robot2 = int(position_Robot2)
    goal1 = int(goal1)
    goal2 = int(goal2)
    X = list()
    Y = list()
    for j in range(1, 26):
        for i in range(1, 17):

            if j % 3 == 1:

                if i % 3 == 1:
                    x = 1 + (i // 3) * 2.1
                    y = 1 + (j // 3) * 1.75
                    X.append(x)
                    Y.append(y)
            elif j % 3 == 2:
                if i % 3 == 2:
                    x = 1.6 + (i // 3) * 2.1
                    y = 1.25 + (j // 3) * 1.75
                    X.append(x)
                    Y.append(y)
                elif i % 3 == 0:
                    x = 2.5 + ((i // 3) - 1) * 2.1
                    y = 1.25 + (j // 3) * 1.75
                    X.append(x)
                    Y.append(y)
            elif j % 3 == 0:
                if i % 3 == 2:
                    x = 1.6 + (i // 3) * 2.1
                    y = 2.5 + ((j // 3) - 1) * 1.75
                    X.append(x)
                    Y.append(y)
                elif i % 3 == 0:
                    x = 2.5 + ((i // 3) - 1) * 2.1
                    y = 2.5 + ((j // 3) - 1) * 1.75
                    X.append(x)
                    Y.append(y)

                    # print(X)
                    # print(Y)
    rowNumber = 0
    columnNumber = 0
    priorityValue = dict()
    for row in points:
        Graph.add_node(rowNumber)
        rowNumber = rowNumber + 1
    n = rowNumber
    print(n)
    for i in range(0, n):
        for j in range(0, n):
            if points[i, j] != 0:
                Graph.add_edge(i, j, points[i, j])
        priorityValue[i]=0.01

    priorityValue[goal1] = 10000000
    priorityValue[goal2] = 100000
    print (priorityValue)
    print(points[40,46])
    step=list()
    Cost=list()
    path_Robot1=list()
    path_Robot2 = list()
    i=0
    for p in range(1,22):

            if p == 1:
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

            else:
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
                    print('stooooooppppppp')
                    #break
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

            if path_Robot1[-1]!=nextBest_position1[0]:
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
    #img = plt.imread('farm2.png')
    #fig, ax = plt.subplots()
    #ax.imshow(img, extent=[120, 570, -50, 350])
    for x in range(0, len(X)):
        if x == position_Robot1:
            plt.annotate('RoombaA', xy=(X[position_Robot1], Y[position_Robot1]),size=12)
            plt.annotate(i, xy=(X[x], Y[x]),color='c',size=1)
        elif x == position_Robot2:
            plt.annotate('RoombaB', xy=(X[position_Robot2], Y[position_Robot2]),size=12)
            plt.annotate(i, xy=(X[x], Y[x]), color='c',size=1)
            plt.annotate(i, xy=(X[x], Y[x]), color='c',size=1)
        elif x == goal1:
            plt.annotate('goal1', xy=(X[goal1], Y[goal1]),size=12)
            plt.annotate(i, xy=(X[x], Y[x]), color='w',size=1)
        elif x == goal2:
            plt.annotate('goal2', xy=(X[goal2], Y[goal2]),size=12)
            plt.annotate(i, xy=(X[x], Y[x]), color='w',size=1)
        else:
            plt.annotate(i, xy=(X[x], Y[x]))
        i = i + 1

    plt.plot(X_subnodes_Robot1, Y_subnodes_Robot1, 'rs', markersize=12,label='Robot1_partition')
    plt.plot(X_subnodes_Robot2, Y_subnodes_Robot2, 'bs', markersize=12,label='Robot2_partition')

    plt.plot(X[position_Robot1], Y[position_Robot1], 'cs', markersize=20)
    plt.plot(X[position_Robot2], Y[position_Robot2], 'cs', markersize=20)



    X_path1 = list()
    Y_path1 = list()
    X_path1.append(X[position_Robot1])
    Y_path1.append(Y[position_Robot1])
    for element in path_Robot1:
        X_path1.append(X[element])
        Y_path1.append(Y[element])
    plt.plot(X_path1, Y_path1, 'k',label='Robot1_path')
    X_path2 = list()
    Y_path2 = list()
    X_path2.append(X[position_Robot2])
    Y_path2.append(Y[position_Robot2])
    for element in path_Robot2:
        X_path2.append(X[element])
        Y_path2.append(Y[element])
    plt.plot(X_path2, Y_path2, 'k',label='Robot2_path')
    plt.legend(bbox_to_anchor=(1.01, 1),loc=2, borderaxespad=0.)
    #plt.show()
    #plt.plot(step, Cost)
    #plt.xlabel('Iterations')
    #plt.ylabel('Total Cost')

    plt.xlim((0, 12))
    plt.ylim((0,16))
    plt.show()
