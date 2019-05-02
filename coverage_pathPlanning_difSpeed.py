#This code is developed to employ two robots with different speeds
from collections import defaultdict, deque
import numpy as np
import matplotlib.pyplot as plt
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
    robotSpeed=[20,20]
    #robotCharge=[100,100]
    for j in range(0, n):
        if j == position_Robot1 or j == position_Robot2:
            continue
        else:
            Robot1_distance = (shortest_path(Graph, position_Robot1, j))
                #print('robot1 to node', j, '=', (Robot1_distance[0]))

            Robot2_distance = (shortest_path(Graph, position_Robot2, j))
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

    #priorityValue[goal2 + 6] = 20000
    #priorityValue[goal2 - 1] = 10000
    priorityValue[goal1] = 10000000
    priorityValue[goal2] = 100000
    #priorityValue[goal1 + 6] = 200000
    #priorityValue[goal1 - 1] = 100000

    print (priorityValue)
    print(points[40,46])
    step=list()
    Cost=list()
    path_Robot1=list()
    path_Robot2 = list()
    i=0
    iterations=22
    for p in range(1,iterations):

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

            else:break

    #plt.xlim(())
    #plt.ylim(())
    #plt.show()
