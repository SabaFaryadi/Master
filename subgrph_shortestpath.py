from collections import defaultdict, deque
import numpy as np
import matplotlib.pyplot as plt

class Graph(object):
    def __init__(self):
        self.nodes = set()
        self.edges = defaultdict(list)
        self.distances = {}

    def add_node(self, value):
        self.nodes.add(value)

    def add_edge(self, from_node, to_node, distance):
        self.edges[from_node].append(to_node)
        #self.edges[to_node].append(from_node)
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

if __name__ == '__main__':
    graph = Graph()
    points = np.genfromtxt("newconnection.txt", delimiter=" ")
    position_Robot1=input('Enter Roomba A position:')
    position_Robot2=input('Enter Roomba B position:')
    goal=input('What is the region of interest?')
    position_Robot1=int(position_Robot1)
    position_Robot2=int(position_Robot2)
    goal= int(goal)
    X = [0.73472, 1.92912, 3.1645, 4.38612, 0.71062, 1.94628, 3.16704, 4.38568, 0.7195, 1.91526, 3.17254, 4.39526,
         0.6789, 1.89398, 3.1669, 4.38698]
    Y = [-0.02102, -0.0125, 0.01, 0.0421, 1.1086, 1.18886, 1.20054, 1.21036, 2.41466, 2.45076, 2.38374, 2.40446,
         3.62488, 3.66896, 3.572, 3.60554]
    rowNumber = 0
    columnNumber = 0

    for row in points:
        graph.add_node(rowNumber)
        rowNumber = rowNumber + 1
    n = rowNumber
    for i in range(0, n):
        for j in range(0, n):
            if points[i, j] != 0:
                graph.add_edge(i, j, points[i, j])
    #print(graph.distances)

    i = 0
    for x in range(0, len(X)):
        if x==position_Robot1:
            plt.annotate('RoombaA', xy=(X[position_Robot1], Y[position_Robot1]))
        elif x==position_Robot2:
            plt.annotate('RoombaB', xy=(X[position_Robot2], Y[position_Robot2]))
        elif x==goal:
            plt.annotate('goal', xy=(X[goal], Y[goal]))
        plt.annotate(i, xy=(X[x], Y[x]))
        i = i + 1
    subnodes_Robot1=list()
    subnodes_Robot2 = list()
    same_nodes=list()
    for j in range(0,n):
        if j==position_Robot1 or j==position_Robot2:
            continue
        else:
            Robot1_distance=(shortest_path(graph,position_Robot1, j))
            print('robot1 to node',j,'=',Robot1_distance[0])

            Robot2_distance=(shortest_path(graph,position_Robot2, j))
            print('robot2 to node', j, '=', Robot2_distance[0])

            if Robot1_distance[0]>Robot2_distance[0]:
                subnodes_Robot2.append(j)
            elif Robot1_distance[0]<Robot2_distance[0]:
                subnodes_Robot1.append(j)
            elif Robot1_distance[0]==Robot2_distance[0]:
                #subnodes_Robot2.append(j)
                #subnodes_Robot1.append(j)
                same_nodes.append(j)
    print('Robot1',subnodes_Robot1)
    print('Robot2', subnodes_Robot2)
    if goal in subnodes_Robot1:
        path = shortest_path(graph, position_Robot1, goal)
        color='r'
    elif goal in subnodes_Robot2:
        path = shortest_path(graph, position_Robot2, goal)
        color = 'b'
    print(path[1])
    X_path = list()
    Y_path = list()
    for element in path[1]:
        X_path.append(X[element])
        Y_path.append(Y[element])
    plt.plot(X_path, Y_path,color)
    X_subnodes_Robot1=list()
    Y_subnodes_Robot1 = list()
    for n in subnodes_Robot1:
        X_subnodes_Robot1.append(X[n])
        Y_subnodes_Robot1.append(Y[n])
    X_subnodes_Robot2 = list()
    Y_subnodes_Robot2 = list()
    for n in subnodes_Robot2:
        X_subnodes_Robot2.append(X[n])
        Y_subnodes_Robot2.append(Y[n])
    X_same_nodes= list()
    Y_same_nodes= list()
    for n in same_nodes:
        X_same_nodes.append(X[n])
        Y_same_nodes.append(Y[n])
    plt.plot(X_subnodes_Robot1, Y_subnodes_Robot1, 'ro', markersize=16)
    plt.plot(X_subnodes_Robot2, Y_subnodes_Robot2, 'bo',markersize=16)
    plt.plot(X_same_nodes, Y_same_nodes, 'go', markersize=16)
    plt.plot(X[position_Robot1],Y[position_Robot1],'rs',markersize=20)
    plt.plot(X[position_Robot2], Y[position_Robot2], 'bs',markersize=20)
    plt.show()
