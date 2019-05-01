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

            else:break
            '''voronoiSubsets = Voronoi(n, nextBest_position1[0], nextBest_position2[0])
            subnodes_Robot1 = voronoiSubsets[0]
            subnodes_Robot2 = voronoiSubsets[1]
            if p == 1:
                nextBest_position1 = finding_nextPoint(subnodes_Robot1, position_Robot1, n)
                nextBest_position2 = finding_nextPoint(subnodes_Robot2, position_Robot2, n)

                #if nextBest_position1[0] != position_Robot1:
                path_Robot1.append(nextBest_position1[0])

                # if nextBest_position2[0] not in path_Robot2:

                #if nextBest_position2[0] != position_Robot2:
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
            # if nextBest_position1[0] not in path_Robot1:


            if path_Robot1[-1]!=nextBest_position1[0]:
                path_Robot1.append(nextBest_position1[0])


            # if nextBest_position2[0] not in path_Robot2:

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
    #pathhhh=list()
    #for pth in range(0,len(path_Robot1)-1):
       # if path_Robot1[pth]!= path_Robot1[pth+1]:
            #pathhhh.append()
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
    plt.plot(X_path1, Y_path1, 'r--',label='Robot1_path')
    X_path2 = list()
    Y_path2 = list()
    X_path2.append(X[position_Robot2])
    Y_path2.append(Y[position_Robot2])
    for element in path_Robot2:
        X_path2.append(X[element])
        Y_path2.append(Y[element])
    plt.plot(X_path2, Y_path2, 'r--',label='Robot2_path')
    plt.legend(bbox_to_anchor=(1.01, 1),loc=2, borderaxespad=0.)
    #plt.show()
    #plt.plot(step, Cost)
    #plt.xlabel('Iterations')
    #plt.ylabel('Total Cost')'''
    plt.plot([11.58,10.8,10,9.48, 9.43,9.40,9.415,9.4,9.4135,9.4095,9,8.64,8.012,7.98, 7.404,7.4,7.42,7.43,7.41,7.4,7.35, 7.308666666666666, 7.77833333333334, 7.815666666666667,8,8.5, 8.759,
              8.856833333333334],
             [1.12,0.98,1.08,0.98250000000000526,1.5,2.12,2.89,3.64,5, 5.9953333333333294,6.12,6.09,6.05,6.065, 6.0526666666666689,7,8,9,10,11,12, 13.116999999999994,
                                  13.428499999999964, 14.693499999999996,14.77,14.75, 14.788499999999997, 13.734333333333332],'b')
    plt.plot([1.05,1.02,0.9785,0.95458,0.9725,0.9416666666666666,2,3.0206666666666666,2.9875,3.0524, 3.0806666666666665,3.06,3.0798, 3.649, 3.6803333333333335, 4.5333333333333,
              4.589833333333334,4.55, 5.177166666666667,5.185, 5.209666666666667],
             [1.03654,2.7,3.5,4.61,6.2,7.954166666666666,8.05,8.06,8.01, 8.0491666666666717,9,9.265,
            9.5681666666666665, 9.9673333333333307, 11.170833333333333, 11.278833333333332,11, 10.090333333333333,
            9.8016666666666694,8, 6.4890000000000017],'b')
    print(X[132],Y[132])
    print(X[106], Y[106])
    print(X[80], Y[80])
    print(X[148], Y[148])
    print(X[149], Y[149])
    print(X[139], Y[139])
    plt.xlim((0, 12))
    plt.ylim((0,16))
    plt.show()
