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

def Voronoi(n,position_Robot1,position_Robot2,position_Robot3):
    subnodes_Robot1 = list()
    subnodes_Robot2 = list()
    subnodes_Robot3 = list()
    for j in range(0, n):
        if j == position_Robot1 or j == position_Robot2 or j == position_Robot3:
            continue
        else:
            Robot1_distance = (shortest_path(graph, position_Robot1, j))
            #print('robot1 to node', j, '=', (Robot1_distance[0]))

            Robot2_distance = (shortest_path(graph, position_Robot2, j))
            #print('robot2 to node', j, '=', Robot2_distance[0])

            Robot3_distance = (shortest_path(graph, position_Robot3, j))
            #print('robot3 to node', j, '=', Robot3_distance[0])
            if (Robot1_distance[0]) <= (Robot2_distance[0]):
                if (Robot1_distance[0]) <= (Robot3_distance[0]):
                    subnodes_Robot1.append(j)
                elif (Robot3_distance[0]) < (Robot1_distance[0]):
                    subnodes_Robot3.append(j)
            elif (Robot1_distance[0]) <= (Robot3_distance[0]):
                if (Robot1_distance[0]) <= (Robot2_distance[0]):
                    subnodes_Robot1.append(j)
                elif (Robot2_distance[0]) < (Robot1_distance[0]):
                    subnodes_Robot2.append(j)
            elif (Robot2_distance[0]) <= (Robot3_distance[0]):
                if (Robot2_distance[0]) < (Robot1_distance[0]):
                    subnodes_Robot2.append(j)
                elif (Robot1_distance[0]) <= (Robot2_distance[0]):
                    subnodes_Robot1.append(j)
            elif (Robot2_distance[0]) <= (Robot1_distance[0]):
                if (Robot2_distance[0]) <= (Robot3_distance[0]):
                    subnodes_Robot2.append(j)
                elif (Robot3_distance[0]) < (Robot2_distance[0]):
                    subnodes_Robot3.append(j)
            elif (Robot3_distance[0]) < (Robot1_distance[0]):
                if (Robot3_distance[0]) < (Robot2_distance[0]):
                    subnodes_Robot3.append(j)
                elif (Robot2_distance[0]) <= (Robot3_distance[0]):
                    subnodes_Robot2.append(j)
            elif (Robot3_distance[0]) <= (Robot2_distance[0]):
                if (Robot3_distance[0]) < (Robot1_distance[0]):
                    subnodes_Robot3.append(j)
                elif (Robot1_distance[0]) <= (Robot3_distance[0]):
                    subnodes_Robot1.append(j)
    return subnodes_Robot1,subnodes_Robot2,subnodes_Robot3

def cost_function(cost,distance):
    totalCost=cost*distance
    return totalCost


def finding_nextPoint(subnodes_Robot,position_Robot):
    initialCost_Robot=list()
    for vertex in subnodes_Robot:
        distance = (shortest_path(graph, position_Robot, vertex))
        initialCost_Robot.append(cost_function( priorityValue[vertex],distance[0]))
    initialCost_Robot=sum(initialCost_Robot)
    totalCost_voronoi_Robot=dict()
    Robot_neighbors=list()
    for node in subnodes_Robot:
        if points[position_Robot, node]!=0:
            Robot_neighbors.append(node)
    #print(Robot_neighbors)

    subnodes_Robot.append(position_Robot)
    for neighbor in Robot_neighbors:
        costNeighbor_Robot = list()
        for vertex in subnodes_Robot:
            if neighbor==vertex:continue
            else:
                distance = (shortest_path(graph, neighbor, vertex))
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
    graph = Graph()
    points = np.genfromtxt("undirected-extended-connection.txt", delimiter=" ")
     #print(points)
    position_Robot1 = input('Enter Roomba A position:')
    position_Robot2 = input('Enter Roomba B position:')
    position_Robot3 = input('Enter Roomba C position:')
    goal1 = input('What is the first region of interest?')
    goal2 = input('What is the second region of interest?')
    goal3 = input('What is the third region of interest?')
    position_Robot1 = int(position_Robot1)
    position_Robot2 = int(position_Robot2)
    position_Robot3 = int(position_Robot3)
    goal1 = int(goal1)
    goal2= int(goal2)
    goal3= int(goal3)
    X = [150.0, 209.5, 271.0, 332.0, 393.5, 455.5, 515.5, 150.0, 211.7, 272.5, 333.5, 394.5, 455.0, 515.5, 150.0, 211.6,
         271.8, 332.2, 393.7, 454.5, 514.7, 150.7, 211.5, 273.7, 334.5, 394.4, 455.8, 513.2, 150.8, 212.0, 272.0, 333.0,
         394.0, 455.2, 515.7, 151.1, 212.6, 274.4, 334.3, 396.3, 456.0, 517.6, 150.7, 210.3, 271.4, 333.0, 394.2, 455.2,
         516.5]
    Y = [-43.0, -43.0, -41.5, -41.5, -41.5, -40.5, -40.0, 19.5, 20.1, 20.0, 18.5, 18.0, 19.0, 18.5, 79.7, 80.7, 80.0,
         79.5, 79.8, 79.0, 79.7, 140.5, 141.2, 141.8, 141.5, 140.0, 141.4, 135.0, 203.0, 202.6, 202.5, 202.0, 202.5,
         204.0, 201.2, 263.4, 264.8, 264.0, 264.0, 265.5, 264.0, 265.0, 324.8, 329.3, 327.6, 324.6, 324.5, 324.3, 326.9]
    rowNumber = 0
    columnNumber = 0
    priorityValue=dict()
    for row in points:
        graph.add_node(rowNumber)
        rowNumber = rowNumber + 1
    n = rowNumber
    print(n)
    for i in range(0, n):
        for j in range(0, n):
            if points[i, j] != 0:
                graph.add_edge(i, j, points[i, j])
        priorityValue[i]=10
    priorityValue[goal1]=1000
    priorityValue[goal2] = 5000
    priorityValue[goal3] = 10000
    print (priorityValue)
    step=list()
    Cost=list()
    for p in range(1,20):
        if p==1:
            voronoiSubsets = Voronoi(n, position_Robot1, position_Robot2, position_Robot3)
        else:
            voronoiSubsets = Voronoi(n, nextBest_position1[0], nextBest_position2[0], nextBest_position3[0])
        subnodes_Robot1 = voronoiSubsets[0]
        subnodes_Robot2 = voronoiSubsets[1]
        subnodes_Robot3 = voronoiSubsets[2]
        if p==1:
            nextBest_position1 = finding_nextPoint(subnodes_Robot1, position_Robot1)
            nextBest_position2 = finding_nextPoint(subnodes_Robot2, position_Robot2)
            nextBest_position3 = finding_nextPoint(subnodes_Robot3, position_Robot3)
        else:
            nextBest_position1 = finding_nextPoint(subnodes_Robot1, nextBest_position1[0])
            nextBest_position2 = finding_nextPoint(subnodes_Robot2, nextBest_position2[0])
            nextBest_position3 = finding_nextPoint(subnodes_Robot3, nextBest_position3[0])
        print('step:',p)
        print('nextBest_position1:', '', nextBest_position1[0],',',nextBest_position1[1])
        print('nextBest_position2:', '', nextBest_position2[0],',',nextBest_position2[1])
        print('nextBest_position3:', '', nextBest_position3[0],',',nextBest_position3[1])
        totalCost=nextBest_position1[1]+nextBest_position2[1]+nextBest_position3[1]
        Cost.append(totalCost)
        step.append(p)
    plt.plot(step,Cost)
    plt.show()
