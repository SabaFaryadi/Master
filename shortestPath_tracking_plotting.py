from __future__ import division
from marvelmind import MarvelmindHedge
from time import sleep
import sys
import time
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import math
from collections import defaultdict, deque
from breezycreate2 import Robot


class Graph(object):
    def __init__(self):
        self.nodes = set()
        self.edges = defaultdict(list)
        self.distances = {}

    def add_node(self, value):
        self.nodes.add(value)

    def add_edge(self, from_node, to_node, distance):
        self.edges[from_node].append(to_node)
        # self.edges[to_node].append(from_node)
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

    return list(full_path)


def navigation(X_Final, Y_Final, Threshold, StepHeading, SleepPeriod):
    ErrorY = 1
    ErrorX = 1
    start_time = time.time()
    while (ErrorX > Threshold or ErrorY > Threshold):
        positionX = list()
        positionY = list()
        positionX1 = list()
        positionY1 = list()
        positionX2 = list()
        positionY2 = list()
        centroidX_path = list()
        centroidY_path = list()
        start_time = time.time()
        while (time.time() - start_time) < 3 * SleepPeriod:
            sleep(SleepPeriod)
            current_psition = hedge1.position()
            if current_psition[1] == 0 or current_psition[2] == 0:
                print 'error in GPS positioning'
                continue
            positionX.append(current_psition[1])
            positionY.append(current_psition[2])
            positionX1.append(current_psition[1])
            positionY1.append(current_psition[2])
            print'current_psition=', current_psition[1], ',', current_psition[2]
        X1 = (sum(positionX1) / len(positionX1))
        Y1 = (sum(positionY1) / len(positionY1))
        start_time = time.time()
        while (time.time() - start_time) < 3 * SleepPeriod:
            sleep(SleepPeriod)
            current_psition = hedge2.position()
            if current_psition[1] == 0 or current_psition[2] == 0:
                print'error in GPS positioning'
                continue
            positionX.append(current_psition[1])
            positionY.append(current_psition[2])
            positionX2.append(current_psition[1])
            positionY2.append(current_psition[2])
            print'current_psition=', current_psition[1], ',', current_psition[2]
        X2 = (sum(positionX2) / len(positionX2))
        Y2 = (sum(positionY2) / len(positionY2))
        x = np.array(positionX)
        y = np.array(positionY)
        theta = np.zeros((2, 1))
        (m, b) = np.polyfit(x, y, 1)
        x1 = X1
        y1 = m * x1 + b
        x2 = X2
        y2 = m * x2 + b
        centroidX = (x1 + x2) / 2
        centroidY = (y1 + y2) / 2
        centroidX_path.append(centroidX)
        centroidY_path.append(centroidY)
        ErrorX = math.fabs(X_Final - centroidX)
        ErrorY = math.fabs(Y_Final - centroidY)
        # if ErrorX<0.2 and ErrorY<0.2:break
        m_degree = math.atan2(y2 - y1, x2 - x1) * 180 / math.pi
        Teta_heading = m_degree - 90
        if Teta_heading < 0:
            Teta_heading = Teta_heading + 360
        print  'Heading_slope ', Teta_heading
        Teta_aproach = math.atan2(Y_Final - centroidY, X_Final - centroidX) * 180 / math.pi
        if Teta_aproach < 0:
            Teta_aproach = Teta_aproach + 360
        print 'Theta Approach=', Teta_aproach

        Teta_Robot = Teta_aproach - Teta_heading
        while Teta_Robot < 0:
            Teta_Robot = Teta_Robot + 360
        while Teta_Robot > 360:
            Teta_Robot = Teta_Robot + 360

        if (Teta_Robot <= 180):
            bot.setForwardSpeed(0)
            bot.setTurnSpeed(-400)
        else:
            bot.setForwardSpeed(0)
            Teta_Robot = 360 - Teta_Robot
            bot.setTurnSpeed(+400)

        sleep(Teta_Robot / 180)
        bot.setTurnSpeed(0)

        bot.setForwardSpeed(50)
        sleep(StepApproach)
        # bot.setForwardSpeed(0)
        ErrorX = math.fabs(X_Final - centroidX)
        ErrorY = math.fabs(Y_Final - centroidY)
    # if ErrorX<Threshold and ErrorY<Threshold:break
    # print 'error',ErrorX,ErrorY
    bot.setForwardSpeed(0)
    return centroidX_path, centroidY_path


if __name__ == '__main__':
    graph = Graph()
    points = np.genfromtxt("newconnection.txt", delimiter=" ")
    rowNumber = 0
    columnNumber = 0
    SleepPeriod = 0.2

    StepHeading = 1
    StepApproach = 2
    Threshold = 0.3
    X = [0.73472, 1.92912, 3.1645, 4.38612, 0.71062, 1.94628, 3.16704, 4.38568, 0.7195, 1.91526, 3.17254, 4.39526,
         0.6789, 1.89398, 3.1669, 4.38698]
    Y = [-0.02102, -0.0125, 0.01, 0.0421, 1.1086, 1.18886, 1.20054, 1.21036, 2.41466, 2.45076, 2.38374, 2.40446,
         3.62488, 3.66896, 3.572, 3.60554]
    plt.plot(X, Y, 'ro')
    i = 0
    for x in range(0, len(X)):
        plt.annotate(i, xy=(X[x], Y[x]))
        i = i + 1

    bot = Robot()
    hedge1 = MarvelmindHedge(tty="/dev/ttyACM0", adr=2, debug=False)
    hedge2 = MarvelmindHedge(tty="/dev/ttyACM1", adr=8, debug=False)
    hedge1.start()
    hedge2.start()
    sleep(1)

    for row in points:
        graph.add_node(rowNumber)
        rowNumber = rowNumber + 1
    n = rowNumber
    # print(n)
    for i in range(0, n):
        for j in range(0, n):
            if points[i, j] != 0:
                graph.add_edge(i, j, points[i, j])
    # print(graph.distances)
    path = shortest_path(graph, 14, 0)
    print path
    X_path = list()
    Y_path = list()
    for element in path:
        X_path.append(X[element])
        Y_path.append(Y[element])
    plt.plot(X_path, Y_path)
    plt.show()
    real_pathX = list()
    real_pathY = list()
    for n in range(1, len(path)):
        print 'destination', n, ':', 'point', path[n], ':', X[path[n]], '', Y[path[n]]
        X_Final = X[path[n]]
        Y_Final = Y[path[n]]
        centroid_path = navigation(X_Final, Y_Final, Threshold, StepHeading, SleepPeriod)
        centroidX_path = centroid_path[0]
        centroidY_path = centroid_path[1]
        for t in centroidX_path:
            real_pathX.append(t)
        for s in centroidY_path:
            real_pathY.append(s)

    plt.plot(real_pathX, real_pathY)
    plt.show()
    hedge1.stop()
    hedge2.stop()
    bot.close()
