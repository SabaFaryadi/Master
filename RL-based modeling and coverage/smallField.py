from __future__ import print_function
import numpy as np
import matplotlib.pyplot as plt
import itertools
import heapq
import numpy.matlib
from collections import defaultdict, deque
from matplotlib import colors


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
    _destination = paths[destination]#1111
    while _destination != origin:
        full_path.appendleft(_destination)
        _destination = paths[_destination]#2222
    full_path.appendleft(origin)
    full_path.append(destination)
    
    
    return visited[destination], list(full_path)

def Voronoi(n,Graph,position_Robot1,position_Robot2):
    subnodes_Robot1 = list()
    subnodes_Robot2 = list()
    robotSpeed=[20,15]
    #robotCharge=[100,100]
    for j in range(0, n):
        if j == position_Robot1 or j == position_Robot2:
            continue
        else:
            try:
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
            except:continue

    return subnodes_Robot1,subnodes_Robot2
#Finding cost function based on priority value for each vertex and metric distance
def cost_function(cost,distance):
    totalCost=cost*distance
    return totalCost

def finding_nextPoint(adjacentGraph,Graph,priorityValue,subnodes_Robot,position_Robot,n):
    initialCost_Robot=list()
    for vertex in subnodes_Robot:
        distance = (shortest_path(Graph, position_Robot, vertex))
        initialCost_Robot.append(cost_function(priorityValue[vertex],distance[0]))
    initialCost_Robot=sum(initialCost_Robot)

    totalCost_voronoi_Robot=dict()
    Robot_neighbors=list()
    for node in range(0,n):
        if adjacentGraph[position_Robot, node]!=0:
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



class Maze:

    def __init__(self):
        # maze width
        self.WORLD_WIDTH =21

        # maze height
        self.WORLD_HEIGHT =18

        # all possible actions
        self.ACTION_UP = 0
        self.ACTION_DOWN = 1
        self.ACTION_LEFT = 2
        self.ACTION_RIGHT = 3
        self.actions = [self.ACTION_UP, self.ACTION_DOWN, self.ACTION_LEFT, self.ACTION_RIGHT]

        # start state
        self.START_STATE = [1, 0]

        # goal state
        self.GOAL_STATES = [[0, 9]]
        self.terminal_STATES = [[17,11]]
        #self.terminal_STATES = []

        # all obstacles
        self.obstacles = [[1, 2], [2, 2], [3, 2], [0, 7], [1, 7], [2, 7], [4, 5]]
        self.oldObstacles = None
        self.newObstacles = None
        self.newObstacles2 = None
        self.findObstacles = []
        self.subStates=[]
        # time to change obstacles
        self.changingPoint = None
        self.changingPoint2 = None

        # initial state action pair values
        self.stateActionValues = np.zeros((self.WORLD_HEIGHT, self.WORLD_WIDTH, len(self.actions)))

        # max steps
        self.maxSteps = float('inf')

        # track the resolution for this maze
        self.resolution = 1

    # extend a state to a higher resolution maze
    # @state: state in lower resoultion maze
    # @factor: extension factor, one state will become factor^2 states after extension
    def extendState(self, state, factor):
        newState = [state[0] * factor, state[1] * factor]
        newStates = []
        for i in range(0, factor):
            for j in range(0, factor):
                newStates.append([newState[0] + i, newState[1] + j])
        return newStates

    # extend a state into higher resolution
    # one state in original maze will become @factor^2 states in @return new maze
    def extendMaze(self, factor):
        newMaze = Maze()
        newMaze.WORLD_WIDTH = self.WORLD_WIDTH * factor
        newMaze.WORLD_HEIGHT = self.WORLD_HEIGHT * factor
        newMaze.START_STATE = [self.START_STATE[0] * factor, self.START_STATE[1] * factor]
        newMaze.GOAL_STATES = self.extendState(self.GOAL_STATES[0], factor)
        newMaze.obstacles = []
        for state in self.obstacles:
            newMaze.obstacles.extend(self.extendState(state, factor))
        newMaze.stateActionValues = np.zeros((newMaze.WORLD_HEIGHT, newMaze.WORLD_WIDTH, len(newMaze.actions)))
        newMaze.resolution = factor
        return newMaze

    # take @action in @state
    # @return: [new state, reward]
    def takeAction(self, state, action):
        x, y = state
        if action == self.ACTION_UP:
            x = max(x - 1, 0)
        elif action == self.ACTION_DOWN:
            x = min(x + 1, self.WORLD_HEIGHT - 1)
        elif action == self.ACTION_LEFT:
            y = max(y - 1, 0)
        elif action == self.ACTION_RIGHT:
            y = min(y + 1, self.WORLD_WIDTH - 1)
        if [x, y] in self.obstacles:
            #print('obstacle:',[x,y])
            self.findObstacles.append([x,y])
            x, y = state
            reward = -1.0
        if [x, y] in self.terminal_STATES:
            reward = 1.0
        else:
            reward = 0.0
        return [x, y], reward

# a wrapper class for parameters of dyna algorithms
class DynaParams:

    def __init__(self):
        # discount
        self.gamma = 0.95

        # probability for exploration
        self.epsilon = 0.1

        # step size
        self.alpha = 0.1

        # weight for elapsed time
        self.timeWeight = 0

        # n-step planning
        self.planningSteps = 5

        # average over several independent runs
        self.runs = 10

        # algorithm names
        self.methods = ['Dyna-Q', 'Dyna-Q+']

        # threshold for priority queue
        self.theta = 0


# choose an action based on epsilon-greedy algorithm
def chooseAction(state, stateActionValues, maze, dynaParams):
    if np.random.binomial(1, dynaParams.epsilon) == 1:
        return np.random.choice(maze.actions)
    else:
        values = stateActionValues[state[0], state[1], :]
        return np.random.choice([action for action, value in enumerate(values) if value == np.max(values)])

# Trivial model for planning in Dyna-Q
class TrivialModel:

    # @rand: an instance of np.random.RandomState for sampling
    def __init__(self, rand=np.random):
        self.model = dict()
        self.rand = rand

    # feed the model with previous experience
    def feed(self, currentState, action, newState, reward):
        if tuple(currentState) not in self.model.keys():
            self.model[tuple(currentState)] = dict()
        self.model[tuple(currentState)][action] = [list(newState), reward]

    # randomly sample from previous experience
    def sample(self):
        stateIndex = self.rand.choice(range(0, len(self.model.keys())))
        state = list(self.model)[stateIndex]
        actionIndex = self.rand.choice(range(0, len(self.model[state].keys())))
        action = list(self.model[state])[actionIndex]
        newState, reward = self.model[state][action]
        return list(state), action, list(newState), reward

# Time-based model for planning in Dyna-Q+
class TimeModel:

    # @maze: the maze instance. Indeed it's not very reasonable to give access to maze to the model.
    # @timeWeight: also called kappa, the weight for elapsed time in sampling reward, it need to be small
    # @rand: an instance of np.random.RandomState for sampling
    def __init__(self, maze, timeWeight=1e-4, rand=np.random):
        self.rand = rand
        self.model = dict()

        # track the total time
        self.time = 0

        self.timeWeight = timeWeight
        self.maze = maze

    # feed the model with previous experience
    def feed(self, currentState, action, newState, reward):
        self.time += 1
        if tuple(currentState) not in self.model.keys():
            self.model[tuple(currentState)] = dict()

            # Actions that had never been tried before from a state were allowed to be considered in the planning step
            for action_ in self.maze.actions:
                if action_ != action:
                    # Such actions would lead back to the same state with a reward of zero
                    # Notice that the minimum time stamp is 1 instead of 0
                    self.model[tuple(currentState)][action_] = [list(currentState), 0, 1]

        self.model[tuple(currentState)][action] = [list(newState), reward, self.time]

    # randomly sample from previous experience
    def sample(self):
        stateIndex = self.rand.choice(range(0, len(self.model.keys())))
        state = list(self.model)[stateIndex]
        actionIndex = self.rand.choice(range(0, len(self.model[state].keys())))
        action = list(self.model[state])[actionIndex]
        newState, reward, time = self.model[state][action]

        # adjust reward with elapsed time since last vist
        reward += self.timeWeight * np.sqrt(self.time - time)

        return list(state), action, list(newState), reward


# play for an episode for Dyna-Q algorithm
# @stateActionValues: state action pair values, will be updated
# @model: model instance for planning
# @planningSteps: steps for planning
# @maze: a maze instance containing all information about the environment
# @dynaParams: several params for the algorithm
def dynaQ(stateActionValues, model, maze, dynaParams):
    currentState = maze.START_STATE
    steps = 0
    while currentState not in maze.GOAL_STATES:
        # track the steps
        steps += 1

        # get action
        action = chooseAction(currentState, stateActionValues, maze, dynaParams)

        # take action
        newState, reward = maze.takeAction(currentState, action)

        # Q-Learning update
        stateActionValues[currentState[0], currentState[1], action] += \
            dynaParams.alpha * (reward + dynaParams.gamma * np.max(stateActionValues[newState[0], newState[1], :]) -
            stateActionValues[currentState[0], currentState[1], action])

        # feed the model with experience
        model.feed(currentState, action, newState, reward)

        # sample experience from the model
        for t in range(0, dynaParams.planningSteps):
            stateSample, actionSample, newStateSample, rewardSample = model.sample()
            stateActionValues[stateSample[0], stateSample[1], actionSample] += \
                dynaParams.alpha * (rewardSample + dynaParams.gamma * np.max(stateActionValues[newStateSample[0], newStateSample[1], :]) -
                stateActionValues[stateSample[0], stateSample[1], actionSample])

        currentState = newState

        # check whether it has exceeded the step limit
        if steps > maze.maxSteps:
            break

    return steps


def multi_dynaQ(stateActionValues_1,stateActionValues_2, model, maze_1,maze_2, dynaParams,episode):
    currentState_1 = maze_1.START_STATE
    currentState_2 = maze_2.START_STATE
    steps= 0

    while currentState_1 not in maze_1.terminal_STATES and currentState_2 not in maze_2.terminal_STATES:
#Step2: current pose -----> update Vronoi subet ------> next best state-------------------------------------------------------------------------------------------------------------------------------------
        
        steps += 1
        #print(steps)
        if currentState_1!=maze_1.terminal_STATES:
            # get action agent_1
            action_1 = chooseAction(currentState_1, stateActionValues_1, maze_1, dynaParams)
            # take action
            newState_1, reward_1 ,= maze_1.takeAction(currentState_1, action_1)
            while newState_1==currentState_2 or newState_1 not in maze_1.subStates:
                action_1 = chooseAction(currentState_1, stateActionValues_1, maze_1, dynaParams)
                # re-take action
                newState_1, reward_1 ,= maze_1.takeAction(currentState_1, action_1)

            # Q-Learning update
            stateActionValues_1[currentState_1[0], currentState_1[1], action_1] += \
            dynaParams.alpha * (reward_1 + dynaParams.gamma * np.max(stateActionValues_1[newState_1[0], newState_1[1], :]) - stateActionValues_1[currentState_1[0], currentState_1[1], action_1])

            # feed the model with experience
            model.feed(currentState_1, action_1, newState_1, reward_1)
            # sample experience from the model
            for t in range(0, dynaParams.planningSteps):
                stateSample_1, actionSample_1, newStateSample_1, rewardSample_1 = model.sample()
                stateActionValues_1[stateSample_1[0], stateSample_1[1], actionSample_1] += \
                dynaParams.alpha * (rewardSample_1 + dynaParams.gamma * np.max(stateActionValues_1[newStateSample_1[0], newStateSample_1[1], :]) -
                stateActionValues_1[stateSample_1[0], stateSample_1[1], actionSample_1])

            currentState_1 = newState_1


        if currentState_2!=maze_2.terminal_STATES:
            # get action agent_1
            action_2 = chooseAction(currentState_2, stateActionValues_2, maze_2, dynaParams)
            # take action
            newState_2, reward_2= maze_2.takeAction(currentState_2, action_2)
            while newState_2==currentState_1 or newState_2 not in maze_2.subStates:
                action_2 = chooseAction(currentState_2, stateActionValues_2, maze_2, dynaParams)
                # re-take action
                newState_2, reward_2 = maze_2.takeAction(currentState_2, action_2)

            # Q-Learning update
            stateActionValues_2[currentState_2[0], currentState_2[1], action_2] += \
            dynaParams.alpha * (reward_2 + dynaParams.gamma * np.max(stateActionValues_2[newState_2[0], newState_2[1], :]) - stateActionValues_2[currentState_2[0], currentState_2[1], action_2])

            # feed the model with experience
            model.feed(currentState_2, action_2, newState_2, reward_2)
            # sample experience from the model
            for t in range(0, dynaParams.planningSteps):
                stateSample_2, actionSample_2, newStateSample_2, rewardSample_2 = model.sample()
                stateActionValues_2[stateSample_2[0], stateSample_2[1], actionSample_2] += \
                dynaParams.alpha * (rewardSample_2 + dynaParams.gamma * np.max(stateActionValues_2[newStateSample_2[0], newStateSample_2[1], :]) -
                stateActionValues_2[stateSample_2[0], stateSample_2[1], actionSample_2])

            currentState_2 = newState_2
            
        if steps==100:
            print('steps')
            PathRobot1=[]
            PathRobot2=[]
            allObstacles=[]
            for element in maze_1.findObstacles:
                allObstacles.append(element)
            for element in maze_2.findObstacles:
                allObstacles.append(element)
            plotMaze(maze_1,maze_2,allObstacles,PathRobot1,PathRobot2,'episode='+ str(episode) +','+'step_100','episode='+ str(episode) +','+'step=100')
            
        if steps==1000:
            print('steps')
            PathRobot1=[]
            PathRobot2=[]
            allObstacles=[]
            for element in maze_1.findObstacles:
                allObstacles.append(element)
            for element in maze_2.findObstacles:
                allObstacles.append(element)
            plotMaze(maze_1,maze_2,allObstacles,PathRobot1,PathRobot2,'episode='+ str(episode) +','+'step_1000','episode='+ str(episode) +','+'step=1000')
        if steps in range(maze_1.changingPoint,maze_1.changingPoint2):
        # change the obstacles
            maze_1.obstacles = maze_1.newObstacles
            maze_2.obstacles = maze_2.newObstacles 
            maze_1.GOAL_STATES = [[11,2],[17,14],[11,15]]
            maze_2.GOAL_STATES = [[11,2],[17,14],[11,15]]
            if steps==10010:
            #print('steps')
                PathRobot1=[]
                PathRobot2=[]
                allObstacles=[]
                for element in maze_1.findObstacles:
                    allObstacles.append(element)
                for element in maze_2.findObstacles:
                    allObstacles.append(element)
                plotMaze(maze_1,maze_2,allObstacles,PathRobot1,PathRobot2,'episode='+ str(episode) +','+'step_10010','episode='+ str(episode) +','+'step=10000')
            
        if steps > maze_1.changingPoint2:
        # change the obstacles
            maze_1.obstacles = maze_1.newObstacles2
            maze_2.obstacles = maze_2.newObstacles2 
            #maze_1.GOAL_STATES = [[12,14],[9,6]]
            maze_1.GOAL_STATES = [[11,2],[17,14],[11,15]]
            maze_2.GOAL_STATES = [[11,2],[17,14],[11,15]]
            
            if steps==30000:
                print('steps')
                PathRobot1=[]
                PathRobot2=[]
                allObstacles=[]
                for element in maze_1.findObstacles:
                    allObstacles.append(element)
                for element in maze_2.findObstacles:
                    allObstacles.append(element)
                plotMaze(maze_1,maze_2,allObstacles,PathRobot1,PathRobot2,'episode='+ str(episode) +','+'step_30000','episode='+ str(episode) +','+'step=30000')
        
            #print('newObstacles:', maze_1.obstacles) 
          
        if steps > maze_1.maxSteps:
            PathRobot1=[]
            PathRobot2=[]
            allObstacles=[]
            for element in maze_1.findObstacles:
                allObstacles.append(element)
            for element in maze_2.findObstacles:
                allObstacles.append(element)
            plotMaze(maze_1,maze_2,allObstacles,PathRobot1,PathRobot2,'episode='+ str(episode) +','+'step='+str(steps),'episode='+ str(episode) +','+'step='+str(steps))
            break
    #print('step',steps)
    
    print('steps=',steps)
    PathRobot1=[]
    PathRobot2=[]
    allObstacles=[]
    for element in maze_1.findObstacles:
        allObstacles.append(element)
    for element in maze_2.findObstacles:
        allObstacles.append(element)
    plotMaze(maze_1,maze_2,allObstacles,PathRobot1,PathRobot2,'episode='+ str(episode) +','+'step='+str(steps),'episode='+ str(episode) +','+'step='+str(steps))
    episode=episode+1
    return steps,currentState_1 ,currentState_2,episode 
    

def multi_changingMaze(maze_1,maze_2, dynaParams):

    # set up max steps
    maxSteps = maze_1.maxSteps

    # track the cumulative rewards
    rewards = np.zeros((2, maxSteps))

    for run in range(0, dynaParams.runs):
        # set up models
        models = [TrivialModel(), TimeModel(maze_1, timeWeight=dynaParams.timeWeight)]

        # track cumulative reward in current run
        rewards_ = np.zeros((2, maxSteps))

        # initialize state action values
        stateActionValues_1 = [np.copy(maze_1.stateActionValues), np.copy(maze_1.stateActionValues)]
        stateActionValues_2 = [np.copy(maze_2.stateActionValues), np.copy(maze_2.stateActionValues)]
     
        for i in range(1, len(dynaParams.methods)):
            print('run episode:', run, dynaParams.methods[i])

            # set old obstacles for the maze
            maze_1.obstacles = maze_1.oldObstacles
            maze_2.obstacles = maze_2.oldObstacles   
            episode=1
            steps = 0
            lastSteps = steps
            while steps < maxSteps:
                #print('step:',steps)
                # play for an episode
                #multi_dynaQ(stateActionValues_1,stateActionValues_2, model, maze_1,maze_2, dynaParams)
                print('episode:',episode)
                episod=multi_dynaQ(stateActionValues_1[i],stateActionValues_2[i], models[i], maze_1,maze_2, dynaParams,episode)
                steps += episod[0]
                episode=episod[3]
                
                Qtable1=stateActionValues_1[1]
                Qtable2=stateActionValues_2[1]
                currentState_1=episod[1]
                currentState_2=episod[2]
                
                #if steps in range(1000,1200):
                 #   print(steps)
                  #  printActions(Qtable1,maze_1,maze_2)                     
                if steps in range(maze_1.changingPoint,maze_1.changingPoint2):
                # change the obstacles
                    maze_1.obstacles = maze_1.newObstacles
                    maze_2.obstacles = maze_2.newObstacles 
                    maze_1.GOAL_STATES = [[11,2],[17,14],[11,15]]
                    maze_2.GOAL_STATES = [[11,2],[17,14],[11,15]]
            
                if steps > maze_1.changingPoint2:
                # change the obstacles
                    maze_1.obstacles = maze_1.newObstacles2
                    maze_2.obstacles = maze_2.newObstacles2 
                    maze_1.GOAL_STATES = [[11,2],[17,14],[11,15]]
                    maze_2.GOAL_STATES = [[11,2],[17,14],[11,15]] 

    return rewards,Qtable1,Qtable2,currentState_1,currentState_2

def changingMaze(maze, dynaParams):

    # set up max steps
    maxSteps = maze.maxSteps

    # track the cumulative rewards
    rewards = np.zeros((2, maxSteps))

    for run in range(0, dynaParams.runs):
        # set up models
        models = [TrivialModel(), TimeModel(maze, timeWeight=dynaParams.timeWeight)]

        # track cumulative reward in current run
        rewards_ = np.zeros((2, maxSteps))

        # initialize state action values
        stateActionValues = [np.copy(maze.stateActionValues), np.copy(maze.stateActionValues)]

        for i in range(1, len(dynaParams.methods)):
            print('run:', run, dynaParams.methods[i])

            # set old obstacles for the maze
            maze.obstacles = maze.oldObstacles

            steps = 0
            lastSteps = steps
            while steps < maxSteps:
                # play for an episode
                steps += dynaQ(stateActionValues[i], models[i], maze, dynaParams)
                saba=stateActionValues[1]
                # update cumulative rewards
                steps_ = min(steps, maxSteps - 1)
                rewards_[i, lastSteps: steps_] = rewards_[i, lastSteps]
                rewards_[i, steps_] = rewards_[i, lastSteps] + 1
                lastSteps = steps

                if steps > maze.changingPoint:
                    # change the obstacles
                    maze.obstacles = maze.newObstacles
            print('end of episode')
        rewards += rewards_

    # averaging over runs
    rewards /= dynaParams.runs

    return rewards,saba


# DynaQ_Plus, ShortcutMaze
def DynaQ_Plus():
    # set up a shortcut maze instance
     # set up a shortcut maze instance
    shortcutMaze = Maze()
    shortcutMaze.START_STATE = [0,0]
    shortcutMaze.GOAL_STATES = []

    #agent2
    shortcutMaze_2 = Maze()
    shortcutMaze_2.START_STATE = [0,20]
    shortcutMaze_2.GOAL_STATES = []
    #[[[15, 6],[17,8],[5,14]]]

    shortcutMaze.oldObstacles = shortcutMaze_2.oldObstacles=[[1,1],[1,2],[1,3],[1,5],[1,6],[1,7],[1,9],[1,10],[1,11],[1,13],[1,14],[1,15],[1,17],[1,18],[1,19],
[4,1],[4,2],[4,3],[4,5],[4,6],[4,7],[4,9],[4,10],[4,11],[4,13],[4,14],[4,15],[4,17],[4,18],[4,19],
[7,1],[7,2],[7,3],[7,5],[7,6],[7,7],[7,9],[7,10],[7,11],[7,13],[7,14],[7,15],[7,17],[7,18],[7,19],
[10,1],[10,2],[10,3],[10,5],[10,6],[10,7],[10,9],[10,10],[10,11],[10,13],[10,14],[10,15],[10,17],[10,18],[10,19],
[13,1],[13,2],[13,3],[13,5],[13,6],[13,7],[13,9],[13,10],[13,11],[13,13],[13,14],[13,15],[13,17],[13,18],[13,19],
[16,1],[16,2],[16,3],[16,5],[16,6],[16,7],[16,9],[16,10],[16,11],[16,13],[16,14],[16,15],[16,17],[16,18],[16,19],[3,16],[11,16]]
   
    shortcutMaze.newObstacles=shortcutMaze_2.newObstacles=[[1,1],[1,2],[1,3],[1,5],[1,6],[1,7],[1,9],[1,10],[1,11],[1,13],[1,14],[1,15],[1,17],[1,18],[1,19],
[4,1],[4,2],[4,3],[4,5],[4,6],[4,7],[4,9],[4,11],[4,13],[4,14],[4,15],[4,17],[4,18],[4,19],
[7,1],[7,2],[7,3],[7,5],[7,6],[7,7],[7,9],[7,10],[7,11],[7,13],[7,14],[7,15],[7,17],[7,18],[7,19],
[10,1],[10,2],[10,3],[10,5],[10,6],[10,7],[10,9],[10,10],[10,11],[10,13],[10,14],[10,15],[10,17],[10,18],[10,19],
[13,1],[13,2],[13,3],[13,5],[13,6],[13,7],[13,9],[13,10],[13,11],[13,13],[13,14],[13,15],[13,17],[13,18],[13,19],
[16,1],[16,2],[16,3],[16,5],[16,6],[16,7],[16,9],[16,10],[16,11],[16,13],[16,14],[16,15],[16,17],[16,18],[16,19],[11,6],[5,4],[14,12],[8,0],[10,9],[3,16],[6,12],[17,15],[11,16]]

    shortcutMaze.newObstacles2=shortcutMaze_2.newObstacles2=[[1,1],[1,2],[1,3],[1,5],[1,7],[1,9],[1,10],[1,11],[1,13],[1,14],[1,15],[1,17],[1,18],[1,19],
[4,1],[4,2],[4,3],[4,5],[4,6],[4,7],[4,9],[4,11],[4,13],[4,14],[4,15],[4,17],[4,18],[4,19],
[7,1],[7,2],[7,3],[7,5],[7,6],[7,7],[7,9],[7,10],[7,11],[7,13],[7,14],[7,15],[7,17],[7,18],[7,19],
[10,1],[10,2],[10,3],[10,5],[10,6],[10,7],[10,9],[10,10],[10,11],[10,13],[10,14],[10,15],[10,17],[10,18],[10,19],
[13,1],[13,2],[13,3],[13,5],[13,6],[13,7],[13,9],[13,10],[13,11],[13,13],[13,14],[13,15],[13,17],[13,18],[13,19],
[16,1],[16,2],[16,3],[16,5],[16,6],[16,7],[16,9],[16,10],[16,11],[16,13],[16,14],[16,15],[16,17],[16,18],[16,19],[11,6],[5,4],[14,12],[8,0],[10,9],[3,16],[6,12],[17,15],[11,16],[15,20],[14,10]]
    
    shortcutMaze.maxSteps = 30000
    shortcutMaze_2.maxSteps = 30000
    # obstacles will change after 3000 steps
    # the exact step for changing will be different
    # However given that 3000 steps is long enough for both algorithms to converge,
    # the difference is guaranteed to be very small
    shortcutMaze.changingPoint = 10000
    shortcutMaze_2.changingPoint = 10000
    
    shortcutMaze.changingPoint2 = 15000
    shortcutMaze_2.changingPoint2 = 15000
    
    # set up parameters
    dynaParams = DynaParams()

    # 50-step planning
    dynaParams.planningSteps =1

    # average over 5 independent runs
    dynaParams.runs = 1

    # weight for elapsed time sine last visit
    dynaParams.timeWeight = 1e-2

    # also a tricky alpha ...
    dynaParams.alpha = 0.7

    
    allObstacles=[]
    plainMatrix=createMatrix()
    shortcutMaze.subStates=plainMatrix[0]
    shortcutMaze_2.subStates=plainMatrix[1]

    plotMaze_voronoi(shortcutMaze,shortcutMaze_2,allObstacles,[],[],plainMatrix[0],plainMatrix[1],'episode=0','episode=0')
    Qtable=multi_changingMaze(shortcutMaze,shortcutMaze_2, dynaParams)
    #printActions(Qtable[2],shortcutMaze_2)
    currentState_1=Qtable[3]
    currentState_2=Qtable[4]
    print('currentState_1=',currentState_1)
    print('currentState_2=',currentState_2)
    
    
    for element in shortcutMaze.findObstacles:
        allObstacles.append(element)
    for element in shortcutMaze_2.findObstacles:
        allObstacles.append(element)

    matrix=printModel(shortcutMaze,shortcutMaze_2,allObstacles)
    return matrix,plainMatrix[2],currentState_1,currentState_2,shortcutMaze,shortcutMaze_2,allObstacles

def printActions(stateActionValues, maze_1):
    #print(maze_1.findObstacles)
    bestActions = []
    for i in range(0, maze_1.WORLD_HEIGHT):
        bestActions.append([])
        for j in range(0, maze_1.WORLD_WIDTH):
            if [i, j] in maze_1.GOAL_STATES:
                bestActions[-1].append('G')
                continue
            if [i, j] in maze_1.findObstacles:
                bestActions[-1].append('X')
                continue
            bestAction = np.argmax(stateActionValues[i, j, :])
            if bestAction == maze_1.ACTION_UP:
                bestActions[-1].append('U')
            if bestAction == maze_1.ACTION_DOWN:
                bestActions[-1].append('D')
            if bestAction == maze_1.ACTION_LEFT:
                bestActions[-1].append('L')
            if bestAction == maze_1.ACTION_RIGHT:
                bestActions[-1].append('R')
    for row in bestActions:
        print(row)
    print('')
def printModel(maze_1,maze_2,obstacles):
    bestActions = []
    for i in range(0, maze_1.WORLD_HEIGHT):
        bestActions.append([])
        for j in range(0, maze_1.WORLD_WIDTH):
            if [i, j] in obstacles:
                bestActions[-1].append('X')
                continue
            if [i, j] in maze_1.GOAL_STATES:
                bestActions[-1].append('G')
                continue
            if [i, j] in maze_2.GOAL_STATES:
                bestActions[-1].append('G')
                continue
           
            else:
                bestActions[-1].append(' ')
    for row in bestActions:
        print(row)
    print('')
    return bestActions
def createMatrix():
#step1:list of total states.................................................................................................    
    cells=[]
    for i in range(18):
        for j in range(21):
           cells.append([i,j])
    print('cells size:', len(cells))
# Step1:raw matrix for unknown environment...............................................................................
    plainMatrix=[]  
    for i in range(0,18):
        plainMatrix.append([])
        for j in range(0,21):
                plainMatrix[-1].append(' ')

#Generate Adjacent Graph for plain field  
    plain_adjacentGraph=np.zeros([378,378])
 
    for node in cells:
        x,y=node
        if x+1 in range(0,18) and plainMatrix[x+1][y]!='X' and  plainMatrix[x][y]!='X' :
            plain_adjacentGraph[cells.index(node)][cells.index([x+1,y])]=1
        
        if x-1 in range(0,18) and plainMatrix[x-1][y]!='X' and  plainMatrix[x][y]!='X' :
            plain_adjacentGraph[cells.index(node)][cells.index([x-1,y])]=1
        
        if y+1 in range(0,21) and plainMatrix[x][y+1]!='X' and  plainMatrix[x][y]!='X' :
            plain_adjacentGraph[cells.index(node)][cells.index([x,y+1])]=1

        if y-1 in range(0,21) and plainMatrix[x][y-1]!='X' and  plainMatrix[x][y]!='X' :
            plain_adjacentGraph[cells.index(node)][cells.index([x,y-1])]=1

        
        
        
    plain_class_Graph=Graph()
    priorityValue = dict()
    rowNumber=0   
    for row in plain_adjacentGraph:
        plain_class_Graph.add_node(rowNumber)
        rowNumber = rowNumber + 1
    n = rowNumber
    print(n)
    for i in range(0, n):
        for j in range(0, n):
            if plain_adjacentGraph[i, j] != 0:
                plain_class_Graph.add_edge(i, j, plain_adjacentGraph[i, j])
        priorityValue[i]=0.01

    #finding voronoi partition for robots
    position_Robot1=cells.index([0,0])
    position_Robot2=cells.index([0,20])
    

#Initial partitioning to deploy agents to learn the environment
    voronoiSubsets = Voronoi(n,plain_class_Graph, position_Robot1, position_Robot2)
    initial_subnodes_Robot1 = voronoiSubsets[0]
    initial_subnodes_Robot2 = voronoiSubsets[1]

    R1_partition=[]
    for v in initial_subnodes_Robot1:
        R1_partition.append(cells[v])
    R2_partition=[]
    for v in initial_subnodes_Robot2:
        R2_partition.append(cells[v])
    
    partition=[]
    for i in range(0,18):
        partition.append([])
        for j in range(0,21):
            if [i,j]==cells[position_Robot1]:
                partition[-1].append('1')
            if [i,j]==cells[position_Robot2]:
                partition[-1].append('2')                
            if [i,j] in R1_partition: 
                partition[-1].append('o')
            elif [i,j] in R2_partition:
                partition[-1].append(' ')
    for row in partition:
        print(row)
    print('')
    
       
    return  R1_partition,R2_partition,cells
     
     
     
     
def plotMaze(maze1,maze2,obstacles,PathRobot1,PathRobot2,name,label):
    print('I am plotting')
    nrows = maze1.WORLD_HEIGHT
    ncols = maze1.WORLD_WIDTH
    cells=[]
    for i in range(maze1.WORLD_HEIGHT):
        for j in range(maze1.WORLD_WIDTH):
           cells.append([i,j])
    X=[]
    Y=[]
    Rx=0
    Ry=0
    for i in range(0,nrows):
        for j in range(0,ncols):
            x=Rx
            y=Ry
            X.append(x)
            Y.append(y)
            Rx=Rx+1
        Rx=0
        Ry=Ry+1

          
    Cellid = []
    Cellval = []
    plantRows=[[1,1],[1,2],[1,3],[1,5],[1,6],[1,7],[1,9],[1,10],[1,11],[1,13],[1,14],[1,15],[1,17],[1,18],[1,19],
[4,1],[4,2],[4,3],[4,5],[4,6],[4,7],[4,9],[4,10],[4,11],[4,13],[4,14],[4,15],[4,17],[4,18],[4,19],
[7,1],[7,2],[7,3],[7,5],[7,6],[7,7],[7,9],[7,10],[7,11],[7,13],[7,14],[7,15],[7,17],[7,18],[7,19],
[10,1],[10,2],[10,3],[10,5],[10,6],[10,7],[10,9],[10,10],[10,11],[10,13],[10,14],[10,15],[10,17],[10,18],[10,19],
[13,1],[13,2],[13,3],[13,5],[13,6],[13,7],[13,9],[13,10],[13,11],[13,13],[13,14],[13,15],[13,17],[13,18],[13,19],
[16,1],[16,2],[16,3],[16,5],[16,6],[16,7],[16,9],[16,10],[16,11],[16,13],[16,14],[16,15],[16,17],[16,18],[16,19]]
           
    goalStates=[]
    for goal in maze1.GOAL_STATES:
        goalStates.append(goal)
    #for goal in maze2.GOAL_STATES:
        #goalStates.append(goal)
    print('goal:',goalStates)
    
    startState=[]
    startState.append(maze1.START_STATE)
    startState.append(maze2.START_STATE)
    
    
    for node in startState:
        Cellid.append(cells.index(node))
        Cellval.append(0)   

    for node in goalStates:
        Cellid.append(cells.index(node))
        Cellval.append(1)
        
    for node in obstacles:
        if node in plantRows:
            Cellid.append(cells.index(node))
            Cellval.append(2)
        elif node not in plantRows:
            Cellid.append(cells.index(node))
            Cellval.append(3)


    data = np.zeros(nrows*ncols)
    data[Cellid] = Cellval
    data = np.ma.array(data.reshape((nrows, ncols)), mask=data==0)
    cmap = colors.ListedColormap(['white', 'yellow','green','red'])

    fig, ax = plt.subplots()
    ax.imshow(data, cmap=cmap, origin="lower" ,vmin=0)
  

    ax.set_xticks(np.arange(ncols+1)-0.5)
    ax.set_yticks(np.arange(nrows+1)-0.5)
    ax.grid(linestyle='-', linewidth='0.5', color='black')
#ax.tick_params(which="minor",color='black', size=0)
    
    ax.annotate('R1', xy=(X[cells.index(maze1.START_STATE)]-0.3,Y[cells.index(maze1.START_STATE)]-0.15), size=12)
    ax.annotate('R2', xy=(X[cells.index(maze2.START_STATE)]-0.3,Y[cells.index(maze2.START_STATE)]-0.15), size=12)
    #ax.annotate('Exit', xy=(X[cells.index(maze2.terminal_STATES[0])]-0.4,Y[cells.index(maze2.terminal_STATES[0])]-0.15), size=12)
    ax.annotate('Exit', xy=(X[cells.index([17,11])]-0.4,Y[cells.index([17,11])]-0.15), size=12)
    
    for node in goalStates:
        ax.annotate('Goal', xy=(X[cells.index(node)]-0.4,Y[cells.index(node)]-0.15), size=11)


    for p in range(0,len(PathRobot1)-2):
    
        x1,y1=PathRobot1[p]
        x2,y2=PathRobot1[p+1]
        ax.annotate("", xy=(y2, x2), xytext=(y1, x1),arrowprops=dict(arrowstyle="->"))
    for p in range(0,len(PathRobot2)-2):
    
        x1,y1=PathRobot2[p]
        x2,y2=PathRobot2[p+1]
        ax.annotate("", xy=(y2, x2), xytext=(y1, x1),arrowprops=dict(arrowstyle="->"))
    mng = plt.gcf()
    mng.set_size_inches(16,9)
    plt.xlabel(label, size=16)
    plt.plot()
    plt.savefig(name +'.png')
    plt.close() 
    
    
def plotMaze_voronoi(maze1,maze2,obstacles,PathRobot1,PathRobot2,voronoi1,voronoi2,name,label):
    print('I am plotting')
    nrows = maze1.WORLD_HEIGHT
    ncols = maze1.WORLD_WIDTH
    cells=[]
    for i in range(maze1.WORLD_HEIGHT):
        for j in range(maze1.WORLD_WIDTH):
           cells.append([i,j])
    X=[]
    Y=[]
    Rx=0
    Ry=0
    for i in range(0,nrows):
        for j in range(0,ncols):
            x=Rx
            y=Ry
            X.append(x)
            Y.append(y)
            Rx=Rx+1
        Rx=0
        Ry=Ry+1

    Cellid = []
    Cellval = []
       
    goalStates=[]
    for goal in maze1.GOAL_STATES:
        goalStates.append(goal)
    for goal in maze2.GOAL_STATES:
        goalStates.append(goal)
    print('goal:',goalStates)
    
    startState=[]
    startState.append(maze1.START_STATE)
    startState.append(maze2.START_STATE)
    
    
    for node in startState:
        Cellid.append(cells.index(node))
        Cellval.append(0)   

    for node in goalStates:
        Cellid.append(cells.index(node))
        Cellval.append(1)
        
    for node in obstacles:
        
        Cellid.append(cells.index(node))
        Cellval.append(2)
        
    for node in voronoi1:
        Cellid.append(cells.index(node))
        Cellval.append(3)
    for node in voronoi2:
        Cellid.append(cells.index(node))
        Cellval.append(4)

    data = np.zeros(nrows*ncols)
    data[Cellid] = Cellval
    data = np.ma.array(data.reshape((nrows, ncols)), mask=data==0)
    cmap = colors.ListedColormap(['white', 'yellow','green','lightgray','sandybrown'])

    fig, ax = plt.subplots()
    ax.imshow(data, cmap=cmap,origin="lower", vmin=0)
  

    ax.set_xticks(np.arange(ncols+1)-0.5)
    ax.set_yticks(np.arange(nrows+1)-0.5)
    ax.grid(linestyle='-', linewidth='0.5', color='black')
#ax.tick_params(which="minor",color='black', size=0)
    
    ax.annotate('R1', xy=(X[cells.index(maze1.START_STATE)]-0.3,Y[cells.index(maze1.START_STATE)]-0.15), size=12)
    ax.annotate('R2', xy=(X[cells.index(maze2.START_STATE)]-0.3,Y[cells.index(maze2.START_STATE)]-0.15), size=12)
    #ax.annotate('Exit', xy=(X[cells.index(maze2.terminal_STATES[0])]-0.4,Y[cells.index(maze2.terminal_STATES[0])]-0.15), size=12)
    ax.annotate('Exit', xy=(X[cells.index([17,11])]-0.4,Y[cells.index([17,11])]-0.15), size=12)    
    for node in goalStates:
        ax.annotate('Goal', xy=(X[cells.index(node)]-0.4,Y[cells.index(node)]-0.15), size=11)


    for p in range(0,len(PathRobot1)-2):
    
        x1,y1=PathRobot1[p]
        x2,y2=PathRobot1[p+1]
        ax.annotate("", xy=(y2, x2), xytext=(y1, x1),arrowprops=dict(arrowstyle="->"))
    for p in range(0,len(PathRobot2)-2):
    
        x1,y1=PathRobot2[p]
        x2,y2=PathRobot2[p+1]
        ax.annotate("", xy=(y2, x2), xytext=(y1, x1),arrowprops=dict(arrowstyle="->"))
    mng = plt.gcf()
    mng.set_size_inches(16,9)
    plt.xlabel(label, size=16)
    plt.plot()
    plt.savefig(name +'.png')
    plt.close() 

def plotMaze_voronoi2(maze1,maze2,obstacles,PathRobot1,PathRobot2,voronoi1,voronoi2,name,label):
    print('I am plotting')
    nrows = maze1.WORLD_HEIGHT
    ncols = maze1.WORLD_WIDTH
    cells=[]
    for i in range(maze1.WORLD_HEIGHT):
        for j in range(maze1.WORLD_WIDTH):
           cells.append([i,j])
    X=[]
    Y=[]
    Rx=0
    Ry=0
    for i in range(0,nrows):
        for j in range(0,ncols):
            x=Rx
            y=Ry
            X.append(x)
            Y.append(y)
            Rx=Rx+1
        Rx=0
        Ry=Ry+1
    plantRows=[[1,1],[1,2],[1,3],[1,5],[1,6],[1,7],[1,9],[1,10],[1,11],[1,13],[1,14],[1,15],[1,17],[1,18],[1,19],
[4,1],[4,2],[4,3],[4,5],[4,6],[4,7],[4,9],[4,11],[4,13],[4,14],[4,15],[4,17],[4,18],[4,19],
[7,1],[7,2],[7,3],[7,5],[7,6],[7,7],[7,9],[7,10],[7,11],[7,13],[7,14],[7,15],[7,17],[7,18],[7,19],
[10,1],[10,2],[10,3],[10,5],[10,6],[10,7],[10,9],[10,10],[10,11],[10,13],[10,14],[10,15],[10,17],[10,18],[10,19],
[13,1],[13,2],[13,3],[13,5],[13,6],[13,7],[13,9],[13,10],[13,11],[13,13],[13,14],[13,15],[13,17],[13,18],[13,19],
[16,1],[16,2],[16,3],[16,5],[16,6],[16,7],[16,9],[16,10],[16,11],[16,13],[16,14],[16,15],[16,17],[16,18],[16,19]]
    newobs=[[5,15],[11,20],[16,3]]
    Cellid = []
    Cellval = []
       
    goalStates=[]
    for goal in maze1.GOAL_STATES:
        goalStates.append(goal)
    for goal in maze2.GOAL_STATES:
        goalStates.append(goal)
    print('goal:',goalStates)
    
    startState=[]
    startState.append(maze1.START_STATE)
    startState.append(maze2.START_STATE)
    
    
    for node in startState:
        Cellid.append(cells.index(node))
        Cellval.append(0)   

    for node in goalStates:
        Cellid.append(cells.index(node))
        Cellval.append(1)
        
    for node in obstacles:
        if node in plantRows:
            Cellid.append(cells.index(node))
            Cellval.append(2)
        else:
            Cellid.append(cells.index(node))
            Cellval.append(5)
        
    for node in newobs:
        Cellid.append(cells.index(node))
        Cellval.append(5)            
        
    for node in voronoi1:
        Cellid.append(cells.index(node))
        Cellval.append(3)
    for node in voronoi2:
        Cellid.append(cells.index(node))
        Cellval.append(4)

    data = np.zeros(nrows*ncols)
    data[Cellid] = Cellval
    data = np.ma.array(data.reshape((nrows, ncols)), mask=data==0)
    cmap = colors.ListedColormap(['white', 'yellow','green','lightgray','sandybrown','red'])

    fig, ax = plt.subplots()
    ax.imshow(data, cmap=cmap,origin="lower", vmin=0)
  

    ax.set_xticks(np.arange(ncols+1)-0.5)
    ax.set_yticks(np.arange(nrows+1)-0.5)
    ax.grid(linestyle='-', linewidth='0.5', color='black')
#ax.tick_params(which="minor",color='black', size=0)
    
    ax.annotate('R1', xy=(X[cells.index(maze1.START_STATE)]-0.3,Y[cells.index(maze1.START_STATE)]-0.15), size=12)
    ax.annotate('R2', xy=(X[cells.index(maze2.START_STATE)]-0.3,Y[cells.index(maze2.START_STATE)]-0.15), size=12)
    #ax.annotate('Exit', xy=(X[cells.index(maze2.terminal_STATES[0])]-0.4,Y[cells.index(maze2.terminal_STATES[0])]-0.15), size=12)
    ax.annotate('Exit', xy=(X[cells.index([17,11])]-0.4,Y[cells.index([17,11])]-0.15), size=12)    
    for node in goalStates:
        ax.annotate('Goal', xy=(X[cells.index(node)]-0.4,Y[cells.index(node)]-0.15), size=11)


    for p in range(0,len(PathRobot1)-2):
    
        x1,y1=PathRobot1[p]
        x2,y2=PathRobot1[p+1]
        ax.annotate("", xy=(y2, x2), xytext=(y1, x1),arrowprops=dict(arrowstyle="->"))
    for p in range(0,len(PathRobot2)-2):
    
        x1,y1=PathRobot2[p]
        x2,y2=PathRobot2[p+1]
        ax.annotate("", xy=(y2, x2), xytext=(y1, x1),arrowprops=dict(arrowstyle="->"))
    mng = plt.gcf()
    mng.set_size_inches(16,9)
    plt.xlabel(label, size=16)
    plt.plot()
    plt.savefig(name +'.png')
    plt.close() 

     
#start learning the environment.........................................................................................
DynaQ=DynaQ_Plus()
finalMatrix=DynaQ[0] # The learned model of environment
cells=DynaQ[1]
currentState_1=DynaQ[2]
currentState_2=DynaQ[3]

finalMatrix[5][15]='X'
finalMatrix[11][20]='X'
finalMatrix[8][0]='X'
finalMatrix[5][4]='X'
finalMatrix[16][3]=' '      
#Generate Adjacent Graph of field including found obstacles and goals    
adjacentGraph=np.zeros([378,378])
for node in cells:
    x,y=node
    if x+1 in range(0,18) and finalMatrix[x+1][y]!='X' and  finalMatrix[x][y]!='X' :
        adjacentGraph[cells.index(node)][cells.index([x+1,y])]=1
        
    if x-1 in range(0,18) and finalMatrix[x-1][y]!='X' and  finalMatrix[x][y]!='X' :
        adjacentGraph[cells.index(node)][cells.index([x-1,y])]=1
        
    if y+1 in range(0,21) and finalMatrix[x][y+1]!='X' and  finalMatrix[x][y]!='X' :
        adjacentGraph[cells.index(node)][cells.index([x,y+1])]=1

    if y-1 in range(0,21) and finalMatrix[x][y-1]!='X' and  finalMatrix[x][y]!='X' :
        adjacentGraph[cells.index(node)][cells.index([x,y-1])]=1
 
#Defined goals after learning the environment
goalList=[]    
for cell in cells:
    x,y=cell
    if finalMatrix[x][y]=='G':
        goalList.append([x,y])
        
class_Graph=Graph()
priorityValue = dict()
rowNumber=0   
for row in adjacentGraph:
    class_Graph.add_node(rowNumber)
    rowNumber = rowNumber + 1
n = rowNumber
print(n)
for i in range(0, n):
    for j in range(0, n):
        if adjacentGraph[i, j] != 0:
            class_Graph.add_edge(i, j, adjacentGraph[i, j])
    priorityValue[i]=0.01

#finding voronoi partition for robots
position_Robot1=cells.index([0,0])
position_Robot2=cells.index([0,20])

goal1=cells.index([11,2])
goal2=cells.index([17,14])
goal3=cells.index([11,15])

priorityValue[goal1]=10000000
priorityValue[goal2]=100000000
priorityValue[goal3]=100000

priorityValue[cells.index([11,1])]=100
priorityValue[cells.index([17,13])]=1000
priorityValue[cells.index([16,12])]=1000
priorityValue[cells.index([10,12])]=100
priorityValue[cells.index([11,13])]=100
priorityValue[cells.index([14,10])]=100


voronoiSubsets = Voronoi(n,class_Graph, position_Robot1, position_Robot2)
initial_subnodes_Robot1 = voronoiSubsets[0]
initial_subnodes_Robot2 = voronoiSubsets[1]
print('Robot1',initial_subnodes_Robot1)
print('Robot2',initial_subnodes_Robot2) 
    
R1_partition=[]
for v in initial_subnodes_Robot1:
    R1_partition.append(cells[v])
R2_partition=[]
for v in initial_subnodes_Robot2:
    R2_partition.append(cells[v])
    
step=list()
Cost=list()
path_Robot1=list()
path_Robot2 = list()

for p in range(1,30):

    if p == 1:
        voronoiSubsets = Voronoi(n,class_Graph, position_Robot1, position_Robot2)
        initial_subnodes_Robot1 = voronoiSubsets[0]
        initial_subnodes_Robot2 = voronoiSubsets[1]
         

    else:
        voronoiSubsets = Voronoi(n,class_Graph,nextBest_position1[0], nextBest_position2[0])
    subnodes_Robot1 = voronoiSubsets[0]
    subnodes_Robot2 = voronoiSubsets[1]
    if p == 1:
        nextBest_position1 = finding_nextPoint(adjacentGraph,class_Graph,priorityValue, subnodes_Robot1, position_Robot1, n)
        nextBest_position2 = finding_nextPoint(adjacentGraph,class_Graph,priorityValue, subnodes_Robot2, position_Robot2, n)
            #if nextBest_position1[0] != position_Robot1:
        path_Robot1.append(nextBest_position1[0])
            # if nextBest_position2[0] not in path_Robot2:
         #if nextBest_position2[0] != position_Robot2:
        path_Robot2.append(nextBest_position2[0])
    #else:
     #   if nextBest_position1[0] == goal2 or nextBest_position2[0] == goal2:
      #      print('stooooooppppppp')
                    #break
    else:
        nextBest_position1 = finding_nextPoint(adjacentGraph,class_Graph,priorityValue, subnodes_Robot1, nextBest_position1[0], n)
        nextBest_position2 = finding_nextPoint(adjacentGraph,class_Graph,priorityValue, subnodes_Robot2, nextBest_position2[0], n)
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



print('First partitioning after learning the environment:')
partition=[]
for i in range(0,18):
    partition.append([])
    for j in range(0,21):
        if [i,j] in goalList:
            partition[-1].append('G')
        if [i,j]==position_Robot1:
            partition[-1].append('1')
        if [i,j]==position_Robot2:
            partition[-1].append('2')                
        if [i,j] in R1_partition and [i,j] not in goalList : 
            partition[-1].append('o')
        elif [i,j] in R2_partition and [i,j] not in goalList:
            partition[-1].append(' ')
        elif [i,j] not in goalList and [i,j] not in R1_partition and [i,j] not in R2_partition:
            partition[-1].append('X')
for row in partition:
    print(row)
print('')

print('goal1=',cells[goal1])
print('goal2=',cells[goal2])
#print('goal2=',goal2)
#print('goal3=',goal3)
final_path1=[]
for path in path_Robot1:
    final_path1.append(cells[path])
final_path2=[]
for path in path_Robot2:
    final_path2.append(cells[path])
print('Final path Robot1:',final_path1)
print('Final path Robot2:',final_path2)  
print('Final partitioning after learning the environment:')    
R1_partition=[]
for v in subnodes_Robot1:
    R1_partition.append(cells[v])
R2_partition=[]
for v in subnodes_Robot2:
    R2_partition.append(cells[v])
    
partition=[]
for i in range(0,18):
    partition.append([])
    for j in range(0,21):
        if [i,j] in goalList:
            partition[-1].append('G')
        if [i,j]==position_Robot1:
            partition[-1].append('1')
        if [i,j]==position_Robot2:
            partition[-1].append('2')                
        if [i,j] in R1_partition and [i,j] not in goalList : 
            partition[-1].append('o')
        elif [i,j] in R2_partition and [i,j] not in goalList:
            partition[-1].append(' ')
        elif [i,j] not in goalList and [i,j] not in R1_partition and [i,j] not in R2_partition:
            partition[-1].append('X')
for row in partition:
    print(row)
print('')
plotMaze_voronoi2(DynaQ[4],DynaQ[5],DynaQ[6],final_path1,final_path2,R1_partition,R2_partition,'path','final path')



