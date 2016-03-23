# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
#
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    """
    "*** YOUR CODE HERE ***"
    # This code is written by Ankit Vora for the course CS188.x

    import pdb
    currentState = problem.getStartState()
    nodesVisited = util.Stack()
    nodesToBeVisited = util.Stack()
    currentPos = currentState

    "Declare parent and direction dictionaries"
    parent = {}
    direction = {}
    #pdb.set_trace()

    "Start DFS"
    while not problem.isGoalState(currentPos):
        #print 'Current State is :',currentPos
        nodesVisited.push(currentPos)
        currentSuccessors = problem.getSuccessors(currentPos)
        #print 'Current Successors are :',currentSuccessors

        "Loop through all successors of a node"
        for successor in currentSuccessors:
            if successor[0] in nodesVisited.list:
                continue
            elif successor[0] in nodesToBeVisited.list:
                parent[successor[0]] = currentPos
                direction[successor[0]] = successor[1]
                continue
            elif successor[0] not in nodesToBeVisited.list:
                nodesToBeVisited.push(successor[0])
                parent[successor[0]] = currentPos
                direction[successor[0]] = successor[1]

        "Choose the node based on LIFO by popping the stack"
        currentPos = nodesToBeVisited.pop();
    goal = currentPos
    #print 'Final current position is :',currentPos
    path = []
    way = []
    current = goal

    "Retrace back to get the path"
    while current!=problem.getStartState():
        path.append(current)
        way.append(direction[current])
        current = parent[current]
    path.append(problem.getStartState())
    path.reverse()
    way.reverse()
    return way
    util.raiseNotDefined()

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    # This code is written by Ankit Vora for the course CS188.x

    import pdb
    try:
        problem.count
    except AttributeError:
        flag = False
    else:
        flag = True
    if flag==False:

        "Similar to previous code but used Queue instead of Stacks"
        currentState = problem.getStartState()
        nodesVisited = util.Queue()
        nodesToBeVisited = util.Queue()
        currentPos = currentState
        parent = {}
        direction = {}
        #pdb.set_trace()
        while not problem.isGoalState(currentPos):
            #print 'Current State is :',currentPos
            nodesVisited.push(currentPos)
            currentSuccessors = problem.getSuccessors(currentPos)
            #print 'Current Successors are :',currentSuccessors
            for successor in currentSuccessors:
                if successor[0] in nodesVisited.list:
                    continue
                elif successor[0] in nodesToBeVisited.list:
                    continue
                elif successor[0] not in nodesToBeVisited.list:
                    nodesToBeVisited.push(successor[0])
                    parent[successor[0]] = currentPos
                    direction[successor[0]] = successor[1]
            currentPos = nodesToBeVisited.pop();
        goal = currentPos
        #print 'Final current position is :',currentPos
        path = []
        way = []
        current = goal
        while current!=problem.getStartState():
            path.append(current)
            way.append(direction[current])
            current = parent[current]
        path.append(problem.getStartState())
        path.reverse()
        way.reverse()
        return way
        util.raiseNotDefined()
    else:
        startState = problem.getStartState()
        nodesVisited = util.Queue()
        nodesToBeVisited = util.Queue()
        currentPos = startState
        parent = {}
        direction = {}
        finalPath = []
        while problem.count!=4:
            #pdb.set_trace()
            problem.goal = problem.corners[problem.goalOrder[problem.count]]
            while problem.goal!=currentPos:
                #print problem.goal
                #print 'Current State is :',currentPos
                nodesVisited.push(currentPos)
                currentSuccessors = problem.getSuccessors(currentPos)
                #print 'Current Successors are :',currentSuccessors
                for successor in currentSuccessors:
                    if successor[0] in nodesVisited.list:
                        continue
                    elif successor[0] in nodesToBeVisited.list:
                        continue
                    elif successor[0] not in nodesToBeVisited.list:
                        nodesToBeVisited.push(successor[0])
                        parent[successor[0]] = currentPos
                        direction[successor[0]] = successor[1]
                currentPos = nodesToBeVisited.pop()
            goal = currentPos
            path = []
            way = []
            current = goal
            while current!=startState:
                path.append(current)
                way.append(direction[current])
                current = parent[current]
            path.append(startState)
            path.reverse()
            way.reverse()
            finalPath.append(way)
            startState = goal
            nodesVisited = util.Queue()
            nodesToBeVisited = util.Queue()
            parent = {}
            #print 'is here'
            #print problem.goalState
            direction = {}
            problem.count = problem.count + 1
            print 'count is :',problem.count
        merged = []
        for element in finalPath:
            merged = merged + element
        finalPath = merged
        print finalPath
        return finalPath
        util.raiseNotDefined()

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    #This code is written by Ankit Vora for the course CS188.x

    "Implement UCS based on the true cost to any given node"
    import pdb
    currentState = problem.getStartState()
    nodesVisited = util.Queue()

    "Need to use priority Queue for the nodesToBeVisited variable"
    nodesToBeVisited = util.PriorityQueue()
    currentPos = currentState
    costToCurrentNode = 0
    costMap = {}
    costMap[currentPos] = costToCurrentNode
    parent = {}
    direction = {}
    #pdb.set_trace()

    "Start UCS implementation"
    while not problem.isGoalState(currentPos):
        #print 'Current State is :',currentPos
        nodesVisited.push(currentPos)
        currentSuccessors = problem.getSuccessors(currentPos)
        #print 'Current Successors are :',currentSuccessors

        "Loop through all successors of current node"
        for successor in currentSuccessors:
            if successor[0] in nodesVisited.list:
                continue
            elif successor[0] in (val[2] for val in nodesToBeVisited.heap):
                costToCurrentSuccessor = costToCurrentNode + successor[2]
                if costToCurrentSuccessor<costMap[successor[0]]:
                    parent[successor[0]] = currentPos
                    direction[successor[0]] = successor[1]
                continue

            else:
                "Update the cost to successors"
                costToCurrentSuccessor = costToCurrentNode + successor[2]
                costMap[successor[0]] = costToCurrentSuccessor

                "Add successor to the nodesToBeVisited priority Queue"
                nodesToBeVisited.push(successor[0],costToCurrentSuccessor)
                parent[successor[0]] = currentPos
                direction[successor[0]] = successor[1]
        "Choose new node"
        currentPos = nodesToBeVisited.pop();
        costToCurrentNode = costMap[currentPos]
    goal = currentPos
    #print 'Final current position is :',currentPos
    path = []
    way = []
    current = goal

    "Get the path by retracing"
    while current!=problem.getStartState():
        path.append(current)
        way.append(direction[current])
        current = parent[current]
    path.append(problem.getStartState())
    path.reverse()
    way.reverse()
    return way
    util.raiseNotDefined()

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    #This code is written by Ankit Vora for the course CS188.x

    "Implement Astar"
    import pdb

    try:
        problem.count
    except AttributeError:
        flagA = False
    else:
        flagA = True
    try:
        problem.heuristicInfo
    except AttributeError:
        flagB = False
    else:
        flagB = True
    if flagA==True:
        startState = problem.getStartState()
        nodesVisited = util.Queue()
        nodesToBeVisited = util.PriorityQueue()
        currentPos = startState
        finalPath = []

        "Implement Astar to eat food at four corners"
        while problem.count!=4:
            problem.goal = problem.corners[problem.goalOrder[problem.count]]
            costToCurrentNode = 0
            costFromStartMap = {}
            totalCostMap = {}
            costFromStartMap[currentPos] = costToCurrentNode
            totalCostMap[currentPos] = costToCurrentNode + heuristic(currentPos,problem)
            parent = {}
            direction = {}
            #pdb.set_trace()
            while problem.goal!=currentPos:
                nodesVisited.push(currentPos)
                currentSuccessors = problem.getSuccessors(currentPos)
                for successor in currentSuccessors:
                    if successor[0] in nodesVisited.list:
                        continue
                    elif successor[0] in (val[2] for val in nodesToBeVisited.heap):
                        costToCurrentSuccessor = costToCurrentNode + successor[2]
                        totalCostToCurrentSuccessor = costToCurrentSuccessor + heuristic(successor[0],problem)
                        if totalCostToCurrentSuccessor<totalCostMap[successor[0]]:
                            costFromStartMap[successor[0]] = costToCurrentSuccessor
                            totalCostMap[successor[0]] = totalCostToCurrentSuccessor
                            nodesToBeVisited.push(successor[0],totalCostToCurrentSuccessor)
                            parent[successor[0]] = currentPos
                            direction[successor[0]] = successor[1]
                        continue
                    else:
                        costToCurrentSuccessor = costToCurrentNode + successor[2]

                        "Total cost here is cost to node + heuristic"
                        totalCostToCurrentSuccessor = costToCurrentSuccessor + heuristic(successor[0],problem)
                        costFromStartMap[successor[0]] = costToCurrentSuccessor
                        totalCostMap[successor[0]] = totalCostToCurrentSuccessor
                        nodesToBeVisited.push(successor[0],totalCostToCurrentSuccessor)
                        parent[successor[0]] = currentPos
                        direction[successor[0]] = successor[1]
                currentPos = nodesToBeVisited.pop();
                costToCurrentNode = costFromStartMap[currentPos]
            goal = currentPos
            #print 'goal is :',problem.goal
            #print 'heuristic is :',heuristic(currentPos,problem)
            path = []
            way = []
            current = goal

            "Get the path"
            while current!=startState:
                path.append(current)
                way.append(direction[current])
                current = parent[current]
            path.append(startState)
            path.reverse()
            way.reverse()
            finalPath.append(way)
            startState = goal
            nodesVisited = util.Queue()
            nodesToBeVisited = util.PriorityQueue()
            parent = {}
            #print 'is here'
            #print problem.goalState
            direction = {}
            problem.count = problem.count + 1
            #print 'count is :',problem.count
        merged = []
        for element in finalPath:
            merged = merged + element
        finalPath = merged
        return finalPath
        util.raiseNotDefined()
    elif flagB==True:
        startState = problem.getStartState()
        currentPos = startState[0]
        currentState = startState
        finalPath = []
        while not problem.isGoalState(currentState):
            nodesVisited = util.Queue()
            nodesToBeVisited = util.PriorityQueue()
            costToCurrentNode = 0
            costFromStartMap = {}
            totalCostMap = {}
            costFromStartMap[currentPos] = costToCurrentNode
            problem.goal = problem.getGoal(currentState)
            #print problem.goal
            totalCostMap[currentPos] = costToCurrentNode + heuristic(currentState,problem)
            #pdb.set_trace()
            parent = {}
            direction = {}
            #print 'Current State is :',currentPos
            while currentPos!=problem.goal:
                nodesVisited.push(currentPos)
                currentSuccessors = problem.getSuccessors(currentState)

                #print 'Current Successors are :',currentSuccessors
                for successor in currentSuccessors:
                    if successor[0][0] in nodesVisited.list:
                        continue
                    elif successor[0][0] in (val[2][0] for val in nodesToBeVisited.heap):
                        costToCurrentSuccessor = costToCurrentNode + successor[2]
                        totalCostToCurrentSuccessor = costToCurrentSuccessor + heuristic(successor[0],problem)
                        if totalCostToCurrentSuccessor<totalCostMap[successor[0][0]]:
                            costFromStartMap[successor[0][0]] = costToCurrentSuccessor
                            totalCostMap[successor[0][0]] = totalCostToCurrentSuccessor
                            nodesToBeVisited.push(successor[0],totalCostToCurrentSuccessor)
                            parent[successor[0][0]] = currentPos
                            direction[successor[0][0]] = successor[1]
                        continue

                    else:
                        costToCurrentSuccessor = costToCurrentNode + successor[2]
                        totalCostToCurrentSuccessor = costToCurrentSuccessor + heuristic(successor[0],problem)
                        costFromStartMap[successor[0][0]] = costToCurrentSuccessor
                        totalCostMap[successor[0][0]] = totalCostToCurrentSuccessor
                        nodesToBeVisited.push(successor[0],totalCostToCurrentSuccessor)
                        parent[successor[0][0]] = currentPos
                        direction[successor[0][0]] = successor[1]
                currentState = nodesToBeVisited.pop();
                currentPos = currentState[0]
                costToCurrentNode = costFromStartMap[currentPos]
            #pdb.set_trace()
            #print 'Final current position is :',currentPos
            path = []
            way = []
            current = problem.goal
            while current!=startState[0]:
                path.append(current)
                way.append(direction[current])
                current = parent[current]
            path.append(startState)
            path.reverse()
            way.reverse()
            finalPath.append(way)
            #pdb.set_trace()
            startState = currentState

            #print currentState[0],problem.getGoal(currentState),currentState[1].asList()
        merged = []
        for element in finalPath:
            merged = merged + element
        finalPath = merged
        return finalPath
        util.raiseNotDefined()
#Modified A star
    else:
        currentState = problem.getStartState()
        nodesVisited = util.Queue()
        nodesToBeVisited = util.PriorityQueue()
        currentPos = currentState
        costToCurrentNode = 0
        costFromStartMap = {}
        totalCostMap = {}
        costFromStartMap[currentPos] = costToCurrentNode
        #pdb.set_trace()
        totalCostMap[currentPos] = costToCurrentNode + heuristic(currentPos,problem)
        parent = {}
        direction = {}
        #pdb.set_trace()
        while not problem.isGoalState(currentPos):
            #print 'Current State is :',currentPos
            nodesVisited.push(currentPos)
            currentSuccessors = problem.getSuccessors(currentPos)
            #print 'Current Successors are :',currentSuccessors
            for successor in currentSuccessors:
                if successor[0] in nodesVisited.list:
                    continue
                elif successor[0] in (val[2] for val in nodesToBeVisited.heap):
                    costToCurrentSuccessor = costToCurrentNode + successor[2]
                    totalCostToCurrentSuccessor = costToCurrentSuccessor + heuristic(successor[0],problem)
                    if totalCostToCurrentSuccessor<totalCostMap[successor[0]]:
                        costFromStartMap[successor[0]] = costToCurrentSuccessor
                        totalCostMap[successor[0]] = totalCostToCurrentSuccessor
                        nodesToBeVisited.push(successor[0],totalCostToCurrentSuccessor)
                        parent[successor[0]] = currentPos
                        direction[successor[0]] = successor[1]
                    continue

                else:
                    costToCurrentSuccessor = costToCurrentNode + successor[2]
                    totalCostToCurrentSuccessor = costToCurrentSuccessor + heuristic(successor[0],problem)
                    costFromStartMap[successor[0]] = costToCurrentSuccessor
                    totalCostMap[successor[0]] = totalCostToCurrentSuccessor
                    nodesToBeVisited.push(successor[0],totalCostToCurrentSuccessor)
                    parent[successor[0]] = currentPos
                    direction[successor[0]] = successor[1]
            currentPos = nodesToBeVisited.pop();
            costToCurrentNode = costFromStartMap[currentPos]
        goal = currentPos
        #print 'Final current position is :',currentPos
        path = []
        way = []
        current = goal
        while current!=problem.getStartState():
            path.append(current)
            way.append(direction[current])
            current = parent[current]
        path.append(problem.getStartState())
        path.reverse()
        way.reverse()
        return way
        util.raiseNotDefined()



# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
