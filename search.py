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

#Travis Mewborne 2/15/2022
def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """

    """
    Methods available under searchAgents.py/PositionSearchProblem
        Useful Methods:
        problem.getStartState()    -> returns the initial state
        problem.isGoalState(state) -> returns if the state passed in is the goal state
        problem.getSuccessors      -> returns list of children states

        Good to know:
        state       -> (xpos,ypos)
        branch      -> (state,action,weight)


    Stack element (path) format: [node, leadsToGoal, visited, directions, weight]
        node        -> the current node, stored as                                  -> (int xpos, int ypos)
        leadsToGoal -> whether or not the path ends at the goal                     -> Boolean goal
        visited     -> nodes that have already been explored alone the current path -> [(int xpos, int ypos)]
        directions  -> a list of actions to the current node                        -> [(string action)]
        weight      -> how expensive the path is                                    -> int weight
    """

    #Change this to switch between methods. 
    # "True"  will use global visitors     (faster, high cost solution possible)
    # "False" will use individual visitors (slower, lowest cost solution)
    globalVisitors = True

    print("[search.py/depthFirstSearch] Generating path using a stack and \x1b[34m",end="")

    if globalVisitors:
        print("global visitors\x1b[0m...")
    else:
        print("individual visitors\x1b[0m...")

    path=[]

    if globalVisitors:
         path=depthFirstSearchGlobalVisitor(problem)
    else:
         path=depthFirstSearchIndividualVisitor(problem)

    node = path[0]
    leadsToGoal = path[1]
    visited=path[2]
    directions=path[3]
    weight=path[4]

    print("[search.py/depthFirstSearch] Generated the following:")
    print("[search.py/depthFirstSearch] \t Goal Found?\t ",end="")
    if leadsToGoal:
        print("\x1b[32mYes\x1b[0m")
    else:
        print("\x1b[31mNo\x1b[0m")
    print("[search.py/depthFirstSearch] \t # Directions?\t", str(len(directions)))
    print("[search.py/depthFirstSearch] \t Weight?\t", str(weight))

    return directions


#Travis Mewborne 2/16/21
#This method is slow but provides the lowest cost answer
def depthFirstSearchIndividualVisitor(problem):
    """    
    Stack element (path) format: [node, leadsToGoal, visited, directions, weight]
        node        -> the current node, stored as                                  -> (int xpos, int ypos)
        leadsToGoal -> whether or not the path ends at the goal                     -> Boolean goal
        visited     -> nodes that have already been explored alone the current path -> [(int xpos, int ypos)]
        directions  -> a list of actions to the current node                        -> [(string action)]
        weight      -> how expensive the path is                                    -> int weight
    """

    s = util.Stack()

    bestPath = [problem.getStartState(), False, [], [], 0]
    s.push(bestPath)
    counter=0
    while not s.isEmpty():
        counter+=1
        #Pop the top node
        path = s.pop()
        node = path[0]
        leadsToGoal = path[1]
        visited=path[2]
        directions=path[3]
        weight=path[4]

        #Determine if this is the best path thus far (and replace if necessary)
        bestPathLeadsToGoal = bestPath[1]
        bestPathWeight = bestPath[4]
        isBetter = (not bestPathLeadsToGoal and leadsToGoal) or (leadsToGoal and weight<bestPathWeight)
        if isBetter:
            bestPath=path

        #If this is not a goal state, push each child that has not been visited
        if not leadsToGoal:
            for branch in problem.getSuccessors(node):
                nodeChild=branch[0]
                if not nodeChild in visited: #Don't push an existing node
                    isGoal = problem.isGoalState(nodeChild)
                    directionChild=branch[1]
                    weightChild=branch[2]
                    visitedChild=visited.copy()
                    visitedChild.append(node)
                    directionsChild=directions.copy()
                    directionsChild.append(directionChild)
                    s.push([nodeChild,isGoal,visitedChild,directionsChild,(weight+weightChild)])
    return bestPath

#Travis Mewborne 2/16/21
#This method is fast but may not provide the lowest cost answer
def depthFirstSearchGlobalVisitor(problem):
    """        
    Stack element (path) format: [node, leadsToGoal, directions, weight]
        node        -> the current node, stored as                                  -> (int xpos, int ypos)
        leadsToGoal -> whether or not the path ends at the goal                     -> Boolean goal
        directions  -> a list of actions to the current node                        -> [(string action)]
        weight      -> how expensive the path is                                    -> int weight
    """
    s = util.Stack()
    visited = []
    bestPath = [problem.getStartState(), False, [], 0]
    s.push(bestPath)
    counter=0
    while not s.isEmpty():
        counter+=1
        #Pop the top node
        path = s.pop()
        node = path[0]
        leadsToGoal = path[1]
        directions=path[2]
        weight=path[3]

        #Determine if this is the best path thus far (and replace if necessary)
        bestPathLeadsToGoal = bestPath[1]
        bestPathWeight = bestPath[3]
        isBetter = (not bestPathLeadsToGoal and leadsToGoal) or (leadsToGoal and weight<bestPathWeight)
        if isBetter:
            bestPath=path

        #If this is not a goal state, push each child that has not been visited
        if not leadsToGoal:
            for branch in problem.getSuccessors(node):
                nodeChild=branch[0]
                if not nodeChild in visited: #Don't push an existing node
                    isGoal = problem.isGoalState(nodeChild)
                    directionChild=branch[1]
                    weightChild=branch[2]
                    directionsChild=directions.copy()
                    directionsChild.append(directionChild)
                    visited.append(nodeChild)
                    s.push([nodeChild,isGoal,directionsChild,(weight+weightChild)])
    return [bestPath[0],bestPath[1],visited,bestPath[2],bestPath[3]]


"""
#Travis Mewborne 2/15/22
def pathString(path):
    red =   "\x1b[31m"
    green = "\x1b[32m"
    reset = "\x1b[0m"
    cost = "".join(["Cost: ",str(path[0]), " | "])

    if path[1]:
        goal = "".join([green,"Goal Found!"])
    else:
        goal = "".join([red,"Goal not found."])

    goal = "".join([goal,reset," | "])

    directions="Directions: "
    if (len(path[2])>10):
        directions = "".join([directions,str(len(path[2]))])
    else:
        directions = "".join([directions,str(path[2])])


    #directions = "".join(["Directions: ", str(path[2])])
    s="".join([cost,goal,directions])
    return s"""

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    util.raiseNotDefined()

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
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
    util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
