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
    "*** YOUR CODE HERE ***"

    """
    Methods available under searchAgents.py/PositionSearchProblem
        Useful Methods:
        problem.getStartState()    -> returns the initial state
        problem.isGoalState(state) -> returns if the state passed in is the goal state
        problem.getSuccessors      -> returns list of children states

        Good to know:
        state       -> (xpos,ypos)
        branch      -> (state,action,weight)
    """
    startState=problem.getStartState()      #Initial state
    visited=[]                              #Visited starting
    path=[]                                 #Path starting
    pathWeight=0                            #How expensive is the path? (cost)
    depthLevel=0                            #For tree printing purposes

    print("[search.py/depthFirstSearch] Generating path...",end="")
    path = depthFirstSearchHelper(problem,  startState,  visited,  path,  pathWeight, depthLevel)
    print("\x1b[32mDone\x1b[0m")
    print("[search.py/depthFirstSearch] Final Path: ", pathString(path))
    #depthFirstSearchHelper(problem,)

    return path[2]
    #util.raiseNotDefined()


#Travis Mewborne 2/15/2022
#The actual searching happens here
#TODO i need to add a stack/queue, apparently the autograder won't work without one of these data structures
def depthFirstSearchHelper(problem,node,visited,path,pathWeight,depthlevel): #node,visited,weightToVisit,directionToVisit):

    verbose=False #whether or not to print the tree

    if problem.isGoalState(node): #Base case 1 -> goal state
        return (pathWeight,True,path)

    if node in visited: #Base Case 2 -> Already been here
        """TODO
            I believe there is a bug in this part of the code. When running with medium maze, pacman takes longest posible route,
            i think that this is because the already visited nodes are ignored even if they provide a better path       
        """
        return (pathWeight,False,path)

    visited.append(node)

    bestPath = (-1,False,[])
    branches = problem.getSuccessors(node)

    for newBranch in branches:
        #Find a new path
        child, direction, weight = newBranch[0], newBranch[1], newBranch[2]
        newPath = path.copy()
        newPath.append(direction)
        childPath = depthFirstSearchHelper(problem,child,visited,newPath,pathWeight+weight,depthlevel+1)

        #Determine the better path
        childPathWeight = childPath[0]
        childGoalFound = childPath[1]
        bestPathWeight = bestPath[0]
        bestPathGoalFound = bestPath[1]
        isBetter = (bestPathWeight ==-1) or (childGoalFound and childPathWeight < bestPathWeight) or (childGoalFound and not bestPathGoalFound)

        #Print tree
        if verbose:
            for i in range(depthlevel):
                print("|", end="")
            if isBetter:
                print(depthlevel,"    \x1b[32m",childPath, "\x1b[0m vs \x1b[31m", bestPath)
            else:
                print(depthlevel,"    \x1b[31m",childPath, "\x1b[0m vs \x1b[32m", bestPath)

            print("\x1b[0m",end="")

        #Update bestPath
        if isBetter:
            bestPath=childPath 

    #Return final path
    return bestPath

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
    return s

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
