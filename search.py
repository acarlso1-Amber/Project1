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

    Methods available under searchAgents.py/PositionSearchProblem
        Useful Methods:
        problem.getStartState()    -> returns the initial state
        problem.isGoalState(state) -> returns if the state passed in is the goal state
        problem.getSuccessors      -> returns list of children states

    Good to know:
        state       -> (xpos,ypos)
        branch      -> (state,action,weight)

    Execution commands:
        Tiny Maze   -> python3.7 pacman.py -l tinyMaze -p SearchAgent
        Medium Maze -> python3.7 pacman.py -l mediumMaze -p SearchAgent
        Big Maze    -> python3.7 pacman.py -l bigMaze -z .5 -p SearchAgent
        Test (q1)   -> python3.7 autograder.py


    Stack element ("path") format: [node, leadsToGoal, visited, directions, weight]
        node        -> the current node, stored as                                  -> (int xpos, int ypos)
        leadsToGoal -> whether or not the path ends at the goal                     -> Boolean goal
        visited     -> nodes that have already been explored alone the current path -> [(int xpos, int ypos)]
        directions  -> a list of actions to the current node                        -> [(string action)]
        weight      -> how expensive the path is                                    -> int weight
    """

    verboseMode = False

    if verboseMode:
        print("[search.py/depthFirstSearch] Verbose Mode Enabled")
        print("[search.py/depthFirstSearch] Generating DFS path...")

    #exploreTheTree(problem,problem.getStartState(),[])

    s = util.Stack()

    bestPath = [problem.getStartState(), False, [], []] #Default path
    s.push(bestPath)
    goalHasBeenFound=False
    counter=0
    while not s.isEmpty(): #Evaluate until the stack is empty
        counter += 1

        #Pop the top node
        path = s.pop()
        node = path[0]
        leadsToGoal = path[1]
        visited=path[2]
        directions=path[3]

        if leadsToGoal:
            goalHasBeenFound=True

        #Determine if this is the best path thus far (and replace if necessary)
        bestPathLeadsToGoal = bestPath[1]
        isBetter = (not bestPathLeadsToGoal and leadsToGoal)
        if isBetter:
            bestPath=path

        #If this is not a goal state, push each child that has not been visited
        if not (leadsToGoal or goalHasBeenFound):
            
            for branch in problem.getSuccessors(node):
                nodeChild=branch[0]
                #print(node,"->",nodeChild," - move cost is ",weightChild," - total cost is ",weight+weightChild)
                if not nodeChild in visited: #Don't push an existing node
                    isGoal = problem.isGoalState(nodeChild)
                    directionChild=branch[1]
                    visitedChild=visited.copy()
                    visitedChild.append(node)
                    directionsChild=directions.copy()
                    directionsChild.append(directionChild)
                    s.push([nodeChild,isGoal,visitedChild,directionsChild])

    """
        End Main Algorithm
    """

    node = bestPath[0]
    leadsToGoal = bestPath[1]
    visited=bestPath[2]
    directions=bestPath[3]

    if verboseMode:
        print("[search.py/depthFirstSearch] Generated the following after", counter, "iterations:")
        print("[search.py/depthFirstSearch] --- Goal Found? ---- ",end="")
        if leadsToGoal:
            print("\x1b[32mYes\x1b[0m")
        else:
            print("\x1b[31mNo\x1b[0m")
        print("[search.py/depthFirstSearch] --- # Directions? --", str(len(directions)))

    return directions

#Travis Mewborne 2/17/22
#Useful for exploring the tree setup in testing
def exploreTheTree(problem,node,printed):
    if node in printed:
        return
    kids = problem.getSuccessors(node)
    printed.append(node)
    print(node,"->",kids)
    for kid in kids:
        exploreTheTree(problem,kid[0],printed)

#Amber Carlson
#BFS: 2/18/21
def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    # path = [node, leadsToGoal, recentlyVisited, visited, directions]
    # print("start state: ", problem.getStartState())

    q = util.Queue()
    currentNode = problem.getStartState()
    problem.currentPath = [currentNode, False, [], [], []]
    q.push(problem.currentPath)
    count, maxCount, found = 0, 1000000, False
    problem.breadCrumbs = []
    problem.allowDups = True   # toggles whether the Corners Problem can have repeated states in different paths

    while (not q.isEmpty()) and (not found) and (count < maxCount):
        count += 1
        problem.currentPath = q.pop()
        currentNode = problem.currentPath[0]
        problem.getStartState() # skeevy way to reset the bread crumbs in Corners Problem

        # base case
        if problem.isGoalState(currentNode):    # clears recentlyVisited if a new Corner is found
            found = True
            problem.currentPath[1] = True
        else:
            # add children to fringe
            recentlyVisited = problem.currentPath[2] + [currentNode]
            visited = problem.currentPath[3] + [currentNode]
            for child in problem.getSuccessors(currentNode):    # child = ( nextState, action, cost)
                if not child[0] in recentlyVisited and not child[0] in problem.breadCrumbs:
                    problem.breadCrumbs = problem.breadCrumbs + [child[0]]
                    action = (child[1])
                    directions = problem.currentPath[4] + [action]
                    q.push([child[0], False, recentlyVisited, visited, directions])
        # input()
        
    if not problem.currentPath[1]:
        print("Error: goal not found after examining: ", count, " states")
    

    # prints path taken for testing    
    # i = 0
    # while i < len(problem.currentPath[4]):
    #     print("leave ", problem.currentPath[3][i], " and go ", problem.currentPath[4][i])
    #     i += 1
    return problem.currentPath[4]

    
    
#Wen Lee 2/18/22 
def uniformCostSearch(problem):
    """Search the node of least total cost first.
    
    By changing the cost function, we can make Pacman find different paths 
    """

    #start Node 
    nodeZero = problem.getStartState()
    visitedNodes = []

    pq = util.PriorityQueue()
    goalHasBeenFound = False 
    goal = nodeZero
    
    #We are pushing the very first state onto the priorityQueue so we have something 
    #to start with
        #push(self, item(state, action, priority), priority)
    pq.push((nodeZero, [], 0), 0)

    while not pq.isEmpty():

        #pattern match the node popped from the queue
        currentNode, action, stepCost = pq.pop()
        if currentNode not in visitedNodes:
            visitedNodes.append(currentNode)

            if problem.isGoalState(currentNode):
                #goalHasBeenFound = True 
                #goal = currentNode
                return action
                    #returning the action gets rid of the long if-else block for when the goalHasBeenFound turns out to be true 
                    #I'm not sure if we really need to add goalHasBeenFound or goal to this part of the code anymore. 
            
            #pattern matching within for block 
            for successorNodes, actions, nextCosts in problem.getSuccessors(currentNode):
                newAction = action + [actions]
                priority = stepCost + nextCosts
                pq.push((successorNodes, newAction, priority),priority)
    util.raiseNotDefined()

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0
#jackson 2-21-2022 (feel kinda bad cause I just straight up booted the code, I literally had to change one line)
def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the autograder.py lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    nodeZero = problem.getStartState()
    problem.visitedNodes = []
    problem.optimalNode = []

    pq = util.PriorityQueue()
    goalHasBeenFound = False 
    goal = nodeZero
    
    #We are pushing the very first state onto the priorityQueue so we have something 
    #to start with
        #push(self, item(state, action, priority), priority)
    pq.push((nodeZero, [], 0), 0)

    while not pq.isEmpty():

        #pattern match the node popped from the queue
        currentNode, action, stepCost = pq.pop()
        problem.currentNode = currentNode

        if currentNode not in problem.visitedNodes:
            problem.visitedNodes.append(currentNode)

            #print('problem isGoalState?:' + str(problem.isGoalState(currentNode)))
            if problem.isGoalState(currentNode):
                #goalHasBeenFound = True 
                #goal = currentNode
                return action
                    #returning the action gets rid of the long if-else block for when the goalHasBeenFound turns out to be true 
                    #I'm not sure if we really need to add goalHasBeenFound or goal to this part of the code anymore. 
            
            #pattern matching within for block 

            for successorNodes, actions, nextCosts in problem.getSuccessors(currentNode):
                newAction = action + [actions]
                #modified priority for how A* works, think this is correct
                priority = stepCost + nextCosts 
                hur = heuristic(successorNodes,problem)
                pq.push((successorNodes, newAction, priority),priority + hur)

    # util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
