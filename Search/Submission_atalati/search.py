# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for 
# educational purposes provided that (1) you do not distribute or publish 
# solutions, (2) you retain this notice, and (3) you provide clear 
# attribution to UC Berkeley, including a link to 
# http://inst.eecs.berkeley.edu/~cs188/pacman/pacman.html
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero 
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and 
# Pieter Abbeel (pabbeel@cs.berkeley.edu).

# Project 1 - CS 3600 (SUMMER 2017)
# By Aatmay S. Talati
# Due Date : SUNDAY (JUNE 4th, 2017)


"""
In search.py, you will implement generic search algorithms which are called
by Pacman agents (in searchAgents.py).
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
        Returns the start state for the search problem
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples,
        (successor, action, stepCost), where 'successor' is a
        successor to the current state, 'action' is the action
        required to get there, and 'stepCost' is the incremental
        cost of expanding to that successor
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.  The sequence must
        be composed of legal moves
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other
    maze, the sequence of moves will be incorrect, so only use this for tinyMaze
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s,s,w,s,w,w,s,w]

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first

    Your search algorithm needs to return a list of actions that reaches
    the goal.  Make sure to implement a graph search algorithm

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    """
    "*** YOUR CODE HERE ***"

    '''
        My approch for search Algorithems....

        1) Initialize Queue / Stack  #DFS has Stack and BFS has Queue
        2) Push start
        3) Then Check for neighbour nodes
        4) if there is a neighbour node then try going it via alphabatical order
        5) Put the visited node with neighbours to stack
    '''

    #initiate queue/Stack
    visited_node = []
    dfsStack = util.Stack()

    #push start
        #start the node. (First Node), and Cost/Edge Value (but there's no cost at any edge)
    dfsStack.push((problem.getStartState(), []))
    
    #Travel in a graph

    #Check if the nodes are there or not?
    while not dfsStack.isEmpty():
        vertex, path = dfsStack.pop() # getting current node and the path of the first path
            # Stack/Queue.pop() method gets the first node and 
        if vertex not in visited_node: # Checking for Vertex if its in visited node stack or not
            visited_node.append(vertex) # putting in stack of visited Noddes.

        #if the current node is your goal then you
            if problem.isGoalState(vertex):
                return path

        #otherwise continue
             #checking vertex connections/brahces # Returns the lists
            CurrentNode_successor = problem.getSuccessors(vertex)
            for successor, action, step_cost in CurrentNode_successor:
                dfsStack.push((successor, path + [action]))
    return []

    #util.raiseNotDefined()

def breadthFirstSearch(problem):
    """
    Search the shallowest nodes in the search tree first.
    """
    "*** YOUR CODE HERE ***"

    #inilize the Queue
    BFSVisited_node = [] #Empty list of Visited nodes. 
    bfsQueue = util.Queue() #got from the file util

    #push start
        #first the node, and thn we care about the cost associated with that edge.
        #However, the the case of BFS/DFS we don't have to worry about cost. So, we will keep that variable blank.
    bfsQueue.push((problem.getStartState(), []))

    #Travel in the graph
    while not bfsQueue.isEmpty(): 
        current_node, tNode = bfsQueue.pop() 
        if current_node not in BFSVisited_node:
            BFSVisited_node.append(current_node)

        #Reaching the Goal
        #if the current node is the soluation then just return that perticular node as soluation. - AT
            if problem.isGoalState(current_node):
                return tNode

        #visit nodes which is not visited.
            CurrentNode_successor = problem.getSuccessors(current_node)
            for successor, action, step_cost in CurrentNode_successor:
                #if successor not in BFSVisited_node:
                    bfsQueue.push((successor, tNode + [action]))
    return []

  # util.raiseNotDefined()


def uniformCostSearch(problem):
    "Search the node of least total cos3t first. "
    "*** YOUR CODE HERE ***"
    visited_node = []
    UCS_pq = util.PriorityQueue()

    #push start
    #UCS_pq.push((problem.getStartState(), []), nullHeuristic(Vertex, problem)) #start the node. (First Node), and Cost/Edge Value (but there's no value associated with that edge
    UCS_pq.push((problem.getStartState(), []), 0)
    #Travel in a graph

    #Check if the nodes are there or not?
    while not UCS_pq.isEmpty():
        vertex, path = UCS_pq.pop() #getting current node and the path of the first path
        if vertex not in visited_node: #Checking for Vertex if its in visited node stack or not
            visited_node.append(vertex) # putting in stack of visited Noddes.

        #if the current node is your goal then you
            if problem.isGoalState(vertex):
                return path

        #otherwise continue
            CurrentNode_successor = problem.getSuccessors(vertex) #checking vertex connections/brahces # Returns the lists
            for successor, action, step_cost in CurrentNode_successor:
                UCS_pq.push((successor, path + [action]), problem.getCostOfActions(path+[action]) + nullHeuristic(successor,problem))
    return []


    UCS_visitedNode = []



    #util.raiseNotDefined()

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    "Search the node that has the lowest combined cost and heuristic first."
    "*** YOUR CODE HERE ***"
    visited_node = []
    UCS_pq = util.PriorityQueue()

    #push start
    #UCS_pq.push((problem.getStartState(), []), nullHeuristic(Vertex, problem)) #start the node. (First Node), and Cost/Edge Value (but there's no value associated with that edge
    UCS_pq.push((problem.getStartState(), []), heuristic(problem.getStartState(), problem))
    #Travel in a graph

    #Check if the nodes are there or not?
    while not UCS_pq.isEmpty():
        vertex, path = UCS_pq.pop() #getting current node and the path of the first path
        if vertex not in visited_node: #Checking for Vertex if its in visited node stack or not
            visited_node.append(vertex) # putting in stack of visited Noddes.

        #if the current node is your goal then you
            if problem.isGoalState(vertex):
                return path

        #otherwise continue
            CurrentNode_successor = problem.getSuccessors(vertex) #checking vertex connections/brahces # Returns the lists
            for successor, action, step_cost in CurrentNode_successor:
                UCS_pq.push((successor, path + [action]), problem.getCostOfActions(path+[action]) + heuristic(successor,problem))
    return []

   #util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
