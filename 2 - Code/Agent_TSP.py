from collections import defaultdict, deque
import heapq

def breadth_first_tree_search(problem,goalCost):
    """Search the shallowest nodes in the search tree first.
    Search through the successors of a problem to find a goal.
    The argument frontier should be an empty queue.
    Repeats infinitely in case of loops. """
    numberOfNodes=0
    frontier = deque([Node(problem.initial)])

    while frontier:
        node = frontier.popleft()
        numberOfNodes+=1
        if problem.goal_test(node.state,goalCost):
            return [node.state,numberOfNodes]
        frontier.extend(node.expand(problem))

    return None

class PriorityQueue:
    """A Queue in which the minimum (or maximum) element (as determined by f and
    order) is returned first.
    If order is 'min', the item with minimum f(x) is
    returned first; if order is 'max', then it is the item with maximum f(x).
    Also supports dict-like lookup."""

    def __init__(self, order='min', f=lambda x: x):
        self.heap = []

        if order == 'min':
            self.f = f
        else:
            raise ValueError("order must be either 'min' or max'.")

    def append(self, item):
        """Insert item at its correct position."""
        heapq.heappush(self.heap, (self.f(item), item))

    def extend(self, items):
        """Insert each item in items at its correct position."""
        for item in items:
            self.append(item)

    def pop(self):
        """Pop and return the item (with min or max f(x) value
        depending on the order."""
        if self.heap:
            return heapq.heappop(self.heap)[1]
        else:
            raise Exception('Trying to pop from empty PriorityQueue.')

    def __len__(self):
        """Return current capacity of PriorityQueue."""
        return len(self.heap)

    def __contains__(self, item):
        """Return True if item in PriorityQueue."""
        return (self.f(item), item) in self.heap

    def __getitem__(self, key):
        for _, item in self.heap:
            if item == key:
                return item

    def __delitem__(self, key):
        """Delete the first occurrence of key."""
        self.heap.remove((self.f(key), key))
        heapq.heapify(self.heap)
    def __str__(self):
        return str(self.heap)


def best_first_graph_search(problem):
    """Search the nodes with the lowest f scores first.
    You specify the function f(node) that you want to minimize; for example,
    if f is a heuristic estimate to the goal, then we have greedy best
    first search; if f is node.depth then we have breadth-first search.
    There is a subtlety: the line "f = memoize(f, 'f')" means that the f
    values will be cached on the nodes as they are computed. So after doing
    a best first search you can examine the f values of the path returned."""
    numberOfNodes=0
    #Since it's UCS
    f = lambda node: node.path_cost
    node = Node(problem.initial)
    frontier = PriorityQueue('min', f)
    frontier.append(node)
    explored = set()
    while frontier:
        node = frontier.pop()
        numberOfNodes+=1
        if problem.goal_test(node.state,200):
            return  [node.state,numberOfNodes]
        explored.add(node.state)
        for child in node.expand(problem):
            if child.state not in explored and child not in frontier:
                frontier.append(child)
            elif child in frontier:
                incumbent = frontier[child]
                if f(child) < f(incumbent):
                    del frontier[incumbent]
                    frontier.append(child)
    return None


class Solution:
    def __init__(self,steps,maxNumOfNodes,time,cost):
        self.steps=steps
        self.maxNumOfNodes=maxNumOfNodes
        self.time=time
        self.cost=cost

def createAdjMatrix(V, G):

  adjMatrix = []

  for i in range(0, V):
    adjMatrix.append([])
    for j in range(0, V):
      adjMatrix[i].append(0)

  for i in range(0, len(G)):
    adjMatrix[G[i][0]][G[i][1]] = G[i][2]
    adjMatrix[G[i][1]][G[i][0]] = G[i][2]

  return adjMatrix
def prims(V, G):

  sum=0
  adjMatrix = createAdjMatrix(V, G)

  vertex = 0

  MST = []
  edges = []
  visited = []
  minEdge = [None,None,float('inf')]

  while len(MST) != V-1:

    visited.append(vertex)

    for r in range(0, V):
      if adjMatrix[vertex][r] != 0:
        edges.append([vertex,r,adjMatrix[vertex][r]])

    for e in range(0, len(edges)):
      if edges[e][2] < minEdge[2] and edges[e][1] not in visited:
        minEdge = edges[e]

    edges.remove(minEdge)
    sum+=minEdge[2]
    MST.append(minEdge)

    vertex = minEdge[1]
    minEdge = [None,None,float('inf')]

  return sum


class State_TSP :
    def __init__(self,adjMatrix,visited,unvisited):
        self.adjMatrix=adjMatrix
        self.visited=[v for v in visited]
        self.unvisited=[v for v in unvisited]
    def __str__(self):
        return ".{} - {}.".format(self.visited,self.unvisited)
    def __eq__(self,obj):
        for i in range(len(self.visited)):
            if self.visited[i] != obj.visited[i]:
                return False
        return True
    def __hash__(self):
        return hash(id(self))

class Problem_TSP:

    """The abstract class for a formal problem. You should subclass
    this and implement the methods actions and result, and possibly
    __init__, goal_test, and path_cost. Then you will create instances
    of your subclass and solve them with the various search functions."""

    def __init__(self, initial, goal=None):
        """The constructor specifies the initial state, and possibly a goal
        state, if there is a unique goal. Your subclass's constructor can add
        other arguments."""
        self.initial = initial
        self.goal = goal

    def actions(self, state):
        """Return the actions that can be executed in the given
        state. The result would typically be a list, but if there are
        many actions, consider yielding them one at a time in an
        iterator, rather than building them all at once."""
        actions=[]
        for v in state.unvisited :
            actions.append(v)
        if len(actions) == 0 and len(state.visited) < len(state.adjMatrix)+1:
            actions.append(0)
        return actions


    def result(self, state, action):
        """Return the state that results from executing the given
        action in the given state. The action must be one of
        self.actions(state)."""
        newState = State_TSP(state.adjMatrix,state.visited,state.unvisited)
        if not (newState.unvisited == [] and action == 0 ):
            newState.unvisited.remove(action)
        newState.visited.append(action)
        return newState

    def goal_test(self, state, goalCost):
        """Return True if the state is a goal. The default method compares the
        state to self.goal or checks for state in self.goal if it is a
        list, as specified in the constructor. Override this method if
        checking against a single self.goal is not enough."""
        return len(state.visited)==len(state.adjMatrix)+1 and computeCost(state.adjMatrix,state.visited)<goalCost

    def path_cost(self, c, state1, action, state2):
        """Return the cost of a solution path that arrives at state2 from
        state1 via action, assuming cost c to get up to state1. If the problem
        is such that the path doesn't matter, this function will only look at
        state2.  If the path does matter, it will consider c and maybe state1
        and action. The default method costs 1 for every step in the path."""
        return state1.adjMatrix[state1.visited[len(state1.visited)-1]][state2.visited[len(state2.visited)-1]]





class Node:

    """A node in a search tree. Contains a pointer to the parent (the node
    that this is a successor of) and to the actual state for this node. Note
    that if a state is arrived at by two paths, then there are two nodes with
    the same state.  Also includes the action that got us to this state, and
    the total path_cost (also known as g) to reach the node.  Other functions
    may add an f and h value; see best_first_graph_search and astar_search for
    an explanation of how the f and h values are handled. You will not need to
    subclass this class."""

    def __init__(self, state, parent=None, action=None, path_cost=0):
        """Create a search tree Node, derived from a parent by an action."""
        self.state = state
        self.parent = parent
        self.action = action
        self.path_cost = path_cost
        self.depth = 0
        if parent:
            self.depth = parent.depth + 1

    def __repr__(self):
        return "<Node {}>".format(self.state)

    def __lt__(self, node):
        return self.path_cost < node.path_cost

    def expand(self, problem):
        """List the nodes reachable in one step from this node."""

        return [self.child_node(problem, action)
                for action in problem.actions(self.state)]

    def child_node(self, problem, action):
        """]"""
        next_state = problem.result(self.state, action)
        path_cost=problem.path_cost(self.path_cost, self.state,action, next_state)
        path_cost+=self.path_cost
        next_node = Node(next_state, self, action,path_cost)

        return next_node

    def solution(self):
        """Return the sequence of actions to go from the root to this node."""
        return [node.action for node in self.path()[1:]]

    def path(self):
        """Return a list of nodes forming the path from the root to this node."""
        node, path_back = self, []
        while node:
            path_back.append(node)
            node = node.parent
        return list(reversed(path_back))


    def __eq__(self, other):
        return isinstance(other, Node) and self.state == other.state

    def __hash__(self):
        return hash(self.state)

    def __str__(self):
        if(self.parent != None):
            return "Node : (State : {}\nParent : {}\nAction : {}\npath_cost: {}\nDepth:{})".format(self.state,self.parent.state,self.action,self.path_cost,self.depth)
        else :
            return "Node : (State : {}\nParent : {}\nAction : {}\npath_cost: {}\nDepth:{})".format(self.state,"no parent",self.action,self.path_cost,self.depth)

class SimpleProblemSolvingAgentProgram_TSP:

    """Abstract framework for a problem-solving agent. """

    def __init__(self, initial_state=None):
        """State is an abstract representation of the state
        of the world, and seq is the list of actions required
        to get to a particular state from the initial state(root)."""
        self.state = initial_state
        self.seq = []

    def __call__(self, initial_state, goal_state,algorithm=1):
        """ Formulate a goal and problem, then
        search for a sequence of actions to solve it."""
        self.state = initial_state
        if not self.seq:
            goal = goal_state
            problem = self.formulate_problem(self.state, goal)
            self.seq = self.search(problem,algorithm)
            if not self.seq:
                return None

            return self.seq





    def formulate_problem(self, state, goal):
        return Problem_TSP(state,goal)


    def search(self, problem,algorithm=1):

        import time
        start_time = time.time()

        solution = None
        goalCost = 0
        if(algorithm==1):
            while not solution :
                goalCost+=1
                solution = breadth_first_tree_search(problem,goalCost)
        else:
            while not solution :
                goalCost+=1
                #Uniform Cost Search
                solution = best_first_graph_search(problem)


        executionTime = time.time() - start_time
        cost=computeCost(self.state.adjMatrix,solution[0].visited)

        numberOfNodes= solution[1]
        return Solution(solution[0].visited,numberOfNodes,executionTime,cost)


def computeCost(adjMatrix,path):
    sum =0
    for i in range(1,len(path)):
        sum+=adjMatrix[path[i-1]][path[i]]
    return sum
