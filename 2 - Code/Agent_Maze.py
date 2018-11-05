import heapq
from Map import Point


class Solution:
    def __init__(self,start,steps):
        self.start=start
        self.length=len(list(steps))
        steps.reverse()
        self.steps=steps


class State_Maze :
    def __init__(self,map,point):
        self.map=map
        self.point=point
    def __str__(self):
        # return "State :" + str(self.point)
        # print("{}\n".format(str(self.point)))
        # return ""
        return ".{}.".format(str(self.point))
    def __eq__(self,obj):
        return self.point.x==obj.point.x and self.point.y==obj.point.y and obj.point.val==self.point.val
class Problem_Maze:

    """The abstract class for a formal problem."""

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
        if state.map.getAdjacent_UP(state.point).val!=state.map.BLOCK :
            actions.append("UP")
        if state.map.getAdjacent_DOWN(state.point).val!=state.map.BLOCK :
            actions.append("DOWN")
        if state.map.getAdjacent_RIGHT(state.point).val!=state.map.BLOCK :
            actions.append("RIGHT")
        if state.map.getAdjacent_LEFT(state.point).val!=state.map.BLOCK :
            actions.append("LEFT")
        # print("ACTIONS : ", actions)
        return actions


    def result(self, state, action):
        """Return the state that results from executing the given
        action in the given state. The action must be one of
        self.actions(state)."""
        # print("IN RESULS :  STATE:{} > ACTION :{}".format(state,action))
        newState = State_Maze(state.map,None)
        if action == "UP" :
            newState.point=state.map.getAdjacent_UP(state.point)
        if action == "DOWN" :
            newState.point=state.map.getAdjacent_DOWN(state.point)
        if action == "RIGHT" :
            newState.point=state.map.getAdjacent_RIGHT(state.point)
        if action == "LEFT" :
            newState.point=state.map.getAdjacent_LEFT(state.point)
        # print("IN RESULS << STATE:",state)
        return newState

    def goal_test(self, state):
        """Return True if the state is a goal. The default method compares the
        state to self.goal or checks for state in self.goal if it is a
        list, as specified in the constructor. Override this method if
        checking against a single self.goal is not enough."""
        # if isinstance(self.goal, list):
        #     return is_in(state, self.goal)
        # else:
        return state == self.goal

    def path_cost(self, c, state1, action, state2):
        """Return the cost of a solution path that arrives at state2 from
        state1 via action, assuming cost c to get up to state1. If the problem
        is such that the path doesn't matter, this function will only look at
        state2.  If the path does matter, it will consider c and maybe state1
        and action. The default method costs 1 for every step in the path."""
        return c + 1





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
        # raise NotImplementedError
        return self.path_cost < node.path_cost

    def expand(self, problem):
        """List the nodes reachable in one step from this node."""

        #Tirebreaking : Put nodes always in order. (The search algorithm takes care of the rest of the problem)
        #This step is significant in the BFS

        result =  [self.child_node(problem, action)for action in problem.actions(self.state)]
        orderedResult = []
        for n in result:
            for i in range(self.state.map.length):
                for j in range(self.state.map.width):
                    if n.state.point.x==j and  n.state.point.y == i:
                        orderedResult.append(n)

        return orderedResult

    def child_node(self, problem, action):
        """[Figure 3.10]"""
        # print(" IN CHILD_NODE << problem state",self.state)

        next_state = problem.result(self.state, action)
        next_node = Node(next_state, self, action,
                    problem.path_cost(self.path_cost, self.state,
                                      action, next_state))
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

    # We want for a queue of nodes in breadth_first_graph_search or
    # astar_search to have no duplicated states, so we treat nodes
    # with the same state as equal. [Problem: this may not be what you
    # want in other contexts.]

    def __eq__(self, other):
        return isinstance(other, Node) and self.state == other.state

    def __hash__(self):
        return hash(self.state)

    def __str__(self):
        if(self.parent != None):
            return "Node : (State : {}\nParent : {}\nAction : {}\npath_cost: {}\nDepth:{})".format(self.state,self.parent.state,self.action,self.path_cost,self.depth)
        else :
            return "Node : (State : {}\nParent : {}\nAction : {}\npath_cost: {}\nDepth:{})".format(self.state,"no parent",self.action,self.path_cost,self.depth)

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
        elif order == 'max':  # now item with max f(x)
            self.f = lambda x: -f(x)  # will be popped first
        else:
            raise ValueError("order must be either 'min' or max'.")

    def append(self, item):
        """Insert item at its correct position."""
        # print("INSIDE PQ :",type(item))
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

def best_first_graph_search(problem,ShowAnimation):
    """Search the nodes with the lowest f scores first.
    You specify the function f(node) that you want to minimize; for example,
    if f is a heuristic estimate to the goal, then we have greedy best
    first search; if f is node.depth then we have breadth-first search.
    There is a subtlety: the line "f = memoize(f, 'f')" means that the f
    values will be cached on the nodes as they are computed. So after doing
    a best first search you can examine the f values of the path returned."""
    f = lambda node : node.path_cost + node.state.map.getManhattenDistance(node.state.point,problem.goal.point)
    node = Node(problem.initial)

    frontier = PriorityQueue('min', f)
    frontier.append(node)
    explored = []
    I=0
    while frontier:
        # print("\n ***************{}*************\n".format(I))
        node = frontier.pop()
        # print("{} POPPED\n".format(node))
        # print([str(x[1]) for x in frontier.heap])
        if problem.goal_test(node.state):
            return node
        # print(">>",node.action)

        if(ShowAnimation):
            import os
            os.system('cls' if os.name == 'nt' else 'clear')
            if isinstance(node.state.map.array[node.state.point.y][node.state.point.x].val, int):
                node.state.map.array[node.state.point.y][node.state.point.x].val=ord(problem.initial.point.val)+ord(problem.goal.point.val)+30
            print(node.state.map)
            import time
            time.sleep(0.03)
        explored.append(node.state)
        # print(node.state)
        # print("Explored:")
        # print([str(x) for x in explored])
        # print("end Explored")

        # print("\n*{\n"+str(list(map(lambda x : str(x),explored)))+"\n}*\n")
        for child in node.expand(problem):

            #debugging print
            # print("for {} >> {} ".format(child.state,child.state in explored))
            if (child.state not in explored) and (child not in frontier):
                # print("{}appeneded".format(child.state))
                frontier.append(child)
            elif child in frontier:
                # print("child in frontier")
                incumbent = frontier[child]
                if f(child) < f(incumbent):
                    del frontier[incumbent]
                    frontier.append(child)


        #debugging print
        # print("Frontier:")
        # print([str(x[1]) for x in frontier.heap])
        # print("End Frontier")
        # I+=1
        # if(I==100):
        #     break

    return None


class SimpleProblemSolvingAgentProgram_Maze:

    """Abstract framework for a problem-solving agent. [Figure 3.1]"""

    def __init__(self, initial_state=None,ShowAnimation=False):
        """State is an abstract representation of the state
        of the world, and seq is the list of actions required
        to get to a particular state from the initial state(root)."""
        self.state = initial_state
        self.seq = []
        self.ShowAnimation=ShowAnimation

    # def __call__(self, percept):
    def __call__(self, initial_state, goal_state,ShowAnimation):
        """[Figure 3.1] Formulate a goal and problem, then
        search for a sequence of actions to solve it."""
        # self.state = self.update_state(self.state, percept)
        self.state = initial_state
        self.ShowAnimation=ShowAnimation
        if not self.seq:
            # goal = self.formulate_goal(self.state)
            goal = goal_state
            problem = self.formulate_problem(self.state, goal)
            self.seq = self.search(problem)
            if not self.seq:
                return None
            steps=[]
            dummy = self.seq
            while dummy :
                # print("{} - {} ".format(dummy.action,dummy.path_cost))
                if dummy.action:
                    steps.append(dummy.action)
                dummy=dummy.parent

            return Solution(initial_state.point,steps)
        # return self.seq.pop(0)

    # def update_state(self, state, percept):
    #     raise NotImplementedError

    # def formulate_goal(self, state):
    #     raise NotImplementedError

    def formulate_problem(self, state, goal):
        return Problem_Maze(state,goal)


    def search(self, problem):
        # return breadth_first_tree_search(problem)
        return best_first_graph_search(problem,self.ShowAnimation)
