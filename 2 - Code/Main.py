"""
The Agent_Maze uses the well-known A* Algorithm with Heuristic of Manhantten distanceself.
Refer to Agent_Maze.py to see the details of the code itself.
the key point is :
{f = lambda node : node.path_cost + node.state.map.getManhattenDistance(node.state.point,problem.goal.point)}


The Agent_TSP uses BFS, and UFS to find the shortest TSP loop.
Since it's an uninformed search, and we don't know the cost of the minimal path,
I tried to repeat the search to find the path with cost == C [ c equals to 1 initially ]
if a path is found, it's returned.
otherwize, C is incremented, the search is repeated.

Based on the statistics, UFS performs significantly better than BFS.
"""



MapFileName = "map3.txt"
ShowAnimation = True

BLOCK_NODE = 0
SPACE_NODE = 1
from Agent_Maze import State_Maze
from Map import Map
from Map import Point
from Agent_Maze import SimpleProblemSolvingAgentProgram_Maze
from Agent_TSP import SimpleProblemSolvingAgentProgram_TSP
from Agent_TSP import State_TSP
from Agent_TSP import createAdjMatrix

map = Map.fromFile(MapFileName)


print("{}x{}".format(map.width,map.length))
print(map)

checkpoints = map.getCheckpointPairs()
checkPointsSymbols = []


#Getting the symboles as they are in the maze (not the most efficient way)
for i in [str(v[0].val) for v in checkpoints] :
    if i not in checkPointsSymbols:
        checkPointsSymbols.append(i)

for i in [str(v[1].val) for v in checkpoints] :
    if i not in checkPointsSymbols:
        checkPointsSymbols.append(i)


# from Agent_Maze import Problem_Maze
# initial_state = State_Maze(map,checkpoints[0][0])
# goal_state = State_Maze(map,checkpoints[0][1])
# p = Problem_Maze(initial_state,goal_state)
# print(map.getAdjacent_LEFT(goal_state.point))
# print(p.actions(goal_state))
# #s = SimpleProblemSolvingAgentProgram_Maze()(initial_state,goal_state)
# exit()
#solving the problem of finding the shortest path between each pair of checkpoints
graph=[]
print(checkPointsSymbols)
line=""
for i in range(len(checkpoints)):
    s=None
    initial_state = State_Maze(map,checkpoints[i][0])
    goal_state = State_Maze(map,checkpoints[i][1])
    s = SimpleProblemSolvingAgentProgram_Maze()(initial_state,goal_state,ShowAnimation)
#    if(s):
    line+="{},{},{}\n".format(initial_state.point.val,goal_state.point.val,s.length)
    graph.append([checkPointsSymbols.index(initial_state.point.val),checkPointsSymbols.index(goal_state.point.val),s.length])



if(ShowAnimation):
    map = Map.fromFile(MapFileName)
    print("{}x{}".format(map.width,map.length))
    print(map)
print(line)
print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")


"""FIRST PART IS Over

MOVING TO THE SECOND PART

to find an "optimal" solution
We need to define the goal
the goal is to pass through all the nodes with a minimal cost (C)
But we dont know the minmal cost.
I'll iterate through a minimal cose = 1 until a solution is found.
Keeping in mind that a loop that passes through all the nodes is guaranteed
in a fully connected graph.

set goal = {
    a state of visited list of length = N + 1
    total cost =  i
}
look for the goal using BFS/UFS, if not found, increase i.

 """


#setting the states for the second problem, TSP

am=createAdjMatrix(len(checkPointsSymbols), graph)
initial_state = State_TSP(am,[0],list(range(1,len(am))))
goal_state= State_TSP(am,list(range(len(am))),[])
#the goal_state is not necessary, the goal_test formulation in Agent_TSP has an independent definition.


print("Algorithm used : BFS")
solution_BFS = SimpleProblemSolvingAgentProgram_TSP()(initial_state,goal_state,1)
print(*[checkPointsSymbols[i] for i in solution_BFS.steps],sep="-")
print("Total Tour Cost : ",solution_BFS.cost)

print("\n\nAlgorithm used : UCS")
solution_UFS = SimpleProblemSolvingAgentProgram_TSP()(initial_state,goal_state,2)
print(*[checkPointsSymbols[i] for i in solution_UFS.steps],sep="-")
print("Total Tour Cost : ",solution_UFS.cost)

print("Statistics : ")
print("     Nodes       Time        Cost")
print("BFS  {}          {}          {}".format(solution_BFS.maxNumOfNodes,round(solution_BFS.time,4),solution_BFS.cost))
print("UCS  {}          {}          {}".format(solution_UFS.maxNumOfNodes,round(solution_UFS.time,4),solution_UFS.cost))
