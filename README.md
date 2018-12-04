# Maze_TSP


Abdelrahman Yaseen

This code is an implementation of A* algorithm to find the shortest path 
between each pair of points in a given maze as a first step.
Then it solves the TSP problem between these points, using both BFS and UFS, and compares their performance. 
Please check the assignment's document for further details. 

DESCRIPTION:

      The code folder has 7 files:
            4 python files :
                  Main.py     > Maintains the flow of the program to produce the intended output.

                  Map.py      > Has classes responsible about reading the map file, and organizing    
                                the points, utility functions like calculating the Manhatten distance, etc.

                  Agent_TSP   > Has the classes responsible about solving the TSP problem. classes to maintain
                                the States, Nodes, Agent, Solution .. etc 
                  Agent_Maze  > Similar to Agent_TSP, yet modified to solve the shortest path between 1 points in the maze 
                  
                  [The Agent Classes are done with the use of the suggested libraries in the assignments' document]
                  http://aima.cs.berkeley.edu/code.html	

            4 text files :
                  Map1.txt    > The map provided in the beginning of the assignment's document
                  Map2.txt    > The map provided in the end of the assignment's document
                  Map3.txt    > A big additional map (15x15)
                  Map4.txt    > A big additional map (23x16)

HOW TO COMPILE:

      All what you need to do is to run Main.py, enter the name of the file that contains the map.
      Choose if you want an animation
      and then run.

A brief explaination of most of the functions is embedded throughout the code


      
