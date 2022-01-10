# Partial-Sensing
This is the second project of the course, Introduction to Artificial Intelligence at Rutgers, the State University of New Jersey, New Brunswick.

In this project, we used python version 3.8.11 and conda version 4.8.3.

The assignment file contains the problem description and the report file contains the generated results. Also, we have added one file named "Description of Project Structure.txt" where you can get how the project is organised.

For this project we had to create five different agents that could all clear randomly generated mazes and then compare their performance.

The details of these five agents is given in the [Assignment here ](https://github.com/Utsav-Patel/Partial-Sensing/blob/master/Assignment%202.pdf)

##### Number of confirmed cells

We first compared how many cells each of the agents could confirm as blocked or unblocked. We get results as follows - 

<p align="center">
	<img src="/images/confirmed_cells.png">
</p>

We can see that our own made agent, Agent five needs to confirm the least amount of cells before successfully completing the maze. 

##### Number of A* calls

We compared the number of times each agent had to run A star algorithm to reach the goal. This directly correlates with time taken to reach goal and is a valuable metric for us.
We got results as follows -

<p align="center">
	<img src="/images/num_astar_calls.png">
</p>

We see that Agent five takes the most amount of time followed by the other agents.


##### Shortest Path in Final discovered grid world / Shortest path in Full grid world.

Here we define disovered grid world as the part of the grid that the agent itself explored while moving to the goal and the full grid world as the full existing grid world.
The above ratio gives us a good idea as to how efficient the final discovered path was in comparison to the actual shortest path. A ratio closer to one is desirable. We get
results as follows -

<p align="center">
	<img src="/images/shortest_path_in_final_discovered_by_full_grid.png">
</p>


In conclusion we can see that Agent five performs the best and the blindfolded agent performs the worst. For a more in-depth explanation of our results please refer to
[our report here](https://github.com/Utsav-Patel/Partial-Sensing/blob/master/Report.pdf)