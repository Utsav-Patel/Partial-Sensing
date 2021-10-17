
# Necessary imports
import numpy as np

from constants import GOAL_POSITION_OF_AGENT
from src.Agent import Agent
from src.helper import parent_to_child_dict, sense_current_node, find_block_while_inference


# Example inference class
class TheExampleInferenceAgent(Agent):
    def __init__(self):
        super().__init__()

    # Overridden execution method
    def execution(self, full_maze: np.array):

        # Increase counter of A* calls
        self.num_astar_calls += 1
        children = parent_to_child_dict(self.parents, GOAL_POSITION_OF_AGENT)
        current_position = self.current_position

        # Add all trajectory cells to one set
        entire_trajectory_nodes = set()
        entire_trajectory_nodes.add(current_position)

        while current_position != children[current_position]:
            current_position = children[current_position]
            entire_trajectory_nodes.add(current_position)

        current_position = self.current_position
        current_path = list()
        current_path.append(current_position)

        # Run this loop until we will reach at goal state or block cell
        while True:

            # Mark visited to unvisited cell and sense it
            if not self.maze[current_position[0]][current_position[1]].is_visited:
                self.maze[current_position[0]][current_position[1]].is_visited = True
                sense_current_node(self.maze, current_position, full_maze)

            # Check whether you can infer anything from the current node
            if find_block_while_inference(self.maze, current_position, full_maze, entire_trajectory_nodes):
                self.num_early_termination += 1
                break

            # Check whether the next node is block or not.
            if full_maze[children[current_position][0]][children[current_position][1]] == 1:
                find_block_while_inference(self.maze, children[current_position], full_maze)
                self.num_bumps += 1
                break
            else:
                if current_position == children[current_position]:
                    break
                current_position = children[current_position]
                current_path.append(current_position)

        self.final_paths.append(current_path)
        self.current_position = current_path[-1]
