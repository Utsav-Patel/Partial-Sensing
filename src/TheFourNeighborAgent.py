# Necessary imports
import numpy as np

from src.Agent import Agent
from src.helper import forward_execution


# Four Neighbor Agent class
class TheFourNeighborAgent(Agent):
    def __init__(self):
        super().__init__()

    # Overridden execution method
    def execution(self, full_maze: np.array):
        self.num_astar_calls += 1
        self.num_bumps += 1
        current_path, num_backtracks = forward_execution(self.maze, full_maze, self.current_position, self.parents,
                                                         want_to_explore_field_of_view=True)
        self.current_position = current_path[-1]
        self.final_paths.append(current_path)
        self.num_backtracks += num_backtracks
