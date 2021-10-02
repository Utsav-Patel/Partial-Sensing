import numpy as np

from src.Agent import Agent
from src.helper import forward_execution


class TheBlindfoldedAgent(Agent):
    def __init__(self):
        super().__init__()

    def execution(self, full_maze: np.array):
        current_path, num_backtracks = forward_execution(self.maze, full_maze, self.current_position, self.parents,
                                                         want_to_explore_field_of_view=False)[:2]
        self.current_position = current_path[-1]
        self.final_paths.append(current_path)
        self.num_backtracks += num_backtracks
