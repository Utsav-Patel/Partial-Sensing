from constants import NUM_ROWS, NUM_COLS, STARTING_POSITION_OF_AGENT
from src.Cell import Cell
from src.helper import astar_search
from abc import ABC, abstractmethod
import numpy as np


class Agent(ABC):
    def __init__(self, algorithm: str = 'astar'):
        self.maze = list()
        for row in range(NUM_ROWS):
            self.maze.append(list())
            for col in range(NUM_COLS):
                self.maze[row].append(Cell())
        self.current_position = STARTING_POSITION_OF_AGENT
        self.algorithm = algorithm
        self.num_cells_explored_while_planning = 0
        self.final_paths = list()
        self.num_backtracks = 0
        self.parents = dict()

    def planning(self):
        # Choose which algorithm you want to use for search
        if self.algorithm == 'astar':
            self.parents, num_explored_nodes = astar_search(self.maze, self.current_position)
            self.num_cells_explored_while_planning += num_explored_nodes
        # elif self.algorithm == 'bfs':
        #     parents, num_explored_nodes = bfs_search(maze, start_pos, goal_pos)
        else:
            raise Exception("algorithm should be either astar or bfs")

    @abstractmethod
    def execution(self, full_maze: np.array):
        pass
