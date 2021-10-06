from constants import NUM_ROWS, NUM_COLS, STARTING_POSITION_OF_AGENT
from src.Cell import Cell
from src.helper import astar_search, compute_heuristics, manhattan_distance
from abc import ABC, abstractmethod
import numpy as np


class Agent(ABC):
    def __init__(self, algorithm: str = 'astar', heuristic_function: str = 'manhattan'):
        self.maze = list()

        for row in range(NUM_ROWS):
            cell_list = list()
            for col in range(NUM_COLS):
                cell_list.append(Cell())
            self.maze.append(cell_list)

        if heuristic_function == 'manhattan':
            compute_heuristics(self.maze, manhattan_distance)

        self.algorithm = algorithm
        self.current_position = STARTING_POSITION_OF_AGENT
        self.num_cells_processed_while_planning = 0
        self.final_paths = list()
        self.num_backtracks = 0
        self.parents = dict()
        self.num_confirmed_cells = 0
        self.num_astar_calls = 0

    def planning(self):
        # Choose which algorithm you want to use for search
        if self.algorithm == 'astar':
            self.parents, num_explored_nodes = astar_search(self.maze, self.current_position)[:2]
            self.num_cells_processed_while_planning += num_explored_nodes
        # elif self.algorithm == 'bfs':
        #     parents, num_explored_nodes = bfs_search(maze, start_pos, goal_pos)
        else:
            raise Exception("algorithm should be either astar or bfs")

    def reset_except_h(self):
        self.current_position = STARTING_POSITION_OF_AGENT
        self.num_cells_processed_while_planning = 0
        self.final_paths = list()
        self.num_backtracks = 0
        self.parents = dict()
        self.num_confirmed_cells = 0
        self.num_astar_calls = 0

        for row in range(NUM_ROWS):
            for col in range(NUM_COLS):
                self.maze[row][col].reset_except_h()

    @abstractmethod
    def execution(self, full_maze: np.array):
        pass
