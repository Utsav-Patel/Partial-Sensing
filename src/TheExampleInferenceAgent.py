import numpy as np

from constants import GOAL_POSITION_OF_AGENT
from src.Agent import Agent
from src.helper import parent_to_child_dict, sense_current_node, find_block_while_inference


class TheExampleInferenceAgent(Agent):
    def __init__(self):
        super().__init__()

    def execution(self, full_maze: np.array):
        self.num_astar_calls += 1
        children = parent_to_child_dict(self.parents, GOAL_POSITION_OF_AGENT)
        current_position = self.current_position

        entire_trajectory_nodes = set()
        entire_trajectory_nodes.add(current_position)

        while current_position != children[current_position]:
            current_position = children[current_position]
            entire_trajectory_nodes.add(current_position)

        current_position = self.current_position
        current_path = list()
        current_path.append(current_position)

        while True:

            if not self.maze[current_position[0]][current_position[1]].is_visited:
                self.maze[current_position[0]][current_position[1]].is_visited = True
                sense_current_node(self.maze, current_position, full_maze)

            if find_block_while_inference(self.maze, current_position, full_maze, entire_trajectory_nodes):
                self.num_early_termination += 1
                break

            if full_maze[children[current_position][0]][children[current_position][1]] == 1:
                find_block_while_inference(self.maze, children[current_position], full_maze)
                self.num_bumps += 1
                break
            else:
                if current_position == children[current_position]:
                    break
                current_position = children[current_position]
                current_path.append(current_position)

        # print("Current path", current_path)
        #
        # print('Visited')
        # for row in range(NUM_ROWS):
        #     for col in range(NUM_COLS):
        #         print(self.maze[row][col].is_visited, end=" ")
        #     print()
        #
        # print('Confirmed')
        # for row in range(NUM_ROWS):
        #     for col in range(NUM_COLS):
        #         print(self.maze[row][col].is_confirmed, end=" ")
        #     print()
        #
        # print('Blocked')
        # for row in range(NUM_ROWS):
        #     for col in range(NUM_COLS):
        #         print(self.maze[row][col].is_blocked, end=" ")
        #     print()
        #
        # print('Num neighbor')
        # for row in range(NUM_ROWS):
        #     for col in range(NUM_COLS):
        #         print(self.maze[row][col].num_neighbor, end=" ")
        #     print()
        #
        # print('Num confirmed blocked')
        # for row in range(NUM_ROWS):
        #     for col in range(NUM_COLS):
        #         print(self.maze[row][col].num_confirmed_blocked, end=" ")
        #     print()
        #
        # print('Num confirmed unblocked')
        # for row in range(NUM_ROWS):
        #     for col in range(NUM_COLS):
        #         print(self.maze[row][col].num_confirmed_unblocked, end=" ")
        #     print()
        #
        # print('Num sensed blocked')
        # for row in range(NUM_ROWS):
        #     for col in range(NUM_COLS):
        #         print(self.maze[row][col].num_sensed_blocked, end=" ")
        #     print()
        #
        # print('Num sensed unblocked')
        # for row in range(NUM_ROWS):
        #     for col in range(NUM_COLS):
        #         print(self.maze[row][col].num_sensed_unblocked, end=" ")
        #     print()

        self.final_paths.append(current_path)
        self.current_position = current_path[-1]
