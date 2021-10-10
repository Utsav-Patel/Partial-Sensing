import numpy as np

from src.Agent import Agent
from src.helper import parent_to_child_dict, sense_current_node, find_block_while_inference
from constants import GOAL_POSITION_OF_AGENT


class OurOwnInferenceAgent(Agent):
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

        while current_position != children[current_position]:

            if not self.maze[current_position[0]][current_position[1]].is_visited:
                self.maze[current_position[0]][current_position[1]].is_visited = True
                sense_current_node(self.maze, current_position, full_maze)

            if find_block_while_inference(self.maze, current_position, full_maze, entire_trajectory_nodes, True, True, True):
                break

            if full_maze[children[current_position][0]][children[current_position][1]] == 1:
                find_block_while_inference(self.maze, children[current_position], full_maze,
                                           entire_trajectory_nodes=None,
                                           want_to_use_one_node_inference_strategy=True,
                                           want_to_use_two_node_inference_strategy=True,
                                           want_to_use_three_node_inference_strategy=False)
                break
            else:
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
