import numpy as np

from src.Agent import Agent
from src.helper import parent_to_child_dict, check
from constants import GOAL_POSITION_OF_AGENT, NUM_ROWS, NUM_COLS
from queue import Queue


class TheExampleInferenceAgent(Agent):
    def __init__(self):
        super().__init__()

    def inference(self, current_position: tuple, full_maze: np.array, entire_trajectory_nodes=None):
        if entire_trajectory_nodes is None:
            entire_trajectory_nodes = set()

        inference_items = Queue()
        items_in_the_queue = set()
        is_block_node_in_current_path = False

        inference_items.put(current_position)
        items_in_the_queue.add(current_position)

        while not inference_items.empty():
            current_node = inference_items.get()
            items_in_the_queue.remove(current_node)

            if not self.maze[current_node[0]][current_node[1]].is_confirmed:

                self.maze[current_node[0]][current_node[1]].is_confirmed = True

                if full_maze[current_node[0]][current_node[1]] == 1:
                    self.maze[current_node[0]][current_node[1]].is_blocked = True
                    if current_node in entire_trajectory_nodes:
                        is_block_node_in_current_path = True
                else:
                    self.maze[current_node[0]][current_node[1]].is_blocked = False

                for row in range(-1, 2):
                    for col in range(-1, 2):
                        neighbor = (current_node[0] + row, current_node[1] + col)
                        if (not check(neighbor)) or (current_node == neighbor) or \
                                (not self.maze[neighbor[0]][neighbor[1]].is_visited):
                            continue
                        if self.maze[current_node[0]][current_node[1]].is_blocked:
                            self.maze[neighbor[0]][neighbor[1]].num_confirmed_blocked += 1
                        else:
                            self.maze[neighbor[0]][neighbor[1]].num_confirmed_unblocked += 1

                        if not (neighbor in items_in_the_queue):
                            items_in_the_queue.add(neighbor)
                            inference_items.put(neighbor)

            if self.maze[current_node[0]][current_node[1]].is_visited:
                if ((self.maze[current_node[0]][current_node[1]].num_sensed_blocked ==
                     self.maze[current_node[0]][current_node[1]].num_confirmed_blocked) and
                        (self.maze[current_node[0]][current_node[1]].num_sensed_unblocked !=
                         self.maze[current_node[0]][current_node[1]].num_confirmed_unblocked)) or \
                        ((self.maze[current_node[0]][current_node[1]].num_sensed_unblocked ==
                          self.maze[current_node[0]][current_node[1]].num_confirmed_unblocked) and
                         (self.maze[current_node[0]][current_node[1]].num_sensed_blocked !=
                          self.maze[current_node[0]][current_node[1]].num_confirmed_blocked)):

                    for row in range(-1, 2):
                        for col in range(-1, 2):
                            neighbor = (current_node[0] + row, current_node[1] + col)

                            if check(neighbor) and (current_node != neighbor) and \
                                    (neighbor not in items_in_the_queue) and \
                                    (not self.maze[neighbor[0]][neighbor[1]].is_confirmed):
                                items_in_the_queue.add(neighbor)
                                inference_items.put(neighbor)
        return is_block_node_in_current_path

    def sense_current_node(self, current_position: tuple, full_maze: np.array):

        for row in range(-1, 2):
            for col in range(-1, 2):
                neighbor = (current_position[0] + row, current_position[1] + col)

                if current_position == neighbor:
                    continue

                if check(neighbor):
                    self.maze[current_position[0]][current_position[1]].num_neighbor += 1

                    if self.maze[neighbor[0]][neighbor[1]].is_confirmed:
                        if self.maze[neighbor[0]][neighbor[1]].is_blocked:
                            self.maze[current_position[0]][current_position[1]].num_confirmed_blocked += 1
                            self.maze[current_position[0]][current_position[1]].num_sensed_blocked += 1
                        else:
                            self.maze[current_position[0]][current_position[1]].num_confirmed_unblocked += 1
                            self.maze[current_position[0]][current_position[1]].num_sensed_unblocked += 1
                    else:
                        if full_maze[neighbor[0]][neighbor[1]] == 1:
                            self.maze[current_position[0]][current_position[1]].num_sensed_blocked += 1
                        else:
                            self.maze[current_position[0]][current_position[1]].num_sensed_unblocked += 1

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
                self.sense_current_node(current_position, full_maze)

            if self.inference(current_position, full_maze, entire_trajectory_nodes):
                break

            if full_maze[children[current_position][0]][children[current_position][1]] == 1:
                self.inference(children[current_position], full_maze)
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
