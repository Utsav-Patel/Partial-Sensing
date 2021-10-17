# Necessary Imports
import numpy as np

from constants import GOAL_POSITION_OF_AGENT
from src.Agent import Agent
from src.helper import parent_to_child_dict, sense_current_node, find_block_while_inference


# Our Own Inference Agent's class
class OurOwnInferenceAgent(Agent):
    def __init__(self):
        super().__init__()

    # Override the execution method
    def execution(self, full_maze: np.array):
        self.num_astar_calls += 1
        children = parent_to_child_dict(self.parents, GOAL_POSITION_OF_AGENT)
        current_position = self.current_position

        # Used this set to check whether agent's path has block node or not in advance
        entire_trajectory_nodes = set()
        entire_trajectory_nodes.add(current_position)

        # Running until Goal is reached
        while current_position != children[current_position]:
            current_position = children[current_position]
            entire_trajectory_nodes.add(current_position)

        current_position = self.current_position
        current_path = list()
        current_path.append(current_position)

        # Running the following while loop until we reach goal state or block cell
        while True:

            # Storing the next four cells in agent's path so that we can infer whether the path contains block or not
            length_of_path_to_check_for_inference = 4
            current_node = current_position

            list_of_variables_in_the_path = list()
            while length_of_path_to_check_for_inference > 0:
                list_of_variables_in_the_path.append(current_node)
                length_of_path_to_check_for_inference -= 1
                current_node = children[current_node]

            # If the current position is not visited then mark it as visited and sense that cell
            if not self.maze[current_position[0]][current_position[1]].is_visited:
                self.maze[current_position[0]][current_position[1]].is_visited = True
                sense_current_node(self.maze, current_position, full_maze, knowledge_base=self.knowledge_base,
                                   variable_to_constraint_dict=self.variable_to_constraint_dict)

            # Calling the following function to make inferences and check whether we have found the block in our current
            # path or not
            if find_block_while_inference(self.maze, current_position, full_maze, entire_trajectory_nodes,
                                          want_to_use_one_node_inference_strategy=True,
                                          want_to_use_two_node_inference_strategy=True,
                                          want_to_use_three_node_inference_strategy=True,
                                          want_to_use_most_constraint_variable_for_backtracking_search=True,
                                          knowledge_base=self.knowledge_base,
                                          variable_to_constraint_dict=self.variable_to_constraint_dict):
                self.num_early_termination += 1
                break

            # If agent's next node is block then check and update the details
            if full_maze[children[current_position][0]][children[current_position][1]] == 1:
                find_block_while_inference(self.maze, children[current_position], full_maze,
                                           entire_trajectory_nodes=None,
                                           want_to_use_one_node_inference_strategy=True,
                                           want_to_use_two_node_inference_strategy=True,
                                           want_to_use_three_node_inference_strategy=True,
                                           want_to_use_most_constraint_variable_for_backtracking_search=True,
                                           knowledge_base=self.knowledge_base,
                                           variable_to_constraint_dict=self.variable_to_constraint_dict)
                self.num_bumps += 1
                break

            # Otherwise check for the terminating condition and change the current position
            else:
                if current_position == children[current_position]:
                    break
                current_position = children[current_position]
                current_path.append(current_position)

        self.final_paths.append(current_path)
        self.current_position = current_path[-1]
