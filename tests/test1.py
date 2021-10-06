"""
This file contains the comparison between Agent 1,2, and 3.
"""
from datetime import datetime

from src.helper import generate_grid_with_probability_p, length_of_path_from_source_to_goal, \
    compute_num_confirmed_cells, compute_explored_cells_from_path, multiple_plot, avg
from constants import LIST_OF_PROBABILITIES, INF, STARTING_POSITION_OF_AGENT, GOAL_POSITION_OF_AGENT, \
    NUM_ITERATION_FOR_EACH_PROBABILITY

from src.TheBlindfoldedAgent import TheBlindfoldedAgent
from src.TheFourNeighborAgent import TheFourNeighborAgent
from src.TheExampleInferenceAgent import TheExampleInferenceAgent
from src.Agent import Agent

# Just printing this to know when the program execution is started
print('Start running this file at', datetime.now().strftime("%m-%d-%Y %H-%M-%S"))

legends = ['Blinded Folded', 'Four Neighbor', 'Example Inference']
agents = [TheBlindfoldedAgent(), TheFourNeighborAgent(), TheExampleInferenceAgent()]
avg_num_explored_cells = [list(), list(), list()]
avg_num_processed_cells = [list(), list(), list()]
avg_num_confirmed_cells = [list(), list(), list()]
avg_num_astar_calls = [list(), list(), list()]
avg_running_time = [list(), list(), list()]


def compute_for_particular_agent(agent: Agent, num_explored_cells_list: list, num_processed_cells_list: list,
                                 num_confirmed_cells_list: list, num_astar_calls_list: list, running_time_list: list):
    agent.reset_except_h()

    start_time = datetime.now()
    while agent.current_position != GOAL_POSITION_OF_AGENT:
        agent.planning()
        agent.execution(maze_array)
    end_time = datetime.now()

    agent.num_confirmed_cells = compute_num_confirmed_cells(agent.maze)

    num_explored_cells_list.append(compute_explored_cells_from_path(agent.final_paths))
    num_processed_cells_list.append(agent.num_cells_processed_while_planning)
    num_confirmed_cells_list.append(agent.num_confirmed_cells)
    num_astar_calls_list.append(agent.num_astar_calls)
    running_time_list.append((end_time - start_time).total_seconds())


for probability in LIST_OF_PROBABILITIES:

    print('Running for', probability)
    num_explored_cells = [list(), list(), list()]
    num_processed_cells = [list(), list(), list()]
    num_confirmed_cells = [list(), list(), list()]
    num_astar_calls = [list(), list(), list()]
    running_time = [list(), list(), list()]

    num_run = 0

    while num_run < NUM_ITERATION_FOR_EACH_PROBABILITY:
        maze_array = generate_grid_with_probability_p(probability)
        distance_from_start_to_goal_in_full_maze = length_of_path_from_source_to_goal(maze_array,
                                                                                      STARTING_POSITION_OF_AGENT,
                                                                                      GOAL_POSITION_OF_AGENT)

        if distance_from_start_to_goal_in_full_maze >= INF:
            continue

        num_run += 1

        for ind in range(len(agents)):
            compute_for_particular_agent(agents[ind], num_explored_cells[ind], num_processed_cells[ind],
                                         num_confirmed_cells[ind], num_astar_calls[ind], running_time[ind])

    for ind in range(len(agents)):
        avg_num_explored_cells[ind].append(avg(num_explored_cells[ind]))
        avg_num_processed_cells[ind].append(avg(num_processed_cells[ind]))
        avg_num_confirmed_cells[ind].append(avg(num_confirmed_cells[ind]))
        avg_num_astar_calls[ind].append(avg(num_astar_calls[ind]))
        avg_running_time[ind].append(avg(running_time[ind]))

# Ending executing this file after adding necessary plots
print('Ending running this file at', datetime.now().strftime("%m-%d-%Y %H-%M-%S"))

multiple_plot(LIST_OF_PROBABILITIES, avg_num_explored_cells, "Number of explored cells", "Density",
              "Num of explored cells", "explored_nodes.png", legends)

multiple_plot(LIST_OF_PROBABILITIES, avg_num_processed_cells, "Number of processed cells", "Density",
              "Num of processed cells", "processed_cells.png", legends)

multiple_plot(LIST_OF_PROBABILITIES, avg_num_confirmed_cells, "Number of confirmed cells", "Density",
              "Num of confirmed cells", "confirmed_cells.png", legends)

multiple_plot(LIST_OF_PROBABILITIES, avg_num_astar_calls, "Number of astar calls", "Density",
              "Num of astar calls", "num_astar_calls.png", legends)

multiple_plot(LIST_OF_PROBABILITIES, avg_running_time, "Running time", "Density",
              "Running time (in seconds)", "running_time.png", legends)
