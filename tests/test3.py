"""
This file contains the comparison between Agent 4 and 5.
"""
from datetime import datetime

from constants import LIST_OF_PROBABILITIES, INF, STARTING_POSITION_OF_AGENT, GOAL_POSITION_OF_AGENT, IMG_PATH, \
    NUM_ITERATION_FOR_EACH_PROBABILITY
from src.Agent import Agent
from src.OurOwnInferenceAgent import OurOwnInferenceAgent
from src.TheBlindfoldedAgent import TheBlindfoldedAgent
from src.TheExampleInferenceAgent import TheExampleInferenceAgent
from src.TheFourNeighborAgent import TheFourNeighborAgent
from src.helper import generate_grid_with_probability_p, length_of_path_from_source_to_goal, avg, multiple_plot, \
    compute_num_confirmed_cells, compute_explored_cells_from_path, create_maze_array_from_discovered_grid, \
    compute_probability_of_being_block_for_each_cell_after_execution, compute_num_confirmed_blocked_cells

# Just printing this to know when the program execution is started
print('Start running this file at', datetime.now().strftime("%m-%d-%Y %H-%M-%S"))

num_agents_to_test = 5
legends = ['Blinded Folded', 'Four Neighbor', 'Example Inference', 'Agent 4', 'Agent 5']
agents = [TheBlindfoldedAgent(), TheFourNeighborAgent(), TheExampleInferenceAgent(), OurOwnInferenceAgent(),
          OurOwnInferenceAgent()]

avg_num_explored_cells = list()
avg_num_processed_cells = list()
avg_trajectory_length_by_shortest_path_in_final_discovered_grid = list()
avg_shortest_path_in_final_discovered_grid_by_full_grid = list()
avg_num_confirmed_cells = list()
avg_num_astar_calls = list()
avg_running_time = list()
avg_num_bumps = list()
avg_num_early_terminations = list()

for ind in range(num_agents_to_test):
    avg_num_explored_cells.append(list())
    avg_num_processed_cells.append(list())
    avg_trajectory_length_by_shortest_path_in_final_discovered_grid.append(list())
    avg_shortest_path_in_final_discovered_grid_by_full_grid.append(list())
    avg_num_confirmed_cells.append(list())
    avg_num_astar_calls.append(list())
    avg_running_time.append(list())
    avg_num_bumps.append(list())
    avg_num_early_terminations.append(list())


def compute_for_particular_agent(agent: Agent, num_explored_cells_list: list, num_processed_cells_list: list,
                                 trajectory_length_by_shortest_path_in_final_discovered_grid_list: list,
                                 shortest_path_in_final_discovered_grid_by_full_grid_list: list,
                                 num_confirmed_cells_list: list, num_astar_calls_list: list, running_time_list: list,
                                 num_bumps_list: list, num_early_termination_list: list,
                                 distance_from_source_to_goal_in_full_grid, is_agent5:bool = False):

    start_time = datetime.now()
    while agent.current_position != GOAL_POSITION_OF_AGENT:
        agent.planning()
        agent.execution(maze_array)
        if is_agent5:
            compute_probability_of_being_block_for_each_cell_after_execution(agent.maze)
    end_time = datetime.now()

    agent.num_confirmed_cells = compute_num_confirmed_cells(agent.maze)
    agent.num_confirmed_blocked_cells = compute_num_confirmed_blocked_cells(agent.maze)

    trajectory_length = compute_explored_cells_from_path(agent.final_paths)
    num_explored_cells_list.append(trajectory_length)
    num_processed_cells_list.append(agent.num_cells_processed_while_planning)

    distance_from_source_to_goal_in_discovered_grid = length_of_path_from_source_to_goal(
        create_maze_array_from_discovered_grid(agent.maze), STARTING_POSITION_OF_AGENT, GOAL_POSITION_OF_AGENT)

    trajectory_length_by_shortest_path_in_final_discovered_grid_list.append(trajectory_length / distance_from_source_to_goal_in_discovered_grid)
    shortest_path_in_final_discovered_grid_by_full_grid_list.append(distance_from_source_to_goal_in_discovered_grid /
                                                                    distance_from_source_to_goal_in_full_grid)

    num_confirmed_cells_list.append(agent.num_confirmed_cells)
    num_astar_calls_list.append(agent.num_astar_calls)
    running_time_list.append((end_time - start_time).total_seconds())
    num_bumps_list.append(agent.num_bumps)
    num_early_termination_list.append(agent.num_early_termination)

    probability_being_blocked = 0.0
    if is_agent5:
        probability_being_blocked = agent.num_confirmed_blocked_cells / agent.num_confirmed_cells
    agent.reset_except_h(probability_being_blocked)


for probability in LIST_OF_PROBABILITIES:

    print('Running for', probability)
    num_explored_cells = list()
    num_processed_cells = list()
    trajectory_length_by_shortest_path_in_final_discovered_grid = list()
    shortest_path_in_final_discovered_grid_by_full_grid = list()
    num_confirmed_cells = list()
    num_astar_calls = list()
    running_time = list()
    num_bumps = list()
    num_early_terminations = list()

    for ind in range(num_agents_to_test):
        num_explored_cells.append(list())
        num_processed_cells.append(list())
        trajectory_length_by_shortest_path_in_final_discovered_grid.append(list())
        shortest_path_in_final_discovered_grid_by_full_grid.append(list())
        num_confirmed_cells.append(list())
        num_astar_calls.append(list())
        running_time.append(list())
        num_bumps.append(list())
        num_early_terminations.append(list())

    num_run = 0

    while num_run < NUM_ITERATION_FOR_EACH_PROBABILITY:
        maze_array = generate_grid_with_probability_p(probability)
        distance_from_start_to_goal_in_full_grid = length_of_path_from_source_to_goal(maze_array,
                                                                                      STARTING_POSITION_OF_AGENT,
                                                                                      GOAL_POSITION_OF_AGENT)

        if distance_from_start_to_goal_in_full_grid >= INF:
            continue

        num_run += 1
        print('Iteration no:', num_run)

        for ind in range(len(agents)):
            is_agent5 = False
            if ind == len(agents) - 1:
                is_agent5 = True
            compute_for_particular_agent(agents[ind], num_explored_cells[ind], num_processed_cells[ind],
                                         trajectory_length_by_shortest_path_in_final_discovered_grid[ind],
                                         shortest_path_in_final_discovered_grid_by_full_grid[ind],
                                         num_confirmed_cells[ind], num_astar_calls[ind], running_time[ind],
                                         num_bumps[ind], num_early_terminations[ind],
                                         distance_from_start_to_goal_in_full_grid, is_agent5=is_agent5)

    for ind in range(len(agents)):
        avg_num_explored_cells[ind].append(avg(num_explored_cells[ind]))
        avg_num_processed_cells[ind].append(avg(num_processed_cells[ind]))
        avg_trajectory_length_by_shortest_path_in_final_discovered_grid[ind].append(
            avg(trajectory_length_by_shortest_path_in_final_discovered_grid[ind]))
        avg_shortest_path_in_final_discovered_grid_by_full_grid[ind].append(
            avg(shortest_path_in_final_discovered_grid_by_full_grid[ind]))
        avg_num_confirmed_cells[ind].append(avg(num_confirmed_cells[ind]))
        avg_num_astar_calls[ind].append(avg(num_astar_calls[ind]))
        avg_running_time[ind].append(avg(running_time[ind]))
        avg_num_bumps[ind].append(avg(num_bumps[ind]))
        avg_num_early_terminations[ind].append(avg(num_early_terminations[ind]))

# Ending executing this file after adding necessary plots
print('Ending running this file at', datetime.now().strftime("%m-%d-%Y %H-%M-%S"))

multiple_plot(LIST_OF_PROBABILITIES * 100, avg_num_explored_cells, "Number of explored cells", "Density (in %)",
              "Num of explored cells", IMG_PATH + "explored_nodes" + str(datetime.now().strftime("%m-%d-%Y %H-%M-%S")) + ".png", legends)

multiple_plot(LIST_OF_PROBABILITIES * 100, avg_num_processed_cells, "Number of processed cells", "Density (in %)",
              "Num of processed cells", IMG_PATH + "processed_cells" + str(datetime.now().strftime("%m-%d-%Y %H-%M-%S")) + ".png", legends)

multiple_plot(LIST_OF_PROBABILITIES * 100, avg_trajectory_length_by_shortest_path_in_final_discovered_grid,
              "Trajectory Length by shortest length in discovered grid", "Density (in %)", "Ratio of two lengths",
              IMG_PATH + "trajectory_length_by_shortest_path_in_final_discovered_grid" +
              str(datetime.now().strftime("%m-%d-%Y %H-%M-%S")) + ".png", legends)

multiple_plot(LIST_OF_PROBABILITIES * 100, avg_shortest_path_in_final_discovered_grid_by_full_grid,
              "Shortest length in (final discovered / full) grid", "Density (in %)", "Ratio of two lengths",
              IMG_PATH + "shortest_path_in_final_discovered_by_full_grid" +
              str(datetime.now().strftime("%m-%d-%Y %H-%M-%S")) + ".png", legends)

multiple_plot(LIST_OF_PROBABILITIES * 100, avg_num_confirmed_cells, "Number of confirmed cells", "Density (in %)",
              "Num of confirmed cells", IMG_PATH + "confirmed_cells" + str(datetime.now().strftime("%m-%d-%Y %H-%M-%S")) + ".png", legends)

multiple_plot(LIST_OF_PROBABILITIES * 100, avg_num_astar_calls, "Number of astar calls", "Density (in %)",
              "Num of astar calls", IMG_PATH + "num_astar_calls" + str(datetime.now().strftime("%m-%d-%Y %H-%M-%S")) + ".png", legends)

multiple_plot(LIST_OF_PROBABILITIES * 100, avg_running_time, "Running time (in seconds)", "Density (in %)",
              "Running time (in seconds)", IMG_PATH + "running_time" + str(datetime.now().strftime("%m-%d-%Y %H-%M-%S")) + ".png", legends)

multiple_plot(LIST_OF_PROBABILITIES * 100, avg_num_bumps, "Number of bumps", "Density (in %)",
              "Number of bumps", IMG_PATH + "num_bumps" + str(datetime.now().strftime("%m-%d-%Y %H-%M-%S")) + ".png", legends)

multiple_plot(LIST_OF_PROBABILITIES * 100, avg_num_early_terminations, "Number of early terminations", "Density (in %)",
              "Number of early terminations", IMG_PATH + "num_early_termination" +
              str(datetime.now().strftime("%m-%d-%Y %H-%M-%S")) + ".png", legends)

# Difference Graphs
diff_legends = ['Agent5 - Agent1', 'Agent5 - Agent2', 'Agent5 - Agent3', 'Agent5 - Agent4']
avg_num_diff_confirmed_cells = list()
avg_num_diff_early_termination_cells = list()
avg_num_diff_bumps = list()

for ind in range(len(agents)-1):
    diff_list_for_confirmed_cells = list()
    diff_list_for_early_termination_cells = list()
    diff_list_for_bumps = list()

    for ind2 in range(len(avg_num_confirmed_cells[ind])):
        diff_list_for_confirmed_cells.append(avg_num_confirmed_cells[len(agents) - 1][ind2] - avg_num_confirmed_cells[ind][ind2])
        diff_list_for_early_termination_cells.append(avg_num_early_terminations[len(agents) - 1][ind2] - avg_num_early_terminations[ind][ind2])
        diff_list_for_bumps.append(avg_num_bumps[len(agents) - 1][ind2] - avg_num_bumps[ind][ind2])

    avg_num_diff_confirmed_cells.append(diff_list_for_confirmed_cells)
    avg_num_diff_bumps.append(diff_list_for_bumps)
    avg_num_diff_early_termination_cells.append(diff_list_for_early_termination_cells)

multiple_plot(LIST_OF_PROBABILITIES * 100, avg_num_diff_confirmed_cells, "Difference of confirmed cells",
              "Density (in %)", "Difference",
              IMG_PATH + "diff_confirmed_cells" + str(datetime.now().strftime("%m-%d-%Y %H-%M-%S")) + ".png", diff_legends)

multiple_plot(LIST_OF_PROBABILITIES * 100, avg_num_diff_early_termination_cells, "Difference of early termination",
              "Density (in %)", "Difference",
              IMG_PATH + "diff_early_termination" + str(datetime.now().strftime("%m-%d-%Y %H-%M-%S")) + ".png", diff_legends)

multiple_plot(LIST_OF_PROBABILITIES * 100, avg_num_diff_bumps, "Difference in bumps",
              "Density (in %)", "Difference",
              IMG_PATH + "diff_num_bumps" + str(datetime.now().strftime("%m-%d-%Y %H-%M-%S")) + ".png", diff_legends)
