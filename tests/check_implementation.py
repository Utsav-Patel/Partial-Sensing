from src.TheBlindfoldedAgent import TheBlindfoldedAgent
from src.TheFourNeighborAgent import TheFourNeighborAgent
from src.TheExampleInferenceAgent import TheExampleInferenceAgent

from src.helper import generate_grid_manually, generate_grid_with_probability_p
from constants import GOAL_POSITION_OF_AGENT

full_maze = generate_grid_manually()
print(full_maze)

# Check Implementation for blindfolded agent
# blinded_folded_agent = TheBlindfoldedAgent()

# while blinded_folded_agent.current_position != GOAL_POSITION_OF_AGENT:
#     blinded_folded_agent.planning()
#     blinded_folded_agent.execution(full_maze)
#
# print("Hurray! Reach Goal")
# print(blinded_folded_agent.final_paths)

# Check Implementation for four dimension agent
# four_neighbor_agent = TheFourNeighborAgent()
#
# while four_neighbor_agent.current_position != GOAL_POSITION_OF_AGENT:
#     four_neighbor_agent.planning()
#     four_neighbor_agent.execution(full_maze)
#
# print("Hurray! Reach Goal")
# print(four_neighbor_agent.final_paths)

# Check Implementation for example inference agent

example_inference_agent = TheExampleInferenceAgent()

while example_inference_agent.current_position != GOAL_POSITION_OF_AGENT:
    example_inference_agent.planning()
    example_inference_agent.execution(full_maze)

print("Hurray! Reach Goal")
print(example_inference_agent.final_paths)
