from src.TheBlindfoldedAgent import TheBlindfoldedAgent
from src.helper import generate_grid_manually, generate_grid_with_probability_p
from constants import GOAL_POSITION_OF_AGENT

blinded_folded_agent = TheBlindfoldedAgent()
full_maze = generate_grid_manually()

while blinded_folded_agent.current_position != GOAL_POSITION_OF_AGENT:
    blinded_folded_agent.planning()
    blinded_folded_agent.execution(full_maze)
    print(blinded_folded_agent.final_paths)

print("Hurray! Reach Goal")
print(blinded_folded_agent.final_paths)
