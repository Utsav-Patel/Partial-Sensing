# Necessary imports

from constants import INF


# Cell class
class Cell:
    # Initialise variables for the cell class
    def __init__(self):
        self.g = INF
        self.h = INF
        self.f = INF

        self.is_blocked = False
        self.is_visited = False
        self.is_confirmed = False

        self.num_neighbor = 0
        self.min_hidden_cell_neighbor = INF

        self.num_confirmed_blocked = 0
        self.num_confirmed_unblocked = 0
        self.num_sensed_blocked = 0
        self.num_sensed_unblocked = 0

        self.probability_of_being_blocked = 0.0
        self.eight_neighbors = list()

    # Reset attributes of this class
    def reset_except_h(self, default_probability: float = 0.0):
        self.g = INF
        self.f = INF

        self.is_blocked = False
        self.is_visited = False
        self.is_confirmed = False

        # self.num_neighbor = 0
        self.min_hidden_cell_neighbor = INF

        self.num_confirmed_blocked = 0
        self.num_confirmed_unblocked = 0
        self.num_sensed_blocked = 0
        self.num_sensed_unblocked = 0

        self.probability_of_being_blocked = default_probability

    def reset(self):
        self.reset_except_h()
        self.h = INF
