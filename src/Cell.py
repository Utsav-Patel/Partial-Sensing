from constants import INF


class Cell:
    def __init__(self):
        self.is_blocked = False
        self.g = INF
        self.h = INF
        self.f = INF
        self.num_neighbor = 0
        self.is_visited = False
        self.is_confirmed = False
        self.num_sensed_blocked = 0
        self.num_sensed_unblocked = 0
        self.num_unconfirmed_cells = 0
