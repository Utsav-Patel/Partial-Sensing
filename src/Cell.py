from constants import INF


class Cell:
    def __init__(self):
        self.g = INF
        self.h = INF
        self.f = INF

        self.is_blocked = False
        self.is_visited = False
        self.is_confirmed = False

        self.num_neighbor = 0

        self.num_confirmed_blocked = 0
        self.num_confirmed_unblocked = 0
        self.num_sensed_blocked = 0
        self.num_sensed_unblocked = 0

    def reset_except_h(self):
        self.g = INF
        self.f = INF

        self.is_blocked = False
        self.is_visited = False
        self.is_confirmed = False

        self.num_neighbor = 0

        self.num_confirmed_blocked = 0
        self.num_confirmed_unblocked = 0
        self.num_sensed_blocked = 0
        self.num_sensed_unblocked = 0

    def reset(self):
        self.reset_except_h()
        self.h = INF