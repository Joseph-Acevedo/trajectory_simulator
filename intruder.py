from constants import L

V_INTR = 0.1

class Intruder:

    def __init__(self, starting_pos):
        self.pos = starting_pos

    def move(self, time_step):
        self.pos[0] += V_INTR * time_step

    def get_pos(self):
        return self.pos

    def has_escaped(self):
        return self.pos[0] >= L