from settings import *

tot_states = state_countdown * 2


class Light:
    def __init__(self, coord: tuple[float, float]):
        # Real coord
        self.coord = coord
        # Matrix coord
        self.matrix_coord = Light.get_matrix_coord(self.coord)
        self.set_state()
        # --print(self.initial_state)

    def set_state(self):
        # Formula
        row_state = self.matrix_coord[0] % tot_states
        cypher = self.matrix_coord[1] % tot_states
        row_state += cypher
        # Making it between the wanted numbers
        self.initial_state = row_state % tot_states
        self.state = self.initial_state

    @staticmethod
    def get_matrix_coord(coord: tuple[float, float]):
        x, y = coord

        x -= adjust_x
        y -= adjust_y

        x /= road_len
        y /= road_len

        x += round(adjust_x / road_len) * 2
        y += round(adjust_y / road_len) * 2

        if x < 0 or y < 0:
            raise ValueError("ERROR: LIGHTS MATRIX COORD HAVE NEGATIVE COORDINATES")
        elif int(x) != x or int(y) != y:
            raise ValueError("ERROR: LIGHTS MATRIX COORD HAVE DECIMAL COORDINATES")

        return (x, y)

    # For testing and debugging purposes
    def update_state(self):
        self.state += 1
        self.state = self.state % tot_states
