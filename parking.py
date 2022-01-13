import settings as stgs
import random as rdn
import math


class Parking_Lot:
    min_park_dist = stgs.min_dist * 1.5
    dist_from_road = stgs.node_width / 2

    def __init__(self):
        self.data = {}

    def __call__(self):
        return self.data

    def __getitem__(self, key):
        return self.data[key]

    def can_park(self, edge, pos, car_length, ID):
        if not edge in self().keys():
            return True
        # Check collisions
        # horizontal or vertical
        coord_index = 0 if edge[0][1] == edge[1][1] else 1

        # [0] pos [1] car_len [2] id
        for park in self()[edge]:
            dist = abs(park[0][coord_index] - pos[coord_index])
            min_dist = park[1] / 2 + car_length / 2 + self.min_park_dist

            """
            if park[2] == ID:
                print("ok...")
                print(pos, park[0])
                if park[0] == pos:
                    print("YOOO")
            """

            if dist < min_dist and ID != park[2]:
                return False

        return True

    def add_car(self, edge, pos, car_length, ID):
        if not edge in self().keys():
            self.data[edge] = []
        self.data[edge].append((pos, car_length, ID))

    def delete_car(self, edge, pos):
        if edge in self().keys():
            for i in range(len(self()[edge])):

                if self()[edge][i][0] == pos:
                    self.data[edge].pop(i)
                    if len(self.data[edge]) == 0:
                        del self.data[edge]
                    return

        raise Exception("ERROR: Car described doesn't exist")


parking = Parking_Lot()


def generate_goal(ID, last_node, before_last_node):
    predicted_nodes = (before_last_node, last_node)
    u_s = 0 if last_node[0] != before_last_node[0] else 1
    a = stgs.car_len / 2 + stgs.node_width / 2 + stgs.park_dist + parking.min_park_dist
    b = (
        abs(last_node[u_s] - before_last_node[u_s])
        - stgs.node_width / 2
        - stgs.car_len / 2
        - stgs.park_dist
        - parking.min_park_dist
    )

    if a >= b:
        return None
    goal = rdn.randint(
        math.ceil(a),
        math.floor(b),
    )

    # Parking reservation
    pos_dir = 1 if predicted_nodes[1][u_s] > predicted_nodes[0][u_s] else -1
    pos = list(last_node)

    pos[u_s] -= goal * pos_dir
    s_i = 0 if u_s == 1 else 1

    if s_i == 1:
        pos[s_i] += 12 * pos_dir
    else:
        pos[s_i] -= 12 * pos_dir

    target_pos = tuple(pos)

    if parking.can_park(predicted_nodes, target_pos, stgs.car_len, ID):
        return (goal, target_pos)
    else:
        return None
