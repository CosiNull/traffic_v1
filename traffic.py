from typing import Tuple
import settings as stgs
import pickle
import math

# System |Initial Char: From| Second Char: To|
# Not including same direction turns because it might create a collision with the speed difference
no_conflicts = {
    # _________________________ UP
    ("u", "d"): {
        ("d", "u"),
        ("d", "r"),
        ("r", "u"),
        ("u", "d"),
        ("u", "l"),
    },  # From Up go straight
    ("u", "l"): {
        ("d", "r"),
        ("d", "u"),
        ("r", "u"),
        ("r", "d"),
        ("l", "r"),
        ("l", "u"),  # Consider taking out?
        ("l", "d"),
        ("u", "l"),
    },  # From Up turn right
    ("u", "r"): {
        ("l", "d"),
        ("r", "u"),  # Consider taking out?
        ("u", "r"),
    },  # From Up turn left
    # __________________________ RIGHT
    ("r", "l"): {
        ("l", "r"),
        ("l", "d"),
        ("d", "r"),
        ("r", "l"),
        ("r", "d"),
    },  # From down go straight
    ("r", "u"): {
        ("d", "r"),
        ("d", "l"),
        ("l", "r"),
        ("l", "d"),
        ("u", "l"),
        ("u", "r"),  # Consider taking out?
        ("u", "d"),
        ("r", "u"),
    },  # From down turn right
    ("r", "d"): {
        ("d", "r"),  # Consider taking out?
        ("u", "l"),
        ("r", "d"),
    },  # From right turn left
    # _______________________________DOWN
    ("d", "u"): {
        ("u", "d"),
        ("u", "l"),
        ("l", "d"),
        ("d", "u"),
        ("d", "r"),
    },  # From down go straight
    ("d", "r"): {
        ("u", "d"),
        ("l", "u"),
        ("l", "d"),
        ("r", "u"),
        ("r", "l"),
        ("r", "d"),  # Consider taking out?
        ("d", "r"),
    },  # From down move right
    ("d", "l"): {
        ("l", "d"),  # Consider taking out?
        ("r", "u"),
        ("d", "l"),
    },
    # __________________________________LEFT
    ("l", "r"): {
        ("r", "l"),
        ("r", "u"),
        ("u", "l"),
        ("l", "r"),
        ("l", "d"),
    },  # From down go straight
    ("l", "d"): {
        ("u", "l"),
        ("u", "r"),
        ("u", "l"),
        ("d", "r"),
        ("d", "l"),  # Consider taking out?
        ("d", "u"),
        ("l", "d"),
    },  # From down turn right
    ("l", "u"): {
        ("u", "l"),  # Consider taking out?
        ("d", "r"),
        ("l", "u"),
    },  # From right turn left
}

# NOTE MIGHT NEED TO REWRITE THIS PART
class Intersection:
    def __init__(self, entries):
        self.entries = {entry: [] for entry in entries}  # Car
        self.queue = []  # Car

        # Data structure
        self.crossing = []

    def add_car_entry(self, ID, entry):
        self.entries[entry].append(ID)

    """
    def add_car_queue(self, ID, entry):
        if self.entries[entry][0] == ID:
            ind = self.get_index_car_first_entry()
            self.queue.insert(ind, (ID, entry))
        else:
            self.queue.append((ID, entry))
    """

    def get_index_car_first_entry(self):
        i = 0
        for ID, entry in self.queue:
            if len(self.entries[entry]) > 0 and self.entries[entry][0] == ID:
                i += 1
            else:
                break
        return i

    def remove_car(self, ID, entry):
        # self.queue.remove((ID, entry))
        self.entries[entry].remove(ID)

        # Bypass the others in line
        """
        if len(self.entries[entry]) >= 1:
            ind = self.get_index_car_first_entry()
            q_index = self.queue.index((self.entries[entry][0], entry))

            if q_index > 4:
                self.queue.pop(q_index)
                self.queue.insert(ind, (self.entries[entry][0], entry))
        """

    def num_cars_entry(self, entry):
        return len(self.entries[entry])

    """
    def crossable_first_glance(self, start_entry, target_entry):
        # NOTE might change data structure for crossing might not just be tuples
        return all(
            [
                car_route in no_conflicts[(start_entry, target_entry)]
                for car_route in self.crossing
            ]
        )
    """

    @property
    def queue_front(self):
        return self.queue[0]


def entry_dir(start_node: Tuple[float, float], end_node: Tuple[float, float]):
    if start_node[0] == end_node[0]:  # Vertical
        if start_node[1] < end_node[1]:
            return "d"
        else:
            return "u"
    elif start_node[1] == end_node[1]:  # Horizontal
        if start_node[0] < end_node[0]:
            return "r"
        else:
            return "l"
    raise Exception("ERROR: UNVALID SET OF TUPLE PARAMETERS")


inverse_dir = {"d": "u", "u": "d", "r": "l", "l": "r"}


def intersection_dist(intersection, pos, angle):
    if angle == math.pi or angle == 0:
        u_s = 0
    elif angle == math.pi / 2 or math.pi * 3 / 2:
        u_s = 1

    return abs(intersection[u_s] - pos[u_s])


def make_intersections_dict(graph):
    return {
        node: Intersection(
            [entry_dir(node, connection[0]) for connection in graph.connections[node]]
        )
        for node in graph.nodes
    }


angle_to_intersect = {0: "l", math.pi / 2: "u", math.pi: "r", math.pi * 3 / 2: "d"}


# Class road
class Road:
    def __init__(self, node1, node2):
        self.cars = []
        self.max_capacity = Road.max_capacity(node1, node2)

        self.start = node1
        self.to = node2

    @staticmethod
    def max_capacity(node1, node2):
        u_s = 0 if node1[0] != node2[0] else 1
        length = abs(node1[u_s] - node2[u_s]) - stgs.node_width

        return int(length / (stgs.car_len + stgs.min_dist))

    def is_full(self):
        return len(self.cars) >= self.max_capacity

    # Controlling the datastructure
    def add_car(self, car_instance, sort=False):
        self.cars.append(car_instance)

        if sort:  # Sort by dist from goal
            self.cars.sort(key=lambda car: self.get_car_dist(car))

    def get_car_dist(self, car):
        ind = 0 if self.start[0] != self.to[0] else 1
        return abs(self.to[ind] - car.pos[ind])

    def pop_car(self, car):
        popped = self.cars.pop(0)
        if popped != car:
            raise Exception(
                "Oh no, it removed the wrong car"
            )  # For debugging purposes for now

    def remove_car(self, car):
        self.cars.remove(car)

    def index_car(self, car):
        return self.cars.index(car)

    # Collision detection
    def car_dist(self, car1, car2):  # Can be optimized but the code is beautiful so...
        dist1 = self.get_car_dist(car1)
        dist2 = self.get_car_dist(car2)

        return abs(dist1 - dist2)

    def can_out_parking(self, my_car):
        my_dist = self.get_car_dist(my_car)

        for car in self.cars:
            if car.state <= 1:
                continue
            car_dist = self.get_car_dist(car)
            # check if it is in correct interval
            if car_dist <= my_dist and abs(car_dist - my_dist) < 12 + car.len:
                return False
            elif car_dist > my_dist and abs(car_dist - my_dist) < car.len + 10:
                return False
        return True


def make_roads_dict(road_network):
    node = next(iter(road_network.nodes))
    vis = set()
    res = dict()

    def dfs(graph, vis_node):
        for neighbour in graph.connections[vis_node]:
            if not (vis_node, neighbour[0]) in vis:
                vis.add((vis_node, neighbour[0]))
                road_dir = entry_dir(vis_node, neighbour[0])
                res[vis_node, road_dir] = Road(vis_node, neighbour[0])
                # print(vis_node, neighbour[0])
                dfs(graph, neighbour[0])

    dfs(road_network, node)

    return res


def abs_dir(current_dir, turn):
    dir_dict = {
        "u": {"u": "u", "r": "r", "l": "l"},
        "r": {"u": "r", "r": "d", "l": "u"},
        "l": {"u": "l", "r": "u", "l": "d"},
        "d": {"u": "d", "r": "l", "l": "r"},
    }
    return dir_dict[current_dir][turn]


# Loading content and generating
with open(stgs.road_file, "rb") as f:
    road_network = pickle.load(f)
junctions = make_intersections_dict(road_network)
roads = make_roads_dict(road_network)
