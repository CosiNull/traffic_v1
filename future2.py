import settings as stgs
import traffic as trf
import math


relative_dir = {
    "u": {"u": "u", "r": "r", "l": "l"},
    "r": {"r": "u", "u": "l", "d": "r"},
    "l": {"l": "u", "u": "r", "d": "l"},
    "d": {"d": "u", "r": "l", "l": "r"},
}

opposite_dir = {"u": "d", "r": "l", "l": "r", "d": "u"}

time_turn = {
    "u": 49,
    "l": 51,
    "r": 22,
}

junction_space = stgs.node_width / 2 + stgs.car_len / 2


def binary_search(elem, arr, func):
    start = 0
    end = len(arr)

    while start < end:
        mid = int((start + end) / 2)

        if func(arr[mid]) < func(elem):
            start = mid + 1
        else:
            end = mid

    return start


def binary_search_ds(elem, arr, func=lambda x: x[1]):
    start = 0
    end = len(arr)

    while start < end:
        mid = int((start + end) / 2)

        if func(arr[mid]) < elem:
            start = mid + 1
        else:
            end = mid

    return start


def binary_insertion(elem, arr, func):
    arr.insert(binary_search(elem, arr, func), elem)


class Intersection:
    def __init__(self, entries):
        self.entries = {entry: [] for entry in entries}  # entry: [(ID,time)]
        self.crossing_enter = []
        self.entry_cross = {entry: [] for entry in entries}
        self.crossing_exit = []

    def add_car_entry(self, ID, entry, time):
        elem = (ID, time)
        arr = self.entries[entry]
        func = lambda x: x[1]

        binary_insertion(elem, arr, func)

    def add_car_crossing(self, ID, time, entry, entry_to):
        elem = (ID, time, entry, entry_to)
        arr = self.crossing_enter
        func = lambda x: x[1]
        binary_insertion(elem, arr, func)

        self.entry_cross[entry].append((ID, time, entry_to))

        time_crossing = time_turn[relative_dir[opposite_dir[entry]][entry_to]]
        elem_2 = (ID, time + time_crossing, entry, entry_to)
        arr_2 = self.crossing_exit
        binary_insert_q(elem_2, arr_2, func=func)


class Road:
    def __init__(self, node1, node2):
        self.enter = []
        self.estimation = []
        self.in_road = []
        self.leave = []
        self.timely_capacity = [(0, 0, "i")]  # (capacity, time, what_happened)

        self.max_capacity = trf.Road.max_capacity(node1, node2)
        self.curr_capacity = 0
        self.start = node1
        self.to = node2

    def add_car_enter(self, ID, time, dist):
        elem = (ID, time, dist)
        arr = self.enter
        func = lambda x: x[1]

        binary_insertion(elem, arr, func)

    def add_car_junc_estimation(self, ID, time, dist):
        timing = Road.estimate_arrive(time, dist)
        elem = (ID, timing)
        arr = self.estimation
        func = lambda x: x[1]

        binary_insertion(elem, arr, func=func)
        self.curr_capacity += 1

        self.timely_capacity.append((self.curr_capacity, time, "i"))

    def add_car_exit(self, ID, time):
        elem = (ID, time)
        arr = self.leave
        func = lambda x: x[1]

        binary_insertion(elem, arr, func)
        self.curr_capacity -= 1

        self.timely_capacity.append((self.curr_capacity, time, "d"))

    def delete_line(self, time):
        self.line.append(self.line[-1][0], time, self.line[-1][2] - 1)

    def get_car_dist(self, pos):
        ind = 0 if self.start[0] != self.to[0] else 1
        return abs(self.to[ind] - pos[ind])

    @staticmethod
    def estimate_arrive(time, dist):
        # Check if the road is empty at that time
        travel_time = math.ceil(dist / stgs.car_speed)
        return travel_time + time
        # Else arrive at a certain time


# _______________________________________________________________________________________________________
def make_intersection_dict(graph):
    return {
        node: Intersection(
            [
                trf.entry_dir(node, connection[0])
                for connection in graph.connections[node]
            ]
        )
        for node in graph.nodes
    }


def make_road_dict(road_network):
    node = next(iter(road_network.nodes))
    vis = set()
    res = dict()

    def dfs(graph, vis_node):
        for neighbour in graph.connections[vis_node]:
            if not (vis_node, neighbour[0]) in vis:
                vis.add((vis_node, neighbour[0]))
                road_dir = trf.entry_dir(vis_node, neighbour[0])
                res[vis_node, road_dir] = Road(vis_node, neighbour[0])
                # print(vis_node, neighbour[0])
                dfs(graph, neighbour[0])

    dfs(road_network, node)

    return res


# _____________
def linear_search(elem, arr, func=lambda x: x[0], step=1):
    for i in range(0, len(arr), step):
        if func(arr[i]) == elem:
            return i


def backward_linear_s(elem, arr, func=lambda x: x[0], step=-1):
    for i in range(len(arr) - 1, -1, step):
        if func(arr[i]) == elem:
            return i


def reLu(x):
    return x if x > 0 else 0


# ____________________________________
def binary_search_q(elem, arr, func=lambda x: x[2]):
    func_2 = lambda x: x[0]
    start = 0
    end = len(arr)

    while start < end:
        mid = int((start + end) / 2)
        if func(arr[mid]) < func(elem):
            start = mid + 1
        elif func(arr[mid]) == func(elem):
            if func_2(arr[mid]) < func_2(elem):
                start = mid + 1
            else:
                end = mid
        else:
            end = mid

    return start


def binary_insert_q(elem, arr, func=lambda x: x[2]):
    arr.insert(binary_search_q(elem, arr, func), elem)


# _____________________________________


def entry_collision(other_dist, my_dist, ID, ID_b):
    if other_dist <= my_dist and my_dist - other_dist < 12 + stgs.car_len:
        return True
    elif other_dist > my_dist and other_dist - my_dist < stgs.car_len + 10:
        return not (other_dist - my_dist == 17.5 and ID < ID_b)

    return False
