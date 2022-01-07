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

        time_crossing = time_turn[relative_dir[opposite_dir[entry]][entry_to]]
        elem_2 = (ID, time + time_crossing, entry, entry_to)
        arr_2 = self.crossing_exit
        binary_insert_q(elem_2, arr_2, func=func)

    def update_intersection(self):
        # Getting the ids to remove
        removed_elem = []
        id_removal = set()
        for elem in self.crossing_exit:
            if elem[1] <= stgs.time:
                removed_elem.append(elem)
            else:
                break

        # Remove in crossing_exit
        for i in removed_elem:
            self.crossing_exit.pop(0)
            id_removal.add(i[0])

        # Remove in crossing_enter
        i = 0
        while len(id_removal) > 0:
            ID = self.crossing_enter[i][0]
            if ID in id_removal:
                id_removal.remove(ID)
                self.crossing_enter.pop(i)
            else:
                i += 1

        # Remove in car_entry
        for ID, time, entry, entry_to in removed_elem:
            arr = self.entries[entry]
            for i in arr:
                if i[0] == ID:
                    arr.remove(i)
                    break


class Road:
    def __init__(self, node1, node2):
        self.enter = []
        self.estimation = []
        self.leave = []
        # self.line = [(None, 0, 0)]  # (id_last_car, length, Time)
        # self.visited = set()

        self.max_capacity = trf.Road.max_capacity(node1, node2)
        self.curr_capacity = 0
        self.start = node1
        self.to = node2

    def add_car_enter(self, ID, time, dist):
        elem = (ID, time, dist)
        arr = self.enter
        func = lambda x: x[1]

        binary_insertion(elem, arr, func)
        self.curr_capacity += 1

    def add_car_junc_estimation(self, ID, time, dist):
        timing = Road.estimate_arrive(time, dist)
        elem = (ID, timing)
        arr = self.estimation
        func = lambda x: x[1]

        binary_insertion(elem, arr, func)

    def add_car_exit(self, ID, time):
        elem = (ID, time)
        arr = self.leave
        func = lambda x: x[1]

        binary_insertion(elem, arr, func)
        self.curr_capacity -= 1

        # self.visited.remove(ID)

    def add_line(self, id_last_car, time):
        self.line.append(id_last_car, time, self.line[-1][2] + 1)

    def delete_line(self, time):
        self.line.append(self.line[-1][0], time, self.line[-1][2] - 1)

    def update_road(self):
        # Getting the ids to remove
        id_removal = set()
        id_removal_2 = set()
        id_removal_3 = set()

        for elem in self.leave:
            if elem[1] <= stgs.time:
                id_removal.add(elem[0])
                id_removal_2.add(elem[0])
                id_removal_3.add(elem[0])
            else:
                break

        # Remove in leave
        for i in id_removal:
            self.leave.pop(0)

        # Remove in enter
        i = 0
        while len(id_removal) > 0:
            ID = self.enter[i][0]
            if ID in id_removal:
                id_removal.remove(ID)
                self.enter.pop(i)
            else:
                i += 1

        """
        # Remove in line
        i = 0
        while len(id_removal_2) > 0:
            ID = self.line[i][0]
            if ID in id_removal_2:
                id_removal_2.remove(ID)
                self.line.pop(i)
            else:
                i += 1
        """

        # Estimation
        i = 0
        while len(id_removal_3) > 0:
            ID = self.estimation[i][0]
            if ID in id_removal_3:
                id_removal_3.remove(ID)
                self.estimation.pop(i)
            else:
                i += 1

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
