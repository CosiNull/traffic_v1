import settings as stgs
import traffic as trf
import pathfinding as pf
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

s_dist_turn = {
    "u": 49,
    "l": 22,
    "r": 51,
}


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
        binary_insertion(elem_2, arr_2, func)

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
        self.leave = []
        self.line = [(0, 0)]  # (id_last_car, length,Time)

        self.max_capacity = trf.Road.max_capacity(node1, node2)
        self.start = node1
        self.to = node2

    def add_car_enter(self, ID, time, dist):
        elem = (ID, time, dist)
        arr = self.enter
        func = lambda x: x[1]

        binary_insertion(elem, arr, func)

    def add_car_exit(self, ID, time):
        elem = (ID, time)
        arr = self.leave
        func = lambda x: x[1]

        binary_insertion(elem, arr, func)

    def add_line(self, id_last_car, time, length):
        self.line.append(id_last_car, time, length)

    def update_road(self):
        # Getting the ids to remove
        id_removal = set()
        id_removal_2 = set()

        for elem in self.leave:
            if elem[1] <= stgs.time:
                id_removal.add(elem[0])
                id_removal_2.add(elem[0])
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

        # Remove in line
        i = 0
        while len(id_removal_2) > 0:
            ID = self.line[i][0]
            if ID in id_removal_2:
                id_removal_2.remove(ID)
                self.line.pop(i)
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


# _____________________________________________________________________________________________________
def add_car_path(ID, pos, action, timing, intersection):
    paths[ID].append((pos, action, intersection))
    timing_paths[ID].append(timing)


# e: enter road
# r, l, u: intersection crossing
# i: arrive intersection


def reset_path(ID):
    paths[ID] = []
    timing_paths[ID] = []


paths = [[] for i in range(stgs.num_car)]
# path (node/turn)
timing_paths = [[] for i in range(stgs.num_car)]
# path timing

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


junctions = make_intersection_dict(trf.road_network)
roads = make_road_dict(trf.road_network)
# Path predicting__________________________________________________________________________
# Simple
def predict_path(car):
    current_dirs = ["" for i in range(stgs.num_car)]
    current_dirs[car.id] = pf.angle_to_dir[car.angle]

    pred = predict_road_entry(current_dirs[car.id], car)
    add_car_path(*pred)

    # In future, no use for loop
    for count, action in enumerate(car.path[0:-1]):
        if not type(action) == str:
            pred = predict_intersection(
                action,
                paths[car.id][count][0],
                current_dirs[car.id],
                timing_paths[car.id][count],
                car,
            )
            add_car_path(*pred)

        else:
            pred = predict_turn(
                paths[car.id][count][2],
                trf.inverse_dir[current_dirs[car.id]],
                action,
                timing_paths[car.id][count],
                car,
            )
            current_dirs[car.id] = trf.abs_dir(current_dirs[car.id], action)
            add_car_path(*pred)

    pred = predict_park(
        car.target_pos,
        paths[car.id][-1][0],
        current_dirs[car.id],
        timing_paths[car.id][-1],
        car,
    )
    add_car_path(*pred)


def predict_road_entry(curr_dir, car):
    # Still need to adjust that
    start_time = stgs.time
    # Exit parking code
    u_s = 0 if curr_dir == "l" or curr_dir == "r" else 1
    road_angle = 1 if curr_dir == "d" or curr_dir == "r" else -1

    next_pos = list(car.init_pos)
    next_pos[u_s] = round(next_pos[u_s] + stgs.park_dist * road_angle)

    s_s = 1 if u_s == 0 else 0
    road_ang_s = 1 if curr_dir == "r" or curr_dir == "u" else -1

    # Use of start nodes here NOT GOOD FOR NOW
    next_pos[s_s] = car.init_nodes[1][s_s] + stgs.road_center * road_ang_s

    next_pos = tuple(next_pos)

    time_delay = 2
    time_finish = start_time + stgs.park_time + time_delay

    return (car.id, next_pos, "e", time_finish, car.init_nodes[1])


def predict_intersection(
    intersection,
    pos,
    curr_dir,
    time,
    car,
):
    # If road is empty
    u_s = 0 if curr_dir == "r" or curr_dir == "l" else 1
    road_ang_u = 1 if curr_dir == "l" or curr_dir == "u" else -1

    dist_to_travel = (
        abs(intersection[u_s] - pos[u_s]) - stgs.node_width / 2 - stgs.car_len / 2
    )
    time_arrive = Road.estimate_arrive(time, dist_to_travel)

    s_s = 1 if u_s == 0 else 0
    road_ang_s = 1 if curr_dir == "r" or curr_dir == "u" else -1

    next_pos = list(pos)
    next_pos[u_s] = (
        intersection[u_s] + (stgs.node_width / 2 + stgs.car_len / 2) * road_ang_u
    )
    next_pos[s_s] = intersection[s_s] + stgs.road_center * road_ang_s

    return (car.id, tuple(next_pos), "i", time_arrive, intersection)


def predict_turn(intersection, entry, turn, time, car):
    # If it is free
    extra_time = time_turn[turn]
    wait_delay = 1
    time_arrive = time + extra_time + wait_delay

    new_dir = trf.abs_dir(trf.inverse_dir[entry], turn)
    u_s = 0 if new_dir == "r" or new_dir == "l" else 1
    road_ang_u = 1 if new_dir == "r" or new_dir == "d" else -1

    s_s = 1 if u_s == 0 else 0
    road_ang_s = 1 if new_dir == "r" or new_dir == "u" else -1

    pos = list(intersection)
    pos[s_s] = intersection[s_s] + stgs.road_center * road_ang_s
    pos[u_s] = intersection[u_s] + (stgs.node_width / 2 + stgs.car_len / 2) * road_ang_u

    return (car.id, tuple(pos), turn, time_arrive, intersection)


def predict_park(goal, pos, curr_dir, time, car):
    # If there is no line of course
    u_s = 0 if curr_dir == "l" or curr_dir == "r" else 1
    dist = abs(goal[u_s] - pos[u_s]) - stgs.park_dist

    park_delay = 32
    time_delay = 1
    time_extra = Road.estimate_arrive(time, dist)

    time_arrive = time_extra + time_delay + park_delay

    return (car.id, goal, "p", time_arrive, car.predicted_nodes[1])
