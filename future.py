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
        self.estimation = []
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

    def add_line(self, id_last_car, time, length):
        self.line.append(id_last_car, time, length)

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


junctions = make_intersection_dict(trf.road_network)
roads = make_road_dict(trf.road_network)
# _____________________________________________________________________________________________________
def add_car_path(ID, pos, timing):
    paths[ID].append((pos))
    timing_paths[ID].append(timing)


def reset_path(ID):
    paths[ID] = []
    timing_paths[ID] = []
    true_paths[ID] = []
    dir_paths[ID] = []
    start_times[ID] = None


def save_true_path(ID, car_path, init_pos, final_pos, direc):
    path = [(init_pos, "e")]
    previous = car_path[0]
    #
    curr_dir = direc
    d_path = [curr_dir]

    for action in car_path[0:-1]:
        if not type(action) == str:
            path.append((action, "i"))
            previous = action
            #
            d_path.append(curr_dir)
        else:
            path.append((previous, action))
            #
            d_path.append("t")
            curr_dir = trf.abs_dir(curr_dir, action)

    path.append((final_pos, "p"))
    #
    d_path.append(curr_dir)

    true_paths[ID] = path
    dir_paths[ID] = d_path

    start_times[ID] = stgs.time


# e: enter road
# r, l, u: intersection crossing
# i: arrive intersection
# p: park

dir_paths = [[] for i in range(stgs.num_car)]
#
true_paths = [[] for i in range(stgs.num_car)]
# path (intersection/action)
paths = [[] for i in range(stgs.num_car)]
# path (true_pos)
timing_paths = [[] for i in range(stgs.num_car)]
# path timing
start_times = [None for i in range(stgs.num_car)]

# Path predicting__________________________________________________________________________
def predict_path():
    queue = []
    reset_all_datastructures()

    # Initilialize in simple way
    for ID in range(stgs.num_car):
        if not start_times[ID] == None:
            pred = predict_road_entry(ID)
            binary_insert_q(pred, queue)

    # The Great Loop
    while len(queue) > 0:
        ID, pos, time = queue.pop(0)
        junction, action = true_paths[ID][len(paths[ID])]

        if action == "e":
            add_car_path(ID, pos, time)
            pred = predict_intersection(ID)
            binary_insert_q(pred, queue)

            # Register to road enter
            road = get_entry_road(true_paths[ID][1][0], dir_paths[ID][1])
            dist = road.get_car_dist(pos) - junction_space
            road.add_car_enter(ID, time, dist)
            road.add_car_junc_estimation(ID, time, dist)

        elif action == "i":
            # If there are no cars still at intersection execute the following
            """
            1. Check car in front
            2. Check if exited
            3. If not well readjust timing to his exit + distance
            """
            # Else: Repredict when it will arrive at the intersection

            add_car_path(ID, pos, time)
            pred = predict_junc_crossable(ID, time + 1)
            binary_insert_q(pred, queue)

            # Adding to datastructure if no one is there
            intersect = junctions[junction]
            entry = trf.inverse_dir[dir_paths[ID][len(paths[ID]) - 1]]
            intersect.add_car_entry(ID, entry, time)

        elif action == "l" or action == "r" or action == "u":
            crossable_pred = predict_junc_crossable(ID, time)
            if crossable_pred[2] == time:  # Let's cross
                # Predicting next cross
                finish_cross = predict_turn(ID)
                add_car_path(*finish_cross)

                # Register road Exit
                road = get_entry_road(junction, dir_paths[ID][len(paths[ID]) - 2])
                road.add_car_exit(ID, time)

                # Register crossing
                intersect = junctions[junction]
                entry = trf.inverse_dir[dir_paths[ID][len(paths[ID]) - 2]]
                entry_to = dir_paths[ID][len(paths[ID])]
                intersect.add_car_crossing(ID, time, entry, entry_to)

                # Add to next road
                next_road = roads[(junction, dir_paths[ID][len(paths[ID])])]
                time_cross = time_turn[action] + 1  # 1 is time delay
                extra_dist = time_cross * stgs.car_speed - junction_space
                dist = next_road.get_car_dist(finish_cross[1]) + extra_dist
                next_road.add_car_enter(ID, time, dist)
                next_road.add_car_junc_estimation(ID, time, dist - stgs.car_speed)

                pred = 0
                if len(paths[ID]) + 1 == len(dir_paths[ID]):
                    pred = predict_park(ID)
                else:
                    pred = predict_intersection(ID)

                binary_insert_q(pred, queue)
            else:
                # Repredict
                binary_insert_q(crossable_pred, queue)
        elif action == "p":
            # If the road is clean
            # Else repredict by seeing when other car exits road
            add_car_path(ID, pos, time)

            # Register exit
            road = roads[(true_paths[ID][-2][0], dir_paths[ID][-1])]
            road.add_car_exit(ID, time)


def predict_road_entry(ID):
    curr_dir = dir_paths[ID][0]
    # Still need to adjust that
    start_time = start_times[ID]
    # Exit parking code
    u_s = 0 if curr_dir == "l" or curr_dir == "r" else 1
    road_angle = 1 if curr_dir == "d" or curr_dir == "r" else -1

    next_pos = list(true_paths[ID][0][0])
    next_pos[u_s] = round(next_pos[u_s] + stgs.park_dist * road_angle)

    s_s = 1 if u_s == 0 else 0
    road_ang_s = 1 if curr_dir == "r" or curr_dir == "u" else -1

    # Use of start nodes here NOT GOOD FOR NOW
    next_pos[s_s] = true_paths[ID][1][0][s_s] + stgs.road_center * road_ang_s

    next_pos = tuple(next_pos)

    time_delay = 2
    time_finish = start_time + stgs.park_time + time_delay

    return (ID, next_pos, time_finish)


def predict_intersection(ID):
    count = len(paths[ID])

    intersection = true_paths[ID][count][0]
    curr_dir = dir_paths[ID][count]
    pos = paths[ID][count - 1]
    time = timing_paths[ID][count - 1]

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

    return (ID, tuple(next_pos), time_arrive)


def predict_turn(ID):
    count = len(paths[ID])

    intersection, turn = true_paths[ID][count]
    entry = trf.inverse_dir[dir_paths[ID][count - 1]]
    time = timing_paths[ID][count - 1]

    # If Intersection is Free
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

    return (ID, tuple(pos), time_arrive)


def predict_park(ID):
    goal = true_paths[ID][-1][0]
    pos = paths[ID][-1]
    curr_dir = dir_paths[ID][-1]
    time = timing_paths[ID][-1]

    # If there is no line of course
    u_s = 0 if curr_dir == "l" or curr_dir == "r" else 1
    dist = abs(goal[u_s] - pos[u_s]) - stgs.park_dist

    park_delay = 32
    time_delay = 1
    time_extra = Road.estimate_arrive(time, dist)

    time_arrive = time_extra + time_delay + park_delay

    return (ID, goal, time_arrive)


def predict_junc_crossable(ID, time):
    count = len(paths[ID])
    timing = time

    # If intersection is not free
    # 1. Check everyone in exit_crossing who exited after you
    # 2. Check from biggest time to smallest
    # 2.1 IF biggest, crossable, go down ELSE Biggest would be new threashold
    # 2.2 When decide threshold CONSIDER ORDER with ID

    # NOTE We are not taking into account if next road is full in future

    junc = junctions[true_paths[ID][count][0]]
    ind = binary_search_ds(time, junc.crossing_exit)

    m_e = trf.inverse_dir[dir_paths[ID][len(paths[ID]) - 1]]
    m_e_t = dir_paths[ID][len(paths[ID]) + 1]

    arr = junc.crossing_exit[ind:]
    arr.reverse()

    for ID_2, time_, entry, entry_to in arr:
        if not (entry, entry_to) in trf.no_conflicts[(m_e, m_e_t)]:
            timing = time_ if ID > ID_2 else time_ + 1

    return (ID, 0, timing)


# ____________________________________
def binary_search_q(elem, arr):
    func = lambda x: x[2]
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


def binary_insert_q(elem, arr):
    arr.insert(binary_search_q(elem, arr), elem)


# _____________________________________
def get_entry_road(node_to, direc):
    neighbours = trf.road_network.connections[node_to]

    for neighbour, edge in neighbours:
        if trf.entry_dir(neighbour, node_to) == direc:
            return roads[(neighbour, direc)]

    raise Exception("Error in determinating Intersection for road entry")


def reset_all_datastructures():
    global roads
    global junctions
    # Reset paths
    for ID in range(stgs.num_car):
        paths[ID] = []
        timing_paths[ID] = []
    # Reset Road
    roads = make_road_dict(trf.road_network)
    junctions = make_intersection_dict(trf.road_network)


# _____________
def linear_search(elem, arr, func=lambda x: x[0], step=1):
    for i in range(0, len(arr), step):
        if func(arr[i]) == elem:
            return i
