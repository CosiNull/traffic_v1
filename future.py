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
    preds = [None for i in range(stgs.num_car)]

    # Initilialize in simple way
    for ID in range(stgs.num_car):
        if not start_times[ID] == None:
            pred = predict_road_entry(ID)
            binary_insert_q(pred, queue)

            preds[ID] = pred[2]

    # The Great Loop
    while len(queue) > 0:
        ID, pos, time = queue.pop(0)
        junction, action = true_paths[ID][len(paths[ID])]

        if action == "e":
            add_car_path(ID, pos, time)
            pred = predict_intersection(ID)
            binary_insert_q(pred, queue)

            preds[ID] = pred[2]

            # Register to road enter
            road = get_entry_road(true_paths[ID][1][0], dir_paths[ID][1])
            dist = road.get_car_dist(pos) - junction_space
            road.add_car_enter(ID, time, dist)
            road.add_car_junc_estimation(ID, time, dist)

        elif action == "i":
            # If there are no cars still at intersection execute the following
            # Else: Repredict when it will arrive at the intersection
            car_in_front = predict_car_in_front(ID, preds, time)
            # road = get_entry_road(junction, dir_paths[ID][len(paths[ID]) - 1])

            if car_in_front == None:  # if no one is there
                add_car_path(ID, pos, time)
                pred = predict_junc_crossable(ID, time + 1, preds)
                binary_insert_q(pred, queue)

                preds[ID] = pred[2]

                # Adding to datastructure
                intersect = junctions[junction]
                entry = trf.inverse_dir[dir_paths[ID][len(paths[ID]) - 1]]
                intersect.add_car_entry(ID, entry, time)

            else:
                binary_insert_q(car_in_front, queue)

                preds[ID] = car_in_front[2]

        elif action == "l" or action == "r" or action == "u":
            crossable_pred = predict_junc_crossable(ID, time, preds)
            if crossable_pred[2] == time:  # Let's cross
                # Predicting next cross
                finish_cross = predict_turn(ID, time)
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

                if len(paths[ID]) + 1 == len(dir_paths[ID]):
                    pred = predict_park(ID)
                else:
                    pred = predict_intersection(ID)

                binary_insert_q(pred, queue)

                preds[ID] = pred[2]
            else:
                # Repredict
                binary_insert_q(crossable_pred, queue)

                preds[ID] = crossable_pred[2]
        elif action == "p":
            # If the road is clean
            # Else repredict by seeing when other car exits road
            line_park = predict_park_line(ID, preds)

            if line_park == None:
                add_car_path(ID, pos, time + 32 + 1)

                # Register exit
                road = roads[(true_paths[ID][-2][0], dir_paths[ID][-1])]
                road.add_car_exit(ID, time)

                preds[ID] = None
            else:
                binary_insert_q(line_park, queue)

                preds[ID] = line_park[2]


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


def predict_turn(ID, time):
    count = len(paths[ID])

    intersection, turn = true_paths[ID][count]
    entry = trf.inverse_dir[dir_paths[ID][count - 1]]

    # If Intersection is Free
    extra_time = time_turn[turn]
    time_arrive = time + extra_time

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

    time_extra = Road.estimate_arrive(time, dist)

    time_arrive = time_extra  # + time_delay + park_delay

    return (ID, goal, time_arrive)


def predict_junc_crossable(ID, time, preds):
    count = len(paths[ID])
    timing = time

    # If intersection is not free
    # 1. Check everyone in exit_crossing who exited after you
    # 2. Check from biggest time to smallest
    # 2.1 IF biggest, crossable, go down ELSE Biggest would be new threashold
    # 2.2 When decide threshold CONSIDER ORDER with ID

    junc = junctions[true_paths[ID][count][0]]
    ind = binary_search_ds(time, junc.crossing_exit)

    m_e = trf.inverse_dir[dir_paths[ID][len(paths[ID]) - 1]]
    m_e_t = dir_paths[ID][len(paths[ID]) + 1]

    arr = junc.crossing_exit[ind:]
    arr.reverse()

    for ID_2, time_, entry, entry_to in arr:
        if not (entry, entry_to) in trf.no_conflicts[(m_e, m_e_t)]:
            timing = time_ if ID > ID_2 else time_ + 1
            break

    # 1. Get road
    # 2. Get capacity
    # 3. If too full: get free flowing cars, or first in line
    # 4. Get the soonest exit
    road = roads[(true_paths[ID][count][0], m_e_t)]

    if road.curr_capacity == road.max_capacity:
        base = -1
        i = base
        j = 0

        best = math.inf
        while j < road.max_capacity:
            i = base - j
            ID_b = road.estimation[i][0]
            count_b = len(paths[ID_b])
            if count_b == len(true_paths[ID_b]):
                base -= 1
                continue
            action_b = true_paths[ID_b][count_b][1]
            if j == road.max_capacity - 1:
                if preds[ID_b] < best:
                    best = preds[ID_b]
                    best_ID = ID_b
                best = best + 1 if action_b == "i" else best
                break
            elif action_b == "p":
                if preds[ID_b] < best:
                    best = preds[ID_b]
                    best_ID = ID_b

            j += 1
        best = best if ID > best_ID else best + 1
        timing = max(timing, best)

    return (ID, 0, timing)


def predict_car_in_front(ID, preds, time):
    # NOTE: Does not take into account when car before is at park phase

    # 1. Check who has estimate just before you
    # 1.1 Linear search backwards
    # 2. Get pred of car before
    # 2.1 Make sure car_before is still in road by checking if its action is:
    # 2.2: r,l,u on its junction, i on its junction, or p
    # 2.3 Else return original pred
    # 3.1 If car before is first: Return pred_leave + (dist_car + min_dist) * speed
    # 3.2 Else car before is not first: Return same thing but + 1

    junction = true_paths[ID][len(paths[ID])][0]
    road = get_entry_road(junction, dir_paths[ID][len(paths[ID])])
    ind = backward_linear_s(ID, road.estimation, step=-1)

    # Making sure that the car before has not parked
    while True:
        if ind == 0:
            return None
        ID_b = road.estimation[ind - 1][0]
        count_b = len(paths[ID_b])
        if count_b == len(true_paths[ID_b]):
            ind -= 1
        else:
            break

    # Calculating the pred of the car
    junction_b, action_b = true_paths[ID_b][count_b]

    if junction_b == junction:
        dist = stgs.car_len + stgs.min_dist
        timing = preds[ID_b] + int(dist / stgs.car_speed)
        timing = timing + 1 if action_b == "i" and ID < ID_b else timing
        timing = (
            timing - 1 if ID > ID_b else timing
        )  # Special technicality with update order
        return (ID, 0, timing)
    else:
        past_junc, past_act = true_paths[ID_b][count_b - 1]
        if past_junc == junction:
            timing_b = timing_paths[ID_b][count_b - 1] - time_turn[past_act] - 1
            dist = stgs.car_len + stgs.min_dist
            dist_time = int(dist / stgs.car_speed)
            time_diff = abs(time - timing_b)
            if time_diff < dist_time or (time_diff == dist_time and ID < ID_b):
                timing = timing_b + dist_time
                timing = timing + 1 if ID < ID_b else timing

                return (ID, 0, timing)

    return None


def predict_park_line(ID, preds):

    # 1. Check the current line
    # 1.1 Check how many cars were before my_car and substract by curr_capacity
    # 2. See if the line exceeds the goal: Do the substraction math
    # 3. Calculate how many cars need to move to attain goal
    # 4. Get N_th earliest

    junction = true_paths[ID][len(paths[ID]) - 1][0]
    road = roads[(junction, dir_paths[ID][len(paths[ID])])]

    # Step 1.
    cars_after = 0
    j = 0
    while True:
        i = -1 - j - cars_after
        ID_b = road.estimation[i][0]
        if len(paths[ID_b]) == len(true_paths[ID_b]):
            j += 1
        elif ID_b == ID:
            break
        else:
            cars_after += 1
    cars_before = road.curr_capacity - 1 - cars_after

    # Step 2.
    curr_dir = dir_paths[ID][len(paths[ID])]
    u_s = 0 if curr_dir in {"r", "l"} else 1
    tot_car_len = stgs.min_dist + stgs.car_len
    line_len = cars_before * tot_car_len + stgs.node_width / 2 + stgs.car_len / 2
    goal_dist = abs(road.to[u_s] - true_paths[ID][-1][0][u_s]) + stgs.park_dist

    if goal_dist >= line_len:
        return None

    # Step 3.
    dist_to_travel = line_len - goal_dist
    cars_to_move = math.ceil(dist_to_travel / tot_car_len)

    # Step 4.
    res = []
    examined_cars = 0
    while examined_cars < cars_before:
        i -= 1
        ID_b = road.estimation[i][0]
        if len(paths[ID_b]) == len(true_paths[ID_b]):
            continue
        else:
            binary_insertion((ID_b, preds[ID_b]), res, lambda x: x[1])
            examined_cars += 1

    ID_b, time = res[cars_to_move]
    timing = time + math.ceil(dist_to_travel / stgs.car_speed)
    timing = timing + 1 if ID < ID_b else timing
    return (ID, 0, timing)


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


def backward_linear_s(elem, arr, func=lambda x: x[0], step=-1):
    for i in range(len(arr) - 1, -1, step):
        if func(arr[i]) == elem:
            return i
