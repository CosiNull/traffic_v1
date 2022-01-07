import settings as stgs
import traffic as trf
import math
from future2 import *


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

    start_times[ID] = stgs.time + 2


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
            pred = predict_road_entry(ID, start_times[ID], preds)
            binary_insert_q(pred, queue)

            preds[ID] = pred[2]

    # The Great Loop
    while len(queue) > 0:
        ID, pos, time = queue.pop(0)
        junction, action = true_paths[ID][len(paths[ID])]

        if action == "e":
            pred = predict_road_entry(ID, start_times[ID], preds, next_pos=pos)

            if pred[2] == time:
                add_car_path(ID, pos, time + 32)
                pred = predict_intersection(ID)
                binary_insert_q(pred, queue)
                preds[ID] = pred[2]

                # Register to road enter
                road = get_entry_road(true_paths[ID][1][0], dir_paths[ID][1])
                dist = road.get_car_dist(pos) - junction_space
                # road.add_car_enter(ID, time + 32, dist)
                road.add_car_junc_estimation(ID, time + 32, dist)
            else:
                binary_insert_q(pred, queue)

                preds[ID] = pred[2]

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

            else:
                binary_insert_q(car_in_front, queue)

                preds[ID] = car_in_front[2]

        elif action == "l" or action == "r" or action == "u":
            crossable_pred = predict_junc_crossable(ID, time, preds)
            if crossable_pred[2] == time:  # Let's cross
                # Adding to cross
                intersect = junctions[junction]
                entry = trf.inverse_dir[dir_paths[ID][len(paths[ID]) - 1]]
                intersect.add_car_entry(ID, entry, time)

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
                # next_road.add_car_enter(ID, time, dist)
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


def get_road_cars_pos(ID, time, preds):
    init_pos = true_paths[ID][0][0]
    road = get_entry_road(true_paths[ID][1][0], dir_paths[ID][1])
    tot_car_len = stgs.car_len + stgs.min_dist

    # 1. GET THE CARS BY ESTIMATION ORDER
    car_order = []
    j = 0
    base = -1
    i = 0
    while j < road.curr_capacity:
        i = base - j
        ID_b, estimation = road.estimation[i]
        if len(paths[ID_b]) == len(true_paths[ID_b]):
            base -= 1
            continue
        car_order.append((ID_b, estimation))
        j += 1
    car_order.reverse()

    # 2. See when next car departs to get extra tiny line
    extra_dist = 0
    while abs(i) < len(road.estimation):
        i -= 1
        ID_b = road.estimation[i][0]
        count_b = len(paths[ID_b])
        if count_b == len(true_paths[ID_b]):
            continue
        past_junc, past_act = true_paths[ID_b][count_b - 1]
        if past_junc == road.to and past_act != "i":
            timing_b = timing_paths[ID_b][count_b - 1] - time_turn[past_act] - 1
            time_elapsed = time - timing_b
            ID_ex = ID_b

            extra_dist = reLu(tot_car_len - time_elapsed * stgs.car_speed)
        break

    # ____________________________________________

    # THE HOLY LOOP
    line_len = stgs.node_width / 2 + extra_dist
    liners = []
    flowing = []

    for ID_b, estimation in car_order:
        count = len(paths[ID_b])
        action = true_paths[ID_b][count][1]
        if preds[ID_b] != estimation and action != "p":
            # Add to line
            liners.append(ID_b)
            line_len += stgs.min_dist + stgs.car_len
        elif action != "p":
            dist = road.get_car_dist(paths[ID_b][-1])
            time_elapsed = time - timing_paths[ID_b][-1]
            dist -= time_elapsed * stgs.car_speed
            if dist <= line_len:
                # Add to line
                liners.append(ID_b)
                line_len += stgs.min_dist + stgs.car_len
            else:
                # Just add in flowing
                flowing.append((ID_b, dist, None))
        else:
            dist = road.get_car_dist(paths[ID_b][-1])
            time_elapsed = time - timing_paths[ID_b][-1]
            dist -= time_elapsed * stgs.car_speed
            goal_dist = road.get_car_dist(true_paths[ID_b][-1][0]) + stgs.park_dist
            if line_len > goal_dist:
                liners.append(ID_b)
                line_len += stgs.min_dist + stgs.car_len
            elif dist > goal_dist:
                flowing.append((ID_b, dist, goal_dist))

    # NOW CHECK LINERS
    my_dist = road.get_car_dist(init_pos)
    boundary = my_dist - 10
    if line_len > boundary:
        # Special case scenario
        diff_dist = line_len - boundary
        if diff_dist < extra_dist:
            if len(liners) == 0:
                timing_b = timing_b + 1 if ID < ID_ex else timing_b
                return timing_b + 1
            else:
                return time + int(diff_dist / stgs.car_speed)

        # See what length is necessary for car out
        diff_dist -= extra_dist
        c = 0
        while diff_dist > 0:
            diff_dist -= stgs.car_len
            c += 1
            if diff_dist <= 0:
                break
            else:
                diff_dist -= stgs.min_dist
        num_cars = len(liners) - (c - 1)
        almost_final_line = num_cars * tot_car_len - stgs.min_dist + stgs.node_width / 2
        time_to_wait = math.ceil(almost_final_line - boundary) / stgs.car_speed
        if time_to_wait > 21:
            raise Exception("I'm dying inside")
        res = []
        [binary_insertion(elem, res, lambda x: x[1]) for elem in liners]
        return res[c][1] + time_to_wait
    else:
        # Time for the freeflowers
        best = time
        for ID_b, other_dist, goal_dist in flowing:
            # check if it is in correct interval
            collision = entry_collision(other_dist, my_dist)
            if collision:
                time_to_wait = (
                    -(my_dist - 10 - stgs.car_len - other_dist) / stgs.car_speed
                )
                time_to_wait = time_to_wait + 1 if ID < ID_b else time_to_wait
                best = max(best, time + time_to_wait)

        return best


def predict_road_entry(ID, time, preds, next_pos=None):

    timing = get_road_cars_pos(ID, time, preds)

    if next_pos == None:  # Exit parking pos code
        curr_dir = dir_paths[ID][0]
        u_s = 0 if curr_dir in {"l", "r"} else 1
        road_angle = 1 if curr_dir == "d" or curr_dir == "r" else -1

        next_pos = list(true_paths[ID][0][0])
        next_pos[u_s] = round(next_pos[u_s] + stgs.park_dist * road_angle)

        s_s = 1 if u_s == 0 else 0
        road_ang_s = 1 if curr_dir == "r" or curr_dir == "u" else -1

        next_pos[s_s] = true_paths[ID][1][0][s_s] + stgs.road_center * road_ang_s

        next_pos = tuple(next_pos)

    return (ID, next_pos, timing)


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
    u_s = 0 if curr_dir in {"l", "r"} else 1
    dist = abs(goal[u_s] - pos[u_s]) - stgs.park_dist

    time_extra = Road.estimate_arrive(time, dist)

    time_arrive = time_extra  # + time_delay + park_delay

    return (ID, pos, time_arrive)


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

    # FOR THE FRONT PARKERS
    # 1. Check if car in front wants to park by checking its action and if it shares same past junction
    # 2. Make sure is in line
    # 3. Get what time it will be when behind (pred: pos - car_dist)
    # 4. Predict intersection

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

    if action_b == "p" and past_junc == true_paths[ID][len(paths[ID]) - 1][0]:
        time_b = timing_paths[ID_b][-1]
        goal_b = true_paths[ID_b][-1][0]
        pos_b = paths[ID_b][-1]
        curr_dir_b = dir_paths[ID_b][-1]
        u_s = 0 if curr_dir_b in {"l", "r"} else 1
        dist = abs(goal_b[u_s] - pos_b[u_s]) - stgs.park_dist
        if preds[ID_b] > Road.estimate_arrive(time_b, dist):
            dist_b = road.get_car_dist(pos_b) - stgs.node_width / 2
            dist_b += stgs.park_dist + stgs.min_dist + stgs.car_len / 2
            timing = Road.estimate_arrive(preds[ID_b], dist_b)
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

    """
    # Get car just before
    extra_dist = 0
    while abs(i) < len(road.estimation):
        i -= 1
        ID_b = road.estimation[i][0]
        count_b = len(paths[ID_b])
        if count_b == len(true_paths[ID_b]):
            continue
        past_junc, past_act = true_paths[ID_b][count_b - 1]
        if ID == 171:
            print(timing_paths[ID_b], preds[ID_b], ID_b)
        if past_junc == road.to and past_act != "i":
            timing_b = timing_paths[ID_b][count_b - 1] - time_turn[past_act] - 1
            time_elapsed = preds[ID_b] - timing_b
            extra_dist = reLu(tot_car_len - time_elapsed * stgs.car_speed)
            if extra_dist != 0:
                print(ID, time_elapsed, extra_dist)
            if ID == 171:
                print(extra_dist, time_elapsed, past_act)
        break
    """

    """
    to_travel = line_len + extra_dist - goal_dist
    if extra_dist != 0 and to_travel < tot_car_len:
        print(ID)
        timing = timing_b + math.ceil(to_travel / stgs.car_speed)
        timing = timing + 1 if ID < ID_b else timing

    else:
    """
    # Step 3.
    dist_to_travel = line_len - goal_dist
    cars_to_move = math.ceil(dist_to_travel / tot_car_len)
    ID_b, time = res[cars_to_move]
    timing = time + math.ceil(dist_to_travel / stgs.car_speed)
    timing = timing + 1 if ID < ID_b else timing

    return (ID, 0, timing)


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


def get_entry_road(node_to, direc):
    neighbours = trf.road_network.connections[node_to]

    for neighbour, edge in neighbours:
        if trf.entry_dir(neighbour, node_to) == direc:
            return roads[(neighbour, direc)]

    raise Exception("Error in determinating Intersection for road entry")
