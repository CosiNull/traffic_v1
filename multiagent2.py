from future2 import *
import future as fut
import settings as stgs


def predict_junc_crossable(ID, time, preds, paths, m_e, m_e_t, junc, road):
    timing = time

    # If intersection is not free
    # 1. Check everyone in exit_crossing who exited after you
    # 2. Check from biggest time to smallest
    # 2.1 IF biggest, crossable, go down ELSE Biggest would be new threashold
    # 2.2 When decide threshold CONSIDER ORDER with ID

    ind = binary_search_ds(time, junc.crossing_exit)

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

    if road.curr_capacity == road.max_capacity:
        base = -1
        j = 0

        best = math.inf
        while j < road.max_capacity:
            i = base - j
            ID_b = road.estimation[i][0]
            count_b = len(paths[ID_b])
            if count_b == len(fut.true_paths[ID_b]):
                base -= 1
                continue
            action_b = fut.true_paths[ID_b][count_b][1]
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


def predict_car_in_front(ID, preds, time, pos, paths, timing_paths, road):
    junction = fut.true_paths[ID][len(paths[ID])][0]
    ind = backward_linear_s(ID, road.estimation, step=-1)

    # Making sure that the car before has not parked
    while True:
        if ind == 0:
            return None
        ID_b = road.estimation[ind - 1][0]
        count_b = len(paths[ID_b])
        if count_b == len(fut.true_paths[ID_b]):
            ind -= 1
        else:
            break

    # Calculating the pred of the car
    junction_b, action_b = fut.true_paths[ID_b][count_b]

    if junction_b == junction:
        dist = stgs.car_len + stgs.min_dist
        timing = preds[ID_b] + int(dist / stgs.car_speed)
        timing = timing + 1 if action_b == "i" and ID < ID_b else timing
        timing = (
            timing - 1 if ID > ID_b else timing
        )  # Special technicality with update order
        return (ID, pos, timing)

    past_junc, past_act = fut.true_paths[ID_b][count_b - 1]
    if past_junc == junction:
        timing_b = timing_paths[ID_b][count_b - 1] - time_turn[past_act] - 1
        dist = stgs.car_len + stgs.min_dist
        dist_time = int(dist / stgs.car_speed)
        time_diff = abs(time - timing_b)
        if time_diff < dist_time or (time_diff == dist_time and ID < ID_b):
            timing = timing_b + dist_time
            timing = timing + 1 if ID < ID_b else timing

            return (ID, pos, timing)

    if action_b == "p" and past_junc == fut.true_paths[ID][len(paths[ID]) - 1][0]:
        time_b = timing_paths[ID_b][-1]
        goal_b = fut.true_paths[ID_b][-1][0]
        pos_b = paths[ID_b][-1]
        curr_dir_b = fut.dir_paths[ID_b][-1]
        u_s = 0 if curr_dir_b in {"l", "r"} else 1
        dist = abs(goal_b[u_s] - pos_b[u_s]) - stgs.park_dist
        if preds[ID_b] > Road.estimate_arrive(time_b, dist):
            dist_b = road.get_car_dist(pos_b) - stgs.node_width / 2
            dist_b += stgs.park_dist + stgs.min_dist + stgs.car_len / 2
            timing = Road.estimate_arrive(preds[ID_b], dist_b)
            return (ID, pos, timing)

    return None


def predict_park_line(ID, preds, paths, road):

    # 1. Check the current line
    # 1.1 Check how many cars were before my_car and substract by curr_capacity
    # 2. See if the line exceeds the goal: Do the substraction math
    # 3. Calculate how many cars need to move to attain goal
    # 4. Get N_th earliest

    # Step 1.
    cars_after = 0
    j = 0
    while True:
        i = -1 - j - cars_after
        ID_b = road.estimation[i][0]  # NOTE THERE
        if len(paths[ID_b]) == len(fut.true_paths[ID_b]):
            j += 1
        elif ID_b == ID:
            break
        else:
            cars_after += 1
    cars_before = road.curr_capacity - 1 - cars_after

    # Step 2.
    curr_dir = fut.dir_paths[ID][len(paths[ID])]
    u_s = 0 if curr_dir in {"r", "l"} else 1
    tot_car_len = stgs.min_dist + stgs.car_len
    line_len = cars_before * tot_car_len + stgs.node_width / 2 + stgs.car_len / 2
    goal_dist = abs(road.to[u_s] - fut.true_paths[ID][-1][0][u_s]) + stgs.park_dist

    if goal_dist >= line_len:
        return None
    # Step 4.
    res = []
    examined_cars = 0
    while examined_cars < cars_before:
        i -= 1
        ID_b = road.estimation[i][0]
        if len(paths[ID_b]) == len(fut.true_paths[ID_b]):
            continue
        else:
            binary_insertion((ID_b, preds[ID_b]), res, lambda x: x[1])
            examined_cars += 1

    # Step 3.
    dist_to_travel = line_len - goal_dist
    cars_to_move = math.ceil(dist_to_travel / tot_car_len)
    ID_b, time = res[cars_to_move]
    timing = time + math.ceil(dist_to_travel / stgs.car_speed)
    timing = timing + 1 if ID < ID_b else timing

    return (ID, 4, timing)
