from future2 import *
import future as fut


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


def predict_car_in_front(ID, preds, time, pos, paths, timing_paths):
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
        return (ID, pos, timing)

    past_junc, past_act = true_paths[ID_b][count_b - 1]
    if past_junc == junction:
        timing_b = timing_paths[ID_b][count_b - 1] - time_turn[past_act] - 1
        dist = stgs.car_len + stgs.min_dist
        dist_time = int(dist / stgs.car_speed)
        time_diff = abs(time - timing_b)
        if time_diff < dist_time or (time_diff == dist_time and ID < ID_b):
            timing = timing_b + dist_time
            timing = timing + 1 if ID < ID_b else timing

            return (ID, pos, timing)

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
            return (ID, pos, timing)

    return None
