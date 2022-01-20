import parking as pk
from future2 import *
import settings as stgs
import future as fut
import math
import traffic as trf
from pathfinding import *
from multiagent2 import *


# ___The code
def multiagent_a_star(graph, start, end, start_dir, true_goal, time, ID):
    timing = {node: math.inf for node in graph.nodes}
    prev = {}

    timing[start] = time
    pq = Priority_Queue()
    pq.insert((start, 0))

    while len(pq()) > 0:
        node = pq.poll()[0]
        if node == end:
            return (timing, prev)

        for neighbour, edge in graph.connections[node]:
            if node in prev and prev[node] == neighbour:
                continue
            elif node == start and opposite_dir[start_dir] == get_abs_direction(
                node, neighbour
            ):
                continue
            elif node == true_goal and neighbour == end:  # No U_turns towards the goal
                continue

            # This is the thing that we want to modify
            if node in prev:
                entry_from = trf.entry_dir(node, prev[node])
            else:
                entry_from = opposite_dir[start_dir]

            entry_to = trf.entry_dir(node, neighbour)
            path_so_far = reverse_path_multi(prev, start, node)

            extra_time = (
                predict_time_cross(
                    ID, timing[node] + 1, node, entry_from, entry_to, path_so_far
                )
                + 1
            )
            current_time = extra_time
            # ___________________________________________

            if current_time < timing[neighbour]:
                timing[neighbour] = current_time
                pq.insert((neighbour, current_time))
                prev[neighbour] = node

    return (None, None)


def reverse_path_multi(previous, start, end):
    current_node = end
    path = [end]

    while current_node != start:
        current_node = previous[current_node]
        path.insert(0, current_node)
        path.insert(0, current_node)

    return path


def pathfind_mlt(graph, start, end, start_dir, true_goal, time, ID):
    return reverse_path(
        *multiagent_a_star(graph, start, end, start_dir, true_goal, time, ID),
        start,
        end
    )


# Predict time cross
def predict_time_cross(ID, time, junc, entry_from, entry_to, path_so_far):
    # _________Attempt 2 -The isolated queue solution ---Less flawed approach
    # Check the arrivals ~52 frames before curr_time
    # Check the everyone inside all roads surrounding this intersection at this time
    # Check the guys that want to enter in these roads
    # Create the special Intersection and Roads Datastructures

    # Predict their crossing by who arrive first
    # Predict the leave of all guys in the next roads (All should be correct)
    # Add all in queue
    # ____________________________________
    # Do the queue
    # Everyone that arrives after in the guys road is in: Get to arrive after

    # Variable prep
    queue = []
    preds = {}
    true_junction = fut.junctions[junc]
    entries = list(true_junction.entries.keys())

    paths = [[] for i in range(stgs.num_car)]
    timing_paths = [[] for i in range(stgs.num_car)]

    true_roads = {entry: fut.roads[(junc, entry)] for entry in entries}
    new_roads = {entry: fut.Road(junc, true_roads[entry].to) for entry in entries}
    last_left = {entry_to: time for entry_to in entries}

    new_junction = fut.Intersection(entries)

    # 0. Add the main car
    my_elem = (ID, "t", time, entry_from, entry_to)
    binary_insert_q(my_elem, queue)

    preds[ID] = time

    # Not real path
    paths[ID] = path_so_far

    # 1. Get the arrivals
    arrivals_ind = {}
    banned_ids = set([ID])

    for entry in entries:
        arr = true_junction.entry_cross[entry]
        index = binary_search_ds(time - 52, arr)

        if index < len(arr):
            stuff = arr[index]
            id = stuff[0]
        else:
            continue

        if not entry in arrivals_ind and entry != entry_from:
            elem = (id, "t", stuff[1], entry, stuff[2])
            # ID, action, time, entry_from, entry_to
            binary_insert_q(elem, queue)
            preds[stuff[0]] = stuff[1]
            arrivals_ind[entry] = index

            ind = backward_linear_s(junc, fut.true_paths[id])
            paths[id] = fut.paths[id][0:ind]
            timing_paths[id] = fut.timing_paths[id][0:ind]

            last_left[stuff[2]] = min(stuff[1], last_left[stuff[2]])

            banned_ids.add(id)

    # Get the guy crossing just before
    arr = true_junction.entries[entry_from]
    index = binary_search_ds(time - 1, arr)

    while True:
        if -1 < index < len(arr):
            if arr[index][0] == ID:
                index -= 1
            id = arr[index][0]
            time_leave = true_junction.entry_cross[entry_from][index][1]
        else:
            break

        if index < len(arr) and arr[index][1] < time and time - time_leave <= 52:
            ind = backward_linear_s(junc, fut.true_paths[id])
            paths[id] = fut.paths[id][0:ind]
            timing_paths[id] = fut.timing_paths[id][0:ind]

            next_entry = fut.dir_paths[id][ind + 1]

            elem = (id, "t", time_leave, entry_from, next_entry)
            binary_insert_q(elem, queue)

            banned_ids.add(id)

            index -= 1

            last_left[next_entry] = min(elem[2], last_left[next_entry])
        else:
            break

    # 2. Get the roads
    for entry, road in true_roads.items():
        ind = binary_search_ds(last_left[entry] - 1, road.timely_capacity)

        if ind >= len(road.timely_capacity) or entry == entry_from:
            continue

        capacity, timing, type = road.timely_capacity[ind]

        ind -= 1
        if ind >= 0:
            capacity = road.timely_capacity[ind][0]
        else:
            capacity = road.timely_capacity[0][0] - 1

        index = binary_search_ds(last_left[entry] - 1, road.leave)

        """
        if ID == 142 and stgs.time == 4695 and entry == "r":
            print(capacity)
            print(road.leave, index, last_left["r"])
        """

        c = 1
        while c <= capacity:
            i = index + c - 1

            if i >= len(road.leave):
                break

            stuff = road.leave[i]

            # Get estimation
            id = stuff[0]

            pind = backward_linear_s(junc, fut.true_paths[id])
            if pind == None:
                pind = 0

            if id in banned_ids:
                c += 1
                continue

            pos = fut.paths[id][pind]
            timing = fut.timing_paths[id][pind]

            dist = road.get_car_dist(pos) - fut.junction_space

            new_roads[entry].add_car_junc_estimation(id, timing, dist)

            action = "r"
            if pind == len(fut.true_paths[id]) - 3:
                action = "p"

            elem = (id, action, stuff[1] - 1, junc, road.to)
            binary_insert_q(elem, queue)
            preds[stuff[0]] = stuff[1] - 1
            # ID, action, time, junction_start, junction_to

            paths[id] = fut.paths[id][0 : pind + 1]
            timing_paths[id] = fut.timing_paths[id][0 : pind + 1]

            c += 1

    # Get parkers
    """
    for road in new_roads.values():
        edge = (road.start, road.to)
        if not edge in pk.parking.data:
            continue
        arr = pk.parking.data

        for pos, car_length, id in arr:
            time_left = fut.timing_paths[id][0]
            if time_left > time-52:
    """

    waiters = set()

    while len(queue) > 0:
        id, action, timing, start, to = queue.pop(0)

        if action == "t":
            can_cross = predict_junc_crossable(
                id, timing, preds, paths, start, to, new_junction, new_roads[to]
            )

            if can_cross[2] == timing:
                if id == ID:
                    return predict_road_arrive(
                        ID,
                        timing,
                        entry_from,
                        entry_to,
                        new_roads[entry_to],
                        preds,
                        waiters,
                        paths,
                        timing_paths,
                    )

                # Register to crossing
                new_junction.add_car_crossing(id, timing, start, to)

                # Add to next road
                finish_cross = fut.predict_turn(id, timing, paths)

                dist = new_roads[to].get_car_dist(finish_cross[1]) - fut.junction_space
                new_roads[to].add_car_junc_estimation(id, finish_cross[2], dist)

                # Addding to paths
                paths[id].append(finish_cross[1])
                timing_paths[id].append(finish_cross[2])

                if len(paths[id]) + 1 == len(fut.dir_paths[id]):
                    next_action = "p"
                else:
                    next_action = "r"

                # Special shit
                if finish_cross[2] == fut.timing_paths[id][len(paths[id]) - 1]:
                    if next_action == "p":
                        pred = fut.timing_paths[id][-1] - 32
                    else:
                        turn = fut.true_paths[id][len(paths[id]) + 1][1]
                        time_turn = fut.time_turn[turn] + 1
                        pred = fut.timing_paths[id][len(paths[id]) + 1] - time_turn
                else:
                    if next_action == "p":
                        pred = fut.predict_park(id, paths, timing_paths)[2]
                    else:
                        pred = fut.predict_intersection(id, paths, timing_paths)[2]
                elem = (id, next_action, pred, junc, new_roads[to].to)

                binary_insert_q(elem, queue)
                preds[id] = pred

                # Predict next guy try to cross
                if start == entry_from:
                    continue

                arrivals_ind[start] += 1
                arr = true_junction.entry_cross[start]

                if arrivals_ind[start] < len(arr):
                    id_b, timing_b, to_b = arr[arrivals_ind[start]]
                    timing_b -= 1
                    if timing_b < timing - stgs.car_dist / stgs.car_speed:
                        timing_b = timing + stgs.car_dist / stgs.car_speed

                    ind = backward_linear_s(junc, fut.true_paths[id_b])
                    paths[id_b] = fut.paths[id][0:ind]
                    timing_paths[id_b] = fut.timing_paths[id][0:ind]

                    elem = (id_b, "t", timing_b + 1, start, to_b)
                    binary_insert_q(elem, queue)

                    preds[id_b] = elem[2]
            else:
                elem = (id, "t", can_cross[2], start, to)
                binary_insert_q(elem, queue)
                preds[id] = can_cross[2]

        elif action == "r":
            dir = get_abs_direction(start, to)
            road = new_roads[dir]
            line = predict_car_in_front(
                id, preds, timing, paths[id][-1], paths, timing_paths, road
            )

            if line == None or line[2] == timing:
                elem = (id, "c", timing + 1, start, to)
                binary_insert_q(elem, queue)
                preds[id] = timing + 1
                waiters.add(id)

                paths[id].append(to)
                timing_paths[id].append(timing)

            else:
                elem = (id, "r", line[2], start, to)
                binary_insert_q(elem, queue)
                preds[id] = line[2]

        elif action == "c":
            dir = get_abs_direction(start, to)
            road = new_roads[dir]

            paths[id].append(to)
            timing_paths[id].append(timing)

            # Just empty the road
            road.add_car_exit(id, timing)

        elif action == "p":
            # Get the car line
            # If works, empty the road
            dir = get_abs_direction(start, to)
            road = new_roads[dir]
            pos = paths[id][-1]
            park_line = predict_park_line(id, preds, paths, road, dir)

            if park_line == None:
                road.add_car_exit(id, timing)
                paths[id] = fut.paths[id]
                timing_paths[id] = fut.timing_paths[id]
                preds[id] = None
                # print(id, "intersect")
            else:
                elem = (id, "p", park_line[2], start, to)
                binary_insert_q(elem, queue)
                preds[id] = park_line[2]


def predict_road_arrive(
    ID, time, entry_from, entry_to, road, preds, waiting, paths, timing_paths
):
    # Get time cross and next pos
    # Add to estimations
    # Take everyone inside that hasnt left yet
    # Add to queue with their respective preds
    # Go till no one in front
    # return the time
    # __________________________________________
    current_dir = trf.inverse_dir[entry_from]
    turn = relative_dir[current_dir][entry_to]
    time_turn = fut.time_turn[turn]
    time_arrive = time + time_turn

    u_s = 0 if entry_to in {"r", "l"} else 1
    road_ang_u = -1 if entry_to in {"l", "u"} else 1

    s_s = 1 if u_s == 0 else 0
    road_ang_s = 1 if entry_to in {"r", "u"} else -1

    next_pos = [0, 0]
    next_pos[u_s] = (
        road.start[u_s] + (stgs.node_width / 2 + stgs.car_len / 2) * road_ang_u
    )
    next_pos[s_s] = road.start[s_s] + stgs.road_center * road_ang_s
    next_pos = tuple(next_pos)

    dist = road.get_car_dist(next_pos) - fut.junction_space
    road.add_car_junc_estimation(ID, time_arrive, dist)

    my_pred = dist / stgs.car_speed + time_arrive
    preds[ID] = my_pred

    # ____________________________________________
    ind = backward_linear_s(ID, road.estimation)

    queue = [(ID, "r", my_pred)]

    c = 1
    j = 0
    while c <= road.curr_capacity:
        index = ind - c - j
        id = road.estimation[index][0]
        if preds[id] == None:
            j += 1
            continue

        pind = backward_linear_s(road.start, fut.true_paths[id])
        if pind == None:
            paths[id] = fut.paths[id][0:1]
            timing_paths[id] = fut.timing_paths[id][0:1]
        else:
            paths[id] = fut.paths[id][0 : pind + 1]
            timing_paths[id] = fut.timing_paths[id][0 : pind + 1]

        if len(paths[id]) + 1 == len(fut.true_paths[id]):
            action = "p"
        else:
            action = "r"
        elem = (id, action, preds[id])

        binary_insert_q(elem, queue)

        c += 1

    while len(queue) > 0:
        id, action, timing = queue.pop(0)
        if action == "r":
            line = predict_car_in_front(
                id,
                preds,
                timing,
                0,
                paths,
                timing_paths,
                road,
                junc=road.to,
            )

            if line == None or line[2] == timing:
                if id == ID:
                    return timing

                elem = (id, "c", timing + 1)
                binary_insert_q(elem, queue)
                preds[id] = timing + 1

                paths[id].append(road.to)
                timing_paths[id].append(timing)

            else:
                elem = (id, "r", line[2])
                binary_insert_q(elem, queue)
                preds[id] = line[2]

        elif action == "c":

            paths[id].append(road.to)
            timing_paths[id].append(timing)

            # Just empty the road
            road.add_car_exit(id, timing)

        elif action == "p":
            # Get the car line
            # If works, empty the road
            park_line = predict_park_line(id, preds, paths, road, entry_to)

            if park_line == None:
                paths[id] = fut.paths[id]
                timing_paths[id] = fut.timing_paths[id]
                preds[id] = None
            else:
                elem = (id, "p", park_line[2])
                binary_insert_q(elem, queue)
                preds[id] = park_line[2]
