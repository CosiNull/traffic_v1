from future2 import *
import settings as stgs
import future as fut
import math
import traffic as trf
from pathfinding import *
from multiagent2 import *


# ___The code
def multiagent_a_star(graph, start, end, start_dir, true_goal, time):
    timing = {node: math.inf for node in graph.nodes}
    prev = {}

    timing[start] = 0
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
            current_time = timing[node] + edge.length

            if current_time < timing[neighbour]:
                timing[neighbour] = current_time
                pq.insert((neighbour, current_time + manhattan_dist(neighbour, end)))
                prev[neighbour] = node

    return (None, None)


delays = {node: [0 for i in range(stgs.num_car)] for node in trf.road_network.nodes}

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

    if index < len(arr):
        if arr[index][0] == ID:
            index -= 1
        id = arr[index][0]
        time_leave = true_junction.entry_cross[entry_from][index][1]

    if index < len(arr) and arr[index][1] < time and time - time_leave <= 52:
        ind = backward_linear_s(junc, fut.true_paths[id])
        paths[id] = fut.paths[id][0:ind]
        timing_paths[id] = fut.timing_paths[id][0:ind]

        next_entry = fut.dir_paths[id][ind + 1]
        elem = (id, "t", time_leave, entry_from, next_entry)
        binary_insert_q(elem, queue)

        last_left[next_entry] = min(elem[2], last_left[next_entry])

        banned_ids.add(id)

    # 2. Get the roads
    for entry, road in true_roads.items():
        ind = binary_search_ds(last_left[entry] - 1, road.timely_capacity)

        if ind >= len(road.timely_capacity):
            continue

        capacity, timing, type = road.timely_capacity[ind]

        if not (timing == time and type == "i"):
            capacity = road.timely_capacity[ind - 1][0]

        index = binary_search_ds(last_left[entry] - 1, road.leave)

        c = 1
        while c <= capacity:
            i = index - c

            stuff = road.leave[i]

            # Get estimation
            id = stuff[0]

            # Only for debugging_________
            if id in banned_ids:
                c += 1
                continue
            # Only for debugging_________

            pind = backward_linear_s(junc, fut.true_paths[id])
            if pind == None:
                pind = 0
            try:
                pos = fut.paths[id][pind]
            except:
                raise Exception(pind, len(paths[id]), len(fut.true_paths[id]), junc)
            timing = fut.timing_paths[id][pind]
            dist = road.get_car_dist(pos)

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
    # Not now :sob

    # if stgs.time == 687:
    #    for entry, road in new_roads.items():
    #        print("Entry:", entry, road.estimation)
    #    print(queue)

    while len(queue) > 0:
        id, action, timing, start, to = queue.pop(0)

        if action == "t":
            can_cross = predict_junc_crossable(
                id, timing, preds, paths, start, to, new_junction, new_roads[to]
            )
            if can_cross[2] == timing:
                if id == ID:
                    return timing

                # Register to crossing
                new_junction.add_car_crossing(id, timing, start, to)

                # Add to next road
                finish_cross = fut.predict_turn(id, timing, paths)

                dist = new_roads[to].get_car_dist(finish_cross[1])
                new_roads[to].add_car_junc_estimation(id, time, dist - stgs.car_speed)

                # Addding to paths
                paths[id].append(junc)
                timing_paths[id].append(timing)

                if len(paths[id]) + 1 == len(fut.dir_paths[id]):
                    pred = fut.predict_park(id, paths, timing_paths)
                else:
                    pred = fut.predict_intersection(id, paths, timing_paths)

                elem = (id, "r", pred[2], junc, new_roads[to].to)
                binary_insert_q(elem, queue)
                preds[id] = pred[2]

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

            if line == None:
                elem = (id, "c", timing + 1, start, to)
                binary_insert_q(elem, queue)
                preds[id] = timing + 1
            else:
                elem = (id, "r", line[2], start, to)
                preds[id] = line[2]

        elif action == "c":
            dir = get_abs_direction(start, to)
            road = new_roads[dir]
            # Just empty the road
            road.add_car_exit(id, time)

        elif action == "p":
            pass
