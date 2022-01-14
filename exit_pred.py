import settings as stgs
import traffic as trf
import future as fut
from pathfinding import get_abs_direction

pred_paths = [[] for i in range(stgs.num_car)]
pred_timing_paths = [[] for i in range(stgs.num_car)]


def add_car_path(ID, pos, timing):
    pred_paths[ID].append(pos)
    pred_timing_paths[ID].append(timing)


# ___The cutting code___________________________________________
def skip_queue(cars, subtime):
    global pred_paths, pred_timing_paths
    fut.reset_datastructures()

    pred_paths = [[] for i in range(stgs.num_car)]
    pred_timing_paths = [[] for i in range(stgs.num_car)]

    queue = []
    preds = [None for i in range(stgs.num_car)]
    for car in cars:
        if car.path != None:
            pred = predict_curr_car(car, subtime)
            fut.binary_insert_q(pred, queue)

            preds[car.id] = pred[2]
        elif car.id == subtime:
            fut.start_times[subtime] = stgs.time + 2
            fut.true_paths[subtime] = [(car.init_pos, "e"), (car.start_nodes[1], "i")]
            dir = get_abs_direction(*car.start_nodes)
            fut.dir_paths[subtime] = [dir, dir]

            pred = (car.id, fut.get_road_entry_pos(car.id), fut.start_times[subtime])
            fut.binary_insert_q(pred, queue)

            preds[car.id] = pred[2]

    return (queue, preds)


def get_car_action(ID, c):
    return fut.true_paths[ID][c + 1][1]


def predict_curr_car(car, subtime):
    action = get_car_action(car.id, car.c)

    if action == "e":
        pred_paths[car.id] = fut.paths[car.id][0 : car.c + 1]
        pred_timing_paths[car.id] = fut.timing_paths[car.id][0 : car.c + 1]

        timing = (
            fut.start_times[car.id]
            if fut.start_times[car.id] + 2 > stgs.time
            else stgs.time
        )

        if timing == stgs.time:
            timing = timing + 1 if car.id < subtime else timing
        return (car.id, fut.get_road_entry_pos(car.id), timing)
    elif action == "i":
        pred_paths[car.id] = fut.paths[car.id][0 : car.c + 1]
        pred_timing_paths[car.id] = fut.timing_paths[car.id][0 : car.c + 1]

        road = fut.get_entry_road(
            fut.true_paths[car.id][car.c + 1][0], fut.dir_paths[car.id][car.c + 1]
        )
        entry_pos = pred_paths[car.id][-1]
        time = pred_timing_paths[car.id][-1]
        dist = road.get_car_dist(entry_pos) - fut.junction_space
        road.add_car_junc_estimation(car.id, time, dist)

        if car.state <= 1:  # Not exited yet
            return fut.predict_intersection(car.id, pred_paths, pred_timing_paths)

        curr_dist = road.get_car_dist(car.pos) - fut.junction_space
        curr_dist = curr_dist if car.id < subtime else curr_dist - stgs.car_speed
        time_arrive = fut.Road.estimate_arrive(stgs.time, curr_dist)
        return (car.id, fut.get_intersection_pos(car.id, pred_paths), time_arrive)

    elif action in {"l", "r", "u"}:
        road = fut.get_entry_road(
            fut.true_paths[car.id][car.c][0], fut.dir_paths[car.id][car.c]
        )

        if car.waiting_intersection:
            pred_paths[car.id] = fut.paths[car.id][0 : car.c + 1]
            pred_timing_paths[car.id] = fut.timing_paths[car.id][0 : car.c + 1]

            entry_pos = pred_paths[car.id][-2]
            time = pred_timing_paths[car.id][-2]
            dist = road.get_car_dist(entry_pos) - fut.junction_space

            road.add_car_junc_estimation(car.id, time, dist)
            time_arrive = pred_timing_paths[car.id][-1]
            timing = time_arrive + 1 if stgs.time == time_arrive else stgs.time
            if timing == stgs.time:
                timing = timing + 1 if car.id < subtime else timing
            return (car.id, 90, timing)

        # If not waiting
        pred_paths[car.id] = fut.paths[car.id][0 : car.c + 2]
        pred_timing_paths[car.id] = fut.timing_paths[car.id][0 : car.c + 2]

        # Not neglecting old road to add estimation and delete -1 capacity
        old_time = pred_timing_paths[car.id][-3]
        old_pos = pred_paths[car.id][-3]
        old_dist = road.get_car_dist(old_pos) - fut.junction_space
        road.add_car_junc_estimation(car.id, old_time, old_dist)
        road.curr_capacity -= 1

        next_road = fut.roads[
            fut.true_paths[car.id][car.c][0], fut.dir_paths[car.id][car.c + 2]
        ]

        time_departed = pred_timing_paths[car.id][-1] - fut.time_turn[action]
        junc = fut.true_paths[car.id][car.c][0]

        entry_pos = pred_paths[car.id][-1]
        time = pred_timing_paths[car.id][-1]
        dist = next_road.get_car_dist(entry_pos) - fut.junction_space
        next_road.add_car_junc_estimation(car.id, time, dist)

        entry = trf.inverse_dir[fut.dir_paths[car.id][car.c]]
        entry_to = fut.dir_paths[car.id][car.c + 2]

        fut.junctions[junc].add_car_crossing(car.id, time_departed, entry, entry_to)

        next_action = get_car_action(car.id, car.c + 1)
        if next_action == "i":
            time_arrive = fut.Road.estimate_arrive(time, dist)
            return (car.id, fut.get_intersection_pos(car.id, pred_paths), time_arrive)
        else:
            return fut.predict_park(car.id, pred_paths, pred_timing_paths)

    elif action == "p":
        pred_paths[car.id] = fut.paths[car.id][0 : car.c + 1]
        pred_timing_paths[car.id] = fut.timing_paths[car.id][0 : car.c + 1]

        # Enter estimation here
        road = fut.roads[fut.true_paths[car.id][car.c][0], fut.dir_paths[car.id][-1]]
        entry_pos = pred_paths[car.id][-1]
        time = pred_timing_paths[car.id][-1]
        dist = road.get_car_dist(entry_pos) - fut.junction_space

        road.add_car_junc_estimation(car.id, time, dist)

        # Prediction
        pos = car.pos
        goal_pos = fut.true_paths[car.id][-1][0]
        u_s = 0 if road.start[0] != road.to[0] else 1

        dist = abs(goal_pos[u_s] - pos[u_s]) - stgs.park_dist
        dist = dist if car.id < subtime else dist - stgs.car_speed

        time_arrive = fut.Road.estimate_arrive(stgs.time, dist)

        return (car.id, fut.true_paths[car.id][-1][0], time_arrive)


# ____The predicting code______________________________________
def predict_first_node(cars, subtime):
    queue, preds = skip_queue(cars, subtime)

    # The Great Loop
    while len(queue) > 0:
        ID, pos, time = queue.pop(0)

        junction, action = fut.true_paths[ID][len(pred_paths[ID])]

        if action == "e":
            pred = fut.predict_road_entry(
                ID, time, preds, pred_paths, pred_timing_paths, next_pos=pos
            )

            if pred[2] == time:
                add_car_path(ID, pos, time + 32)
                pred = fut.predict_intersection(ID, pred_paths, pred_timing_paths)
                fut.binary_insert_q(pred, queue)
                preds[ID] = pred[2]

                # Register to road enter
                road = fut.get_entry_road(
                    fut.true_paths[ID][1][0], fut.dir_paths[ID][1]
                )
                dist = road.get_car_dist(pos) - fut.junction_space
                # road.add_car_enter(ID, time + 32, dist)
                road.add_car_junc_estimation(ID, time + 32, dist)
            else:
                fut.binary_insert_q(pred, queue)

                preds[ID] = pred[2]

        elif action == "i":
            # If there are no cars still at intersection execute the following
            # Else: Repredict when it will arrive at the intersection
            car_in_front = fut.predict_car_in_front(
                ID, preds, time, pos, pred_paths, pred_timing_paths
            )
            # road = get_entry_road(junction, dir_paths[ID][len(paths[ID]) - 1])

            if car_in_front == None:  # if no one is there

                if subtime == ID:
                    return time

                add_car_path(ID, pos, time)
                pred = fut.predict_junc_crossable(ID, time + 1, preds, pred_paths)
                fut.binary_insert_q(pred, queue)

                preds[ID] = pred[2]

            else:
                fut.binary_insert_q(car_in_front, queue)
                preds[ID] = car_in_front[2]

        elif action in {"l", "r", "u"}:
            crossable_pred = fut.predict_junc_crossable(ID, time, preds, pred_paths)
            if crossable_pred[2] == time:  # Let's cross
                # Adding to cross
                # intersect = junctions[junction]
                # entry = trf.inverse_dir[dir_paths[ID][len(paths[ID]) - 1]]
                # intersect.add_car_entry(ID, entry, time)

                # Predicting next cross
                finish_cross = fut.predict_turn(ID, time, pred_paths)
                add_car_path(*finish_cross)

                # Register road Exit
                road = fut.get_entry_road(
                    junction, fut.dir_paths[ID][len(pred_paths[ID]) - 2]
                )
                road.add_car_exit(ID, time)

                # Register crossing
                intersect = fut.junctions[junction]
                entry = trf.inverse_dir[fut.dir_paths[ID][len(pred_paths[ID]) - 2]]
                entry_to = fut.dir_paths[ID][len(pred_paths[ID])]
                intersect.add_car_crossing(ID, time, entry, entry_to)

                # Add to next road
                next_road = fut.roads[
                    (junction, fut.dir_paths[ID][len(pred_paths[ID])])
                ]
                time_cross = fut.time_turn[action] + 1  # 1 is time delay
                extra_dist = time_cross * stgs.car_speed - fut.junction_space
                dist = next_road.get_car_dist(finish_cross[1]) + extra_dist
                # next_road.add_car_enter(ID, time, dist)
                next_road.add_car_junc_estimation(ID, time, dist - stgs.car_speed)

                if len(pred_paths[ID]) + 1 == len(fut.dir_paths[ID]):
                    pred = fut.predict_park(ID, pred_paths, pred_timing_paths)
                else:
                    pred = fut.predict_intersection(ID, pred_paths, pred_timing_paths)

                fut.binary_insert_q(pred, queue)

                preds[ID] = pred[2]
            else:
                # Repredict
                fut.binary_insert_q(crossable_pred, queue)

                preds[ID] = crossable_pred[2]
        elif action == "p":
            # If the road is clean
            # Else repredict by seeing when other car exits road
            line_park = fut.predict_park_line(ID, preds, pred_paths)

            if line_park == None:
                add_car_path(ID, pos, time + 32 + 1)

                # Register exit
                road = fut.roads[(fut.true_paths[ID][-2][0], fut.dir_paths[ID][-1])]
                road.add_car_exit(ID, time)

                preds[ID] = None
            else:
                fut.binary_insert_q(line_park, queue)

                preds[ID] = line_park[2]

    # If nothing is returned:
    raise Exception("Could not predict the parking entrance accuretely")
