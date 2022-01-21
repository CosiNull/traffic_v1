from copy import deepcopy
from future2 import linear_search
import settings as stgs

import math
import random as rdn
import pathfinding as pf
import future as fut

import traffic as trf
import parking as pk

import exit_pred as ex
import multi_agent as mlt


# Local settings
rdn.seed(stgs.seed)
car_width = int((stgs.node_width / 2) * 0.5)


# Car__________________________________________________________________________________________________________
class Car:
    # Inittializing
    def __init__(self, ID, color, parktime=0, autonomous=False):
        self.id = ID
        self.state = 0
        self.len = stgs.car_len
        self.width = car_width

        self.dead = False

        self.set_pos(trf.road_network)

        self.color = color
        self.speed = stgs.car_speed
        self.park_speed = 0.4

        self.path = None
        if self.id < stgs.tested:
            self.park_time = 300
        else:
            self.park_time = 0  # parktime
        self.goal = 0

        self.pause = False
        self.waiting_intersection = False
        self.intersection_line = False

        self.autonomous = autonomous
        self.func = "as"
        self.c = -1

        self.checked = False

    def set_pos(self, graph, random=True):
        if random:
            node, to, start = pf.rand_graph_pos(graph, self.len)
        else:
            node, to = self.start_nodes
            # Setting up the start
            unsame_indexes = (
                0 if self.start_nodes[0][0] != self.start_nodes[1][0] else 1
            )
            start = abs(node[unsame_indexes] - self.pos[unsame_indexes])

        # Set Position
        variation = stgs.node_width / 2 - self.width
        variation += pk.Parking_Lot.dist_from_road

        if node[0] == to[0]:
            if node[1] > to[1]:  # Vertical (x are the same) (0,4)(0,5)
                self.pos = (node[0] + variation, node[1] - start)
                #
                self.angle = math.pi * 3 / 2
                # self.dead = True
            else:
                self.pos = (node[0] - variation, node[1] + start)
                #
                self.angle = math.pi / 2

        else:  # Horizontal (y are the same)
            if node[0] < to[0]:  # -->
                self.pos = (node[0] + start, node[1] + variation)
                #
                self.angle = 0

            else:  # <--
                self.pos = (node[0] - start, node[1] - variation)
                #
                self.angle = math.pi

        self.start_nodes = (node, to)
        self.road_angle = self.angle
        self.pos = (round(self.pos[0]), round(self.pos[1]))

        #
        if not pk.parking.can_park(self.start_nodes, self.pos, self.len, self.id):
            self.set_pos(graph, True)
            return

        if random:
            pk.parking.add_car(self.start_nodes, self.pos, self.len, self.id)

        #
        self.init_pos = self.pos
        self.init_nodes = self.start_nodes
        self.last_intersection = self.start_nodes[0]

    def enter_road(self):
        if not self.state == 0:
            raise Exception("ERROR: Car cannot exit parking that it already exited")

            # print(self.goal)
        self.state = 0.5
        self.gas = 0
        self.turn_state = 0

    def find_path(self, goal, true_goal, cars, dist, target_pos, func="as"):
        fut.reset_path(self.id)

        if self.id < stgs.tested:  # HERE DUM DUM
            func = "mlt"
            if func == "mlt":
                self.func = "mlt"

        # Set Start Pos
        start = self.start_nodes[1]
        start_dir = pf.angle_to_dir[self.angle]
        self.init_dir = start_dir

        # Before its paths
        exit_time = ex.predict_first_node(cars, self.id) + 1

        if func == "dj":
            self.path = pf.pathfind_dj(trf.road_network, start, goal, start_dir)[0]
        elif func == "as":
            self.path = pf.pathfind_as(
                trf.road_network, start, goal, start_dir, true_goal
            )[0]

            """
            if self.id == 0:
                self.path2 = mlt.pathfind_mlt(
                    trf.road_network,
                    start,
                    goal,
                    start_dir,
                    true_goal,
                    exit_time,
                    self.id,
                )[0]
            """

        elif func == "mlt":
            res = mlt.pathfind_mlt(
                trf.road_network,
                start,
                goal,
                start_dir,
                true_goal,
                exit_time,
                self.id,
            )
            self.path = res[0]

            self.mypred = res[1]

        else:
            raise Exception("ERROR: Bad 'func' Parameter")

        if self.path == None:
            print("Could not find good path")
            return

        self.path.append(true_goal)
        self.predicted_nodes = (goal, true_goal)
        self.backup_path = deepcopy(self.path)

        self.goal = dist
        self.target_pos = target_pos

        pk.parking.add_car(self.predicted_nodes, target_pos, stgs.car_len, self.id)

        # The stuff
        def add_dir(path_list, index, abs_curr_dir):
            if index >= len(path_list) - 1:
                return
            direc = pf.get_abs_direction(path_list[index], path_list[index + 1])

            try:
                turn = pf.relative_dir[abs_curr_dir][direc]
            except:
                raise Exception(abs_curr_dir, direc, index, "\n", path_list)

            path_list.insert(index + 1, (turn))
            add_dir(path_list, index + 2, direc)

        add_dir(self.path, 0, start_dir)
        """
        self.path2.append(true_goal)  #
        add_dir(self.path2, 0, start_dir)  #
        """

        self.predict_path(cars)

    # Future predicting_________________________________________________________________________
    def predict_path(self, cars, panic=False):

        if not panic:
            fut.save_true_path(
                self.id,
                self.path,
                self.init_pos,
                self.target_pos,
                pf.angle_to_dir[self.angle],
            )

        panic = fut.predict_path(cars, self.id)

        if panic:
            fut.reset_path(self.id)
            self.path = None
            self.predict_path(cars, panic=True)
            print("PANIC MODE")

        # self.repredict_mlts(cars)

    def repredict_mlts(self, cars):
        for i in range(0, stgs.tested):
            if cars[i].func == "mlt" and cars[i].path != None:
                cars[i].refind_path(cars)

    def refind_path(self, cars):
        if self.c + 1 > len(fut.true_paths[self.id]) - 3:
            return

        exit_time = None
        if self.c == -1:
            index = 1
        elif self.start_nodes[1] != self.last_intersection:
            if type(self.path[0]) == str:
                print("This is weird")
            else:
                index = self.c + 1
        elif type(self.path[0]) == str and self.waiting_intersection:
            index = self.c
            exit_time = stgs.time
        elif type(self.path[0]) == str and not self.waiting_intersection:
            index = self.c + 2

        start = fut.true_paths[self.id][index][0]
        exit_time = fut.timing_paths[self.id][index] if exit_time == None else exit_time
        goal, true_goal = self.predicted_nodes
        start_dir = fut.dir_paths[self.id][index]
        curr_action = self.path[0]

        new_path = mlt.pathfind_mlt(
            trf.road_network,
            start,
            goal,
            start_dir,
            true_goal,
            exit_time,
            self.id,
        )[0]

        new_path.append(true_goal)

        def add_dir(path_list, index, abs_curr_dir):
            if index >= len(path_list) - 1:
                return
            direc = pf.get_abs_direction(path_list[index], path_list[index + 1])

            turn = pf.relative_dir[abs_curr_dir][direc]

            path_list.insert(index + 1, (turn))
            add_dir(path_list, index + 2, direc)

        add_dir(new_path, 0, start_dir)

        if not self.c < 1:
            new_path = self.backup_path[0 : index + 1] + new_path[1:]

        last_time = fut.timing_paths[self.id][-1]
        self.path = new_path

        fut.save_true_path(
            self.id, self.path, self.init_pos, self.target_pos, start_dir
        )

        panic = fut.predict_path(cars, self.id)

        if panic or fut.timing_paths[self.id][-1] > last_time:
            self.path = self.backup_path
            fut.save_true_path(
                self.id, self.path, self.init_pos, self.target_pos, self.init_dir
            )
            print(panic)
            fut.predict_path(cars, self.id)

        print("Got out")

    # The Holy Update Method_____________________________________________________________
    def update(self):
        if self.pause:
            return
        if self.state == 0.5:
            my_dir = trf.entry_dir(*self.start_nodes)
            if trf.roads[(self.start_nodes[0], my_dir)].can_out_parking(self):
                self.c = 0
                self.state = 1
                # Add to road dict
                my_dir = trf.entry_dir(*self.start_nodes)
                junc = self.start_nodes[0]
                trf.roads[(junc, my_dir)].add_car(self, sort=True)

        elif self.state == 1:  # Exit parking
            self.gas += 1
            self.park(exit=True)

        elif self.state == 2:  # Moving to destination
            self.gas += 1
            self.move_to_dest()

        elif self.state == 3:  # Enter Parking
            self.gas += 1
            self.park(exit=False)

    # Basic Control flow
    turn_speed = 0.12

    def park(self, exit=True):

        # NOTE: Gas: 32  Distance: 9.89707610702726
        turn = self.turn_speed  # if horizontal else -self.turn_speed
        turn = turn if exit else -turn

        if self.turn_state >= 0:
            if self.turn_state == 0:  # Turn into it
                if abs(self.road_angle - self.angle) < math.pi / 4:
                    self.angle -= turn
                else:
                    self.turn_state = 1
            elif self.turn_state < 17:  # How far into the road
                self.turn_state += 1
            elif self.turn_state > 0:  # Turn out of it
                if abs(self.road_angle - self.angle) > 0:
                    self.angle += turn
                else:
                    self.turn_state = -1

            self.move_forward(self.park_speed)

        if self.turn_state == -1:
            if exit:
                self.turn_state = 0
                pk.parking.delete_car(self.start_nodes, self.init_pos)
                self.state = 2
                self.start_time = stgs.time
                # print(self.init_pos[self.u_s] - self.pos[self.u_s])

                self.center_to_road(False)

                """
                timing = fut.timing_paths[self.id][0]
                if timing != stgs.time:
                    print("EXIT______________")
                    print(self.id, self.color, fut.true_paths[self.id][1])
                    print(timing, stgs.time)
                    print("___________________")
                """

                """
                # Road enter and estimations
                ind = fut.binary_search_ds(stgs.time, fut.roads[(junc, my_dir)].enter)
                print(fut.roads[(junc, my_dir)].enter[ind], stgs.time)
            
                ind = fut.binary_search_ds(
                    stgs.time, fut.roads[(junc, my_dir)].estimation
                )
                print(fut.roads[(junc, my_dir)].estimation[ind], stgs.time)
                """

                # print(fut.timing_paths[0][0], stgs.time)
                # print(fut.paths[0][0], self.pos)

                if fut.timing_paths[self.id][self.c] != stgs.time:
                    fut.timing_paths[self.id][self.c] = stgs.time
                    # print("Error cocorrection", self.id)

            else:
                self.set_pos(trf.road_network, False)

                self.state = 0
                self.park_time = 200
                self.gas = 0
                self.goal = 0

                timing = fut.timing_paths[self.id][-1]
                # if timing != stgs.time:
                #     print(self.id, self.color, fut.true_paths[self.id][-2])
                #     print(timing, stgs.time)

                self.c = -1

                """
                # Road Exit
                ind = fut.binary_search_ds(stgs.time, fut.roads[(junc, my_dir)].leave)
                print(fut.roads[(junc, my_dir)].leave[ind], stgs.time)
                """

                # Can out this in the future
                self.pause = True

                # print(stgs.time)
                if self.id < stgs.tested:
                    self.pause = True
                    stgs.count += 1
                    print(self.id, stgs.time - self.start_time, self.start_time)

    def move_to_dest(self):
        # Move Forward
        if len(self.path) == 0:
            # self.color = (255, 255, 255)
            self.path = None
            self.state = 3

            # HELLO
            stgs.car_evolution.append(stgs.time)
            # print(len(stgs.car_evolution))

        elif not type(self.path[0]) == str:
            self.advance_to_dest()
        else:
            if self.waiting_intersection:
                self.wait_intersection()
                return
            if self.path[0] == "u":
                self.intersect_forward()
            elif self.path[0] == "r":
                self.intersect_right()
            elif self.path[0] == "l":
                self.intersect_left()

    def advance_to_dest(self):
        unsame_indexes = 0 if self.start_nodes[0][0] != self.start_nodes[1][0] else 1
        pos_angle = (
            1
            if self.start_nodes[0][unsame_indexes] < self.start_nodes[1][unsame_indexes]
            else -1
        )

        # To understand the dist formula and not spend 2 minutes staring at it, Imagine that the car is going to the right
        if not len(self.path) == 1:
            dist = (
                self.path[0][unsame_indexes]
                - (stgs.node_width / 2 + self.len / 2) * pos_angle
            ) - (self.pos[unsame_indexes])
            dist = abs(dist)

            # NOTE ADD LIMIT HERE
        else:
            dist = (
                self.path[0][unsame_indexes] - (self.goal + stgs.park_dist) * pos_angle
            ) - (self.pos[unsame_indexes])
            dist = abs(dist)
            # dist -= self.goal

        if dist <= self.speed:
            self.turn_state = 0
            if dist != 0:
                self.move_forward(dist)

            # self.save = (self.pos, self.gas)
            self.last_intersection = self.path.pop(0)

            if len(self.path) > 1:
                from_inter = trf.angle_to_intersect[self.angle]
                self.junction_id = (self.id, from_inter)
                self.road_to = trf.abs_dir(trf.inverse_dir[from_inter], self.path[0])

                if not self.waiting_intersection:
                    self.waiting_intersection = True
                    trf.junctions[self.last_intersection].add_car_entry(
                        *self.junction_id
                    )

                    """
                    # Intersection enter
                    junc = self.last_intersection
                    ind = fut.binary_search_ds(
                        stgs.time, fut.junctions[junc].entries[from_inter]
                    )
                    print(fut.junctions[junc].entries[from_inter][ind], stgs.time)
                    """
                self.c += 1

                if fut.timing_paths[self.id][self.c] != stgs.time:
                    fut.timing_paths[self.id][self.c] = stgs.time
                    print("Error cocorrection", self.id)

                # if self.time_pred != stgs.time and self.c == 1:
                #    print(self.id, self.time_pred, stgs.time)

                # print(fut.paths[0][self.c], self.pos)

                # if self.id == 22:
                #    print("a", fut.timing_paths[self.id], stgs.time)
            else:
                # Remove the car from the road, it's parking time!
                my_dir = pf.angle_to_dir[self.angle]
                junc = self.start_nodes[0]
                trf.roads[(junc, my_dir)].remove_car(self)

        else:
            my_dir = pf.angle_to_dir[self.angle]
            intersection_from = self.start_nodes[0]
            current_road = trf.roads[(intersection_from, my_dir)]
            car_index = current_road.index_car(self)

            dont_move = False
            if car_index > 0:
                car1 = current_road.cars[car_index]
                car2 = current_road.cars[car_index - 1]
                dont_move = (
                    current_road.car_dist(car1, car2) <= stgs.car_len + stgs.min_dist
                )

            if not dont_move:
                self.move_forward(self.speed)

            """
            else:
                # Not necessary for now but it is good to keep
                if (
                    len(self.path) > 1
                    and not self.waiting_intersection
                    and current_road.cars[car_index - 1].waiting_intersection
                ):

                    self.last_intersection = self.path[
                        0
                    ]  # It is called last intersection even though it didnt reach it next
                    self.waiting_intersection = True
                    trf.junctions[self.last_intersection].add_car_entry(
                        *self.junction_id
                    )
                    trf.junctions[self.last_intersection].add_car_queue(
                        *self.junction_id
                    )
            """

    # Movement
    def move_forward(self, speed):
        mov_x = math.cos(self.angle) * speed
        mov_y = math.sin(self.angle) * speed

        self.pos = (self.pos[0] + mov_x, self.pos[1] + mov_y)

    def center_to_road(self, intersection_cross=True):
        u_s = self.u_s
        road_angle = 1 if self.start_nodes[0][u_s] < self.start_nodes[1][u_s] else -1

        if u_s == 0:
            x, y = self.pos
            y = self.start_nodes[0][1]
            y += stgs.road_center * road_angle

            x = round(x)
            if intersection_cross:
                x = (
                    round(self.start_nodes[0][0])
                    + (stgs.node_width / 2 + self.len / 2) * road_angle
                )
            self.pos = (x, y)
        else:
            x, y = self.pos
            x = self.start_nodes[0][0]
            x -= stgs.road_center * road_angle

            y = round(y)
            if intersection_cross:
                y = (
                    round(self.start_nodes[0][1])
                    + (stgs.node_width / 2 + self.len / 2) * road_angle
                )

            self.pos = (x, y)

    # intersections______________________________________________________________
    def intersect_forward(self):
        # Distance: 24.0, Gas: 49
        if self.turn_state < math.ceil((stgs.node_width + self.len) / self.speed):
            self.move_forward(self.speed)
            self.turn_state += 1
        else:
            self.exit_intersection()

    right_turn_deviation = 0.075

    def intersect_right(self):  # dist_inter: 12
        # different-axis Distance: 6.914533413670597, same-axis distance: 6.441554850340253, Gas: 22
        if self.angle - self.road_angle < math.pi / 2:
            self.move_forward(self.speed)
            self.angle += self.right_turn_deviation
        else:

            self.angle = (self.road_angle + math.pi / 2) % (math.pi * 2)
            self.road_angle = self.angle

            self.exit_intersection()

    left_turn_deviation = 0.032

    def intersect_left(self):  # dist_inter: 12
        # Same-axis Distance: 15.829976845746955, Different-axis distance: 15.87430464054819, Gas: 51
        if self.road_angle - self.angle < math.pi / 2:
            self.move_forward(self.speed)
            self.angle -= self.left_turn_deviation
        else:
            self.angle = (self.road_angle - math.pi / 2) % (math.pi * 2)
            self.road_angle = self.angle

            # if self.road_angle == 0 or self.road_angle == math.pi:
            #    print(
            #        f"Distance: {self.pos[1]-self.save[0][1]}, Gas: {self.gas-self.save[1]}"
            #    )
            self.exit_intersection()

    def wait_intersection(self):
        junction_data = trf.junctions[self.last_intersection]
        start_dir = pf.angle_to_dir[self.angle]
        start_inter = trf.inverse_dir[start_dir]
        next_dir = trf.abs_dir(start_dir, self.path[0])

        """
        if not self.checked and self.c + 3 < len(fut.true_paths[self.id]):
            self.checked = True
            path_so_far = fut.paths[self.id][0 : self.c + 2]
            timing = mlt.predict_time_cross(
                self.id,
                stgs.time,
                self.last_intersection,
                start_inter,
                next_dir,
                path_so_far,
            )
            real = fut.timing_paths[self.id][self.c + 2]
            # ind = fut.linear_search(
            #    self.id, fut.junctions[self.last_intersection].crossing_enter
            # )
            # real = fut.junctions[self.last_intersection].crossing_enter[ind][1]
            if timing != real:
                print("_" * 20)
                print("ID", self.id)
                print(stgs.time, timing, real)
                print("IDEN", self.last_intersection, self.color)
                print(junction_data.crossing)
                # ind = fut.linear_search(
                #    10, fut.junctions[self.last_intersection].crossing_enter
                # )
                # print(fut.junctions[self.last_intersection].crossing_enter[ind][1])
                print("_" * 20)
        """

        # Go check who is where
        crossable = all(
            [
                (start, to) in trf.no_conflicts[(start_inter, next_dir)]
                for ID, start, to in junction_data.crossing
            ]
        )

        if (
            crossable  # Crossing
            and not trf.roads[(self.last_intersection, next_dir)].is_full()
        ):
            self.checked = False

            self.waiting_intersection = False
            # Remove from junction data structure
            junction_data.remove_car(*self.junction_id)
            junction_data.crossing.append(
                (self.junction_id[0], self.junction_id[1], self.road_to)
            )

            # Remove the car from the road
            my_dir = pf.angle_to_dir[self.angle]
            intersection_from = self.start_nodes[0]
            trf.roads[(intersection_from, my_dir)].pop_car(self)

            # Add it to other Road
            new_dir = trf.abs_dir(pf.angle_to_dir[self.angle], self.path[0])
            intersection_from = self.start_nodes[1]
            trf.roads[(intersection_from, new_dir)].add_car(self)

            # Crossing_enter
            """
            junc = self.last_intersection
            ind = fut.linear_search(self.id, fut.junctions[junc].crossing_enter)
            timing = fut.junctions[junc].crossing_enter[ind][1]
            if timing != stgs.time:
                global yo
                yo += 1
                print("_______________________________")
                print(yo, junc, self.id, self.color)
                print(timing, stgs.time)
            """

            """"
            # Road_exit
            junc = self.start_nodes[0]
            ind = fut.binary_search_ds(stgs.time, fut.roads[(junc, start_dir)].leave)
            print(fut.roads[(junc, start_dir)].leave[ind], stgs.time)
            """

            #  Road_enter and estimation
            """
            junc = self.last_intersection

            # ind = fut.binary_search_ds(stgs.time, fut.roads[(junc, next_dir)].enter)
            # print(fut.roads[(junc, next_dir)].enter[ind], stgs.time)

            ind = fut.binary_search_ds(
                stgs.time, fut.roads[(junc, next_dir)].estimation
            )
            print(fut.roads[(junc, next_dir)].estimation[ind], stgs.time)
            """

    def exit_intersection(self):
        self.turn_state = 0
        self.path.pop(0)
        self.start_nodes = (self.last_intersection, self.path[0])
        self.center_to_road()

        trf.junctions[self.last_intersection].crossing.remove(
            (self.junction_id[0], self.junction_id[1], self.road_to)
        )

        """
        # Crossing_exit
        junc = self.last_intersection
        ind = fut.binary_search_ds(stgs.time, fut.junctions[junc].crossing_exit)
        print(fut.junctions[junc].crossing_exit[ind], stgs.time)
        """

        # Crossing remove
        self.c += 1

        if fut.timing_paths[self.id][self.c] != stgs.time:
            fut.timing_paths[self.id][self.c] = stgs.time
            print("Error cocorrection", self.id)

        # print(
        #     fut.timing_paths[0],
        #     stgs.time,
        #     fut.true_paths[self.id][self.c + 1][1],
        # ),

        # if self.id == 22:
        #     print("i", fut.timing_paths[self.id], stgs.time)

    @property
    def points(self):
        basic_points = [
            (self.len / 2, self.width / 2),  # up-right
            (self.len / 2, -self.width / 2),  # down-right
            (-self.len / 2, -self.width / 2),  # down-left
            (-self.len / 2, self.width / 2),  # up-left
        ]

        res = []
        for point in basic_points:
            x = point[0] * math.cos(self.angle) - point[1] * math.sin(self.angle)
            y = point[0] * math.sin(self.angle) + point[1] * math.cos(self.angle)
            res.append((x + self.pos[0], y + self.pos[1]))

        return res

    @property
    def u_s(self):
        return 0 if self.start_nodes[0][0] != self.start_nodes[1][0] else 1


yo = 0
