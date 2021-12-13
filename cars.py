import settings as stgs

import math
import random as rdn
import pathfinding as pf

import traffic as trf


# Local settings
rdn.seed(1)
car_width = int((stgs.node_width / 2) * 0.5)
colors = [
    (255, 0, 0),
    (0, 150, 0),
    (0, 0, 255),
    (255, 0, 255),
    (255, 255, 0),
    (0, 255, 255),
    (128, 0, 128),
    (0, 128, 128),
    (255, 140, 0),
]

# Car data class
class Car:
    # Inittializing
    def __init__(self, graph, ID):
        self.id = ID
        self.state = 0
        self.len = stgs.car_len
        self.width = car_width

        self.dead = False

        self.set_pos(graph)

        self.graph = graph

        self.color = colors[rdn.randint(0, len(colors) - 1)]
        self.speed = stgs.car_speed
        self.park_speed = 0.4

        self.path = None
        self.park_time = 0
        self.goal = 0

        self.pause = False
        self.waiting_intersection = False
        self.intersection_line = False

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
        variation += Parking_Lot.dist_from_road

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
        if not parking.can_park(self.start_nodes, self.pos, self.len, self.id):
            self.set_pos(graph, True)
            return
        if random:
            parking.add_car(self.start_nodes, self.pos, self.len, self.id)

        #
        self.init_pos = self.pos

    def enter_road(self):
        if not self.state == 0:
            raise Exception("ERROR: Car cannot exit parking that it already exited")

            # print(self.goal)
        self.state = 0.5
        self.gas = 0
        self.turn_state = 0

    def find_path(self, goal, func="as"):
        # Set Start Pos
        start = self.start_nodes[1]
        start_dir = pf.angle_to_dir[self.angle]

        # Graph
        # graph = cpy.deepcopy(self.graph)
        # record first direction and to ban u-turns
        # graph.remove_edge(self.start_nodes[1], self.start_nodes[0])

        if func == "dj":
            self.path = pf.pathfind_dj(self.graph, start, goal, start_dir)[0]
        elif func == "as":
            self.path = pf.pathfind_as(self.graph, start, goal, start_dir)[0]
        else:
            raise Exception("ERROR: Bad 'func' Parameter")

        # Next step path itinerary NOTE
        last_node = self.path[-1]
        before_last_node = self.path[-2]

        self.goal = 0
        if not len(self.path) >= 2:
            raise Exception("Path too short")

        u_s = 0 if last_node[0] != before_last_node[0] else 1
        a = (
            stgs.car_len / 2
            + stgs.node_width / 2
            + stgs.park_dist
            + parking.min_park_dist
        )
        b = (
            abs(last_node[u_s] - before_last_node[u_s])
            - stgs.node_width / 2
            - stgs.car_len / 2
            - stgs.park_dist
            - parking.min_park_dist
        )

        if a >= b:
            self.path = None
            return
        self.goal = rdn.randint(
            math.ceil(a),
            math.floor(b),
        )
        self.predicted_nodes = (before_last_node, last_node)

        # Parking reservation
        pos_dir = (
            1 if self.predicted_nodes[1][u_s] > self.predicted_nodes[0][u_s] else -1
        )
        edge = (before_last_node, last_node)
        pos = list(last_node)

        pos[u_s] -= self.goal * pos_dir
        s_i = 0 if u_s == 1 else 1

        if s_i == 1:
            pos[s_i] += 12 * pos_dir
        else:
            pos[s_i] -= 12 * pos_dir

        pos = tuple(pos)

        self.target_pos = pos

        if parking.can_park(edge, pos, self.len, self.id):
            parking.add_car(edge, pos, self.len, self.id)
        else:
            self.path = None
            return

        def add_dir(path_list, index, abs_curr_dir):
            if index >= len(path_list) - 1:
                return
            direc = pf.get_abs_direction(path_list[index], path_list[index + 1])

            try:
                turn = pf.relative_dir[abs_curr_dir][direc]
            except:
                raise Exception(abs_curr_dir, direc)

            path_list.insert(index + 1, (turn))
            add_dir(path_list, index + 2, direc)

        add_dir(self.path, 0, start_dir)

    # The Holy Update Method
    def update(self):
        if self.pause:
            return
        if self.state == 0.5:
            my_dir = trf.entry_dir(*self.start_nodes)
            if trf.roads[(self.start_nodes[0], my_dir)].can_out_parking(self):
                self.state = 1
        elif self.state == 1:  # Exit parking
            self.park(exit=True)
            self.gas += 1
        elif self.state == 2:  # Moving to destination
            self.move_to_dest()
            self.gas += 1
        elif self.state == 3:  # Enter Parking
            self.park(exit=False)

    # Control flow
    turn_speed = 0.12

    def park(self, exit=True):  # NOTE: Unfinished

        # NOTE: Gas: 31  Distance: 9.497076107027738
        horizontal = self.start_nodes[0][1] == self.start_nodes[1][1]
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
                parking.delete_car(self.start_nodes, self.init_pos)
                self.state = 2
                self.center_to_road(False)

                # Add to road dict
                my_dir = trf.entry_dir(*self.start_nodes)
                trf.roads[(self.start_nodes[0], my_dir)].add_car(self, sort=True)

            else:
                self.set_pos(self.graph, False)

                """
                print(
                    f"\nPOS: {self.pos},\nTARGET_POS{self.target_pos},\nSTART_NODES {self.start_nodes},\nPREDICTED_NODES{self.predicted_nodes},\nGOAL: {self.goal}"
                )
                """

                self.state = 0
                self.path = None
                self.park_time = 200
                self.gas = 0
                self.goal = 0
                # print(self.pos, self.angle)

                # Remove the car from the road
                my_dir = pf.angle_to_dir[self.angle]
                intersection_from = self.start_nodes[0]
                trf.roads[(intersection_from, my_dir)].remove_car(self)

    def move_to_dest(self):  # NOTE: UNFINISHED
        # Move Forward
        if len(self.path) == 0:
            # self.color = (255, 255, 255)
            self.state = 3
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

    def advance_to_dest(self):  # NOTE: UNFINISHED
        unsame_indexes = 0 if self.start_nodes[0][0] != self.start_nodes[1][0] else 1
        pos_angle = (
            1
            if self.start_nodes[0][unsame_indexes] < self.start_nodes[1][unsame_indexes]
            else -1
        )

        # To understand the dist formula and not spend 2 minutes staring at it, Imagine that the car is going to the right
        """
        car_line = 0
        """
        if not len(self.path) == 1:
            """
            cars_waiting = trf.junctions[self.path[0]].num_cars_entry(
                trf.angle_to_intersect[self.angle]
            )
            car_line = cars_waiting * (stgs.car_len + parking.min_park_dist)
            """

            dist = (
                self.path[0][unsame_indexes]
                - (stgs.node_width / 2 + self.len / 2) * pos_angle
            ) - (self.pos[unsame_indexes])
            dist = abs(dist)
            """
            dist -= car_line
            """

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

            """
            self.intersection_line = False
            """

            if len(self.path) > 1:
                # self.last_intersection = self.path[0]
                self.junction_id = (self.id, trf.angle_to_intersect[self.angle])

                if not self.waiting_intersection:
                    self.waiting_intersection = True
                    trf.junctions[self.last_intersection].add_car_entry(
                        *self.junction_id
                    )
                    trf.junctions[self.last_intersection].add_car_queue(
                        *self.junction_id
                    )

            else:
                # Special parking case
                pass

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
                    current_road.car_dist(car1, car2)
                    < (car1.len / 2 + car2.len / 2) + parking.min_park_dist
                )

            if not dont_move:
                self.move_forward(self.speed)
            else:
                # Not necessary for now but it is good to keep
                if (
                    len(self.path) > 1
                    and not self.waiting_intersection
                    and current_road.cars[car_index - 1].waiting_intersection
                ):
                    self.junction_id = (self.id, trf.angle_to_intersect[self.angle])
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

    # Movement
    def move_forward(self, speed):
        mov_x = math.cos(self.angle) * speed
        mov_y = math.sin(self.angle) * speed

        self.pos = (self.pos[0] + mov_x, self.pos[1] + mov_y)

    def center_to_road(self, intersection_cross=True):
        u_s = 0 if self.start_nodes[0][0] != self.start_nodes[1][0] else 1
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

    def intersect_right(self):
        # different-axis Distance: 6.914533413670597, same-axis distance: 6.441554850340253, Gas: 22
        if self.angle - self.road_angle < math.pi / 2:
            self.move_forward(self.speed)
            self.angle += self.right_turn_deviation
        else:

            self.angle = (self.road_angle + math.pi / 2) % (math.pi * 2)
            self.road_angle = self.angle

            # if self.road_angle == 0 or self.road_angle == math.pi:
            #    print(
            #        f"Distance: {self.pos[0]-self.save[0][0]}, Gas: {self.gas-self.save[1]}"
            #    )
            self.exit_intersection()

    left_turn_deviation = 0.032

    def intersect_left(self):
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

        if (
            # not self.intersection_line
            len(junction_data.crossing) == 0  # Crossing
            and junction_data.queue_front[0] == self.id
        ):
            # Go code the stuff that calculates if the next road is full here
            # NOTE YOU ARE HERE
            start_dir = pf.angle_to_dir[self.angle]
            next_dir = trf.abs_dir(start_dir, self.path[0])
            if trf.roads[(self.last_intersection, next_dir)].is_full():
                car = junction_data.queue.pop(0)
                i = junction_data.get_index_car_first_entry()
                junction_data.queue.insert(i, car)
                return

            # Remove from junction data structure
            self.waiting_intersection = False
            junction_data.remove_car(*self.junction_id)
            junction_data.crossing.append(self.junction_id)

            # Remove the car from the road
            my_dir = pf.angle_to_dir[self.angle]
            intersection_from = self.start_nodes[0]
            trf.roads[(intersection_from, my_dir)].pop_car(self)

        """"
        elif self.intersection_line:
            # Calculate the target distance
            cars_waiting = trf.junctions[self.last_intersection].entries[
                trf.angle_to_intersect[self.angle]
            ]
            cars_before = 0
            for i in cars_waiting:
                if i == self.id:
                    break
                cars_before += 1

            car_line = cars_before * (stgs.car_len + parking.min_park_dist)
            target_dist = car_line + stgs.node_width / 2 + self.len / 2

            dist_to_travel = (
                trf.intersection_dist(self.last_intersection, self.pos, self.angle)
                - target_dist
            )

            if dist_to_travel > 0:
                if dist_to_travel < self.speed:
                    self.move_forward(dist_to_travel)
                else:
                    self.move_forward(self.speed)
            else:
                # if dist_to_travel < 0:
                #    print("UMMMMMM")
                if (
                    trf.intersection_dist(self.last_intersection, self.pos, self.angle)
                    == self.len / 2 + stgs.node_width / 2
                ):
                    self.intersection_line = False
                    junction_data.add_car_queue(*self.junction_id)

            # Make it advance to it
            # If at the front of the queue not in line anymore
        
        return
        """

    def exit_intersection(self):
        self.turn_state = 0
        self.path.pop(0)
        self.start_nodes = (self.last_intersection, self.path[0])
        self.center_to_road()

        trf.junctions[self.last_intersection].crossing.remove(self.junction_id)

        # Add to road data structure
        my_dir = pf.angle_to_dir[self.angle]
        intersection_from = self.start_nodes[0]
        trf.roads[(intersection_from, my_dir)].add_car(self)

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


# Parked
class Parking_Lot:
    min_park_dist = stgs.car_len * 0.3
    dist_from_road = stgs.node_width / 2

    def __init__(self):
        self.data = {}

    def __call__(self):
        return self.data

    def __getitem__(self, key):
        return self.data[key]

    def can_park(self, edge, pos, car_length, ID):
        if not edge in self().keys():
            return True
        # Check collisions
        # horizontal or vertical
        coord_index = 0 if edge[0][1] == edge[1][1] else 1

        # [0] pos [1] car_len [2] id
        for park in self()[edge]:
            dist = abs(park[0][coord_index] - pos[coord_index])
            min_dist = park[1] / 2 + car_length / 2 + self.min_park_dist

            """
            if park[2] == ID:
                print("ok...")
                print(pos, park[0])
                if park[0] == pos:
                    print("YOOO")
            """

            if dist < min_dist and ID != park[2]:
                return False

        return True

    """
    def is_cars_spot(self, edge, pos, car_length, ID):
        if not edge in self().keys():
            return False
        # Check collisions
        # horizontal or vertical
        coord_index = 0 if edge[0][1] == edge[1][1] else 1

        # [0] pos [1] car_len [2] id
        for park in self()[edge]:
            if ID == park[2]:
                dist = abs(park[0][coord_index] - pos[coord_index])
                min_dist = park[1] / 2 + car_length / 2 + self.min_park_dist

                # if park[2] == ID and park[0] == pos:
                #    print("YOOO")

                if dist < min_dist:
                    return True

        return False
    """

    def add_car(self, edge, pos, car_length, ID):
        if not edge in self().keys():
            self.data[edge] = []
        self.data[edge].append((pos, car_length, ID))

    def delete_car(self, edge, pos):
        if edge in self().keys():
            for i in range(len(self()[edge])):

                if self()[edge][i][0] == pos:
                    self.data[edge].pop(i)
                    if len(self.data[edge]) == 0:
                        del self.data[edge]
                    return

        raise Exception("ERROR: Car described doesn't exist")


parking = Parking_Lot()
