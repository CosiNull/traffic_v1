import settings as stgs

import math
import random as rdn
import copy as cpy
import pathfinding as pf

# Local settings
rdn.seed(1)
car_width = int((stgs.node_width / 2) * 0.5)
colors = [
    (255, 0, 0),
    (0, 100, 0),
    (0, 0, 255),
    (255, 0, 255),
    (255, 255, 0),
    (0, 255, 255),
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
        self.speed = 0.5
        self.park_speed = 0.4

        self.path = None
        self.park_time = 0
        self.goal = 0

        self.pause = False

    def set_pos(self, graph, random=True):
        if random:
            node, to, start = pf.rand_graph_pos(graph, self.len)
        else:

            node, to = self.start_nodes

            # But why?
            # if node[0] == to[0]:
            #    node, to = to, node

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

        #
        if not parking.can_park(self.start_nodes, self.pos, self.len, self.id):
            # self.set_pos(graph, True)
            print("Place Back")
            # return
            pass

        parking.add_car(self.start_nodes, self.pos, self.len, self.id)

        #
        self.init_pos = self.pos

    def enter_road(self):
        if not self.state == 0:
            raise Exception("ERROR: Car cannot exit parking that it already exited")

            # print(self.goal)
        self.state = 1
        self.gas = 0
        self.turn_state = 0

    def find_path(self, goal, func="dj"):
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

        self.target_pos = tuple(pos)

        if parking.can_park(edge, pos, self.len, self.id):
            parking.add_car(edge, pos, self.len, self.id)
        else:
            self.path = None
            # print("Spot already reserved in advance")
            return

        def add_dir(path_list, index, abs_curr_dir):
            if index >= len(path_list) - 1:
                return
            dir = pf.get_abs_direction(path_list[index], path_list[index + 1])

            turn = pf.relative_dir[abs_curr_dir][dir]

            path_list.insert(index + 1, (turn))
            add_dir(path_list, index + 2, dir)

        add_dir(self.path, 0, start_dir)

    # The Holy Update Method
    def update(self):
        if self.pause:
            return

        if self.state == 1:  # Exit parking
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

            else:
                # NOTE ADD CAR IN PARKING DATABASE

                self.set_pos(self.graph, False)
                self.pos = (round(self.pos[0]), round(self.pos[1]))

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

    def move_to_dest(self):  # NOTE: UNFINISHED
        # Move Forward
        if len(self.path) == 0:
            # self.color = (255, 255, 255)
            self.state = 3
        elif not type(self.path[0]) == str:
            self.advance_to_dest()
        else:
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

        # Dont forget to include when there is a car in front to stop

        # To understand the dist formula and not spend 2 minutes staring at it, Imagine that the car is going to the right
        if not len(self.path) == 1:
            dist = (
                self.path[0][unsame_indexes]
                - (stgs.node_width / 2 + self.len / 2) * pos_angle
            ) - (self.pos[unsame_indexes])
            dist = abs(dist)
        else:
            dist = (
                self.path[0][unsame_indexes] - (self.goal + stgs.park_dist) * pos_angle
            ) - (self.pos[unsame_indexes])
            dist = abs(dist)
            # dist -= self.goal

        if dist == 0:
            self.turn_state = 0
            self.last_intersection = self.path.pop(0)

        elif dist <= self.speed:
            self.turn_state = 0
            self.move_forward(dist)
            self.last_intersection = self.path.pop(0)
            # self.save = (self.pos, self.gas)
        else:
            self.move_forward(self.speed)

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

    # intersections
    def intersect_forward(self):
        # Distance: 24.0, Gas: 49
        if self.turn_state < math.ceil((stgs.node_width + self.len) / self.speed):
            self.move_forward(self.speed)
            self.turn_state += 1
        else:

            self.turn_state = 0
            self.path.pop(0)
            self.start_nodes = (self.last_intersection, self.path[0])

            self.center_to_road()

    right_turn_deviation = 0.075

    def intersect_right(self):
        # different-axis Distance: 6.914533413670597, same-axis distance: 6.441554850340253, Gas: 22
        if self.angle - self.road_angle < math.pi / 2:
            self.move_forward(self.speed)
            self.angle += self.right_turn_deviation
        else:

            self.angle = (self.road_angle + math.pi / 2) % (math.pi * 2)
            self.turn_state = 0
            self.path.pop(0)

            self.start_nodes = (self.last_intersection, self.path[0])

            self.road_angle = self.angle

            # if self.road_angle == 0 or self.road_angle == math.pi:
            #    print(
            #        f"Distance: {self.pos[0]-self.save[0][0]}, Gas: {self.gas-self.save[1]}"
            #    )
            self.center_to_road()

    left_turn_deviation = 0.032

    def intersect_left(self):
        # Same-axis Distance: 15.829976845746955, Different-axis distance: 15.87430464054819, Gas: 51
        if self.road_angle - self.angle < math.pi / 2:
            self.move_forward(self.speed)
            self.angle -= self.left_turn_deviation
        else:
            self.angle = (self.road_angle - math.pi / 2) % (math.pi * 2)
            self.turn_state = 0
            self.path.pop(0)

            self.start_nodes = (self.last_intersection, self.path[0])

            self.road_angle = self.angle

            # if self.road_angle == 0 or self.road_angle == math.pi:
            #    print(
            #        f"Distance: {self.pos[1]-self.save[0][1]}, Gas: {self.gas-self.save[1]}"
            #    )
            self.center_to_road()

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

            # if park[2] == ID and park[0] == pos:
            #    print("YOOO")

            if dist < min_dist and ID != park[2]:
                return False

        return True

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
