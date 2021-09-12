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
    def __init__(self, graph):
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

    def set_pos(self, graph, random=True):
        if random:
            node, to, start = pf.rand_graph_pos(graph, self.len)
        else:

            node, to = self.start_nodes
            if node[0] == to[0]:
                node, to = to, node

            # Setting up the start
            unsame_indexes = (
                0 if self.start_nodes[0][0] != self.start_nodes[1][0] else 1
            )
            start = abs(node[unsame_indexes] - self.pos[unsame_indexes])

        # Set Position
        variation = stgs.node_width / 2 - self.width
        variation += Parking_Lot.dist_from_road

        if node[0] == to[0]:  # Vertical (x are the same)
            if node[1] < to[1]:
                self.pos = (node[0] + variation, node[1] + start)
                #
                self.angle = math.pi * 3 / 2
            else:
                self.pos = (node[0] - variation, node[1] - start)
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
        if not parking.can_park(self.start_nodes, self.pos, self.len):
            self.set_pos(graph, True)
            return

        parking.add_car(self.start_nodes, self.pos, self.len)

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
        start = self.start_nodes[0]
        if self.angle == 0 or self.angle == math.pi:
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
        self.goal = 0
        if len(self.path) >= 2:
            u_s = 0 if self.path[-1][0] != self.path[-2][0] else 1
            a = stgs.car_len + stgs.node_width
            b = (
                abs(self.path[-1][u_s] - self.path[-2][u_s])
                - stgs.node_width
                - stgs.car_len
            )
        else:
            u_s = 0 if self.path[-1][0] != self.start_nodes[1][0] else 1
            a = stgs.car_len + stgs.node_width
            b = (
                abs(self.path[-1][u_s] - self.start_nodes[1][u_s])
                - stgs.node_width
                - stgs.car_len
            )
        if a > b:
            self.path = None
            return
        self.goal = rdn.randint(
            a,
            b,
        )

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
        # horizontal = self.start_nodes[0][1] == self.start_nodes[1][1]
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
            else:
                # NOTE ADD CAR IN PARKING DATABASE
                self.set_pos(self.graph, False)
                self.state = 0
                self.path = None
                self.park_time = 200
                self.gas = 0
                pass

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
        pos_angle = 1 if self.angle == 0 or self.angle == math.pi / 2 else -1

        # Dont forget to include when there is a car in front to stop
        dist = (self.pos[unsame_indexes] + self.len / 2 * pos_angle) - (
            self.path[0][unsame_indexes] - stgs.node_width / 2 * pos_angle
        )
        if len(self.path) == 1:
            dist += (stgs.park_dist + self.goal) * pos_angle

        dist = abs(dist)

        if dist > 0:
            if abs(dist) - self.speed <= 0:
                self.turn_state = 0
                self.move_forward(abs(self.speed - dist))
                self.last_intersection = self.path.pop(0)
                # Delete after
                # self.save = (self.pos, self.gas)
            else:
                self.move_forward(self.speed)
        """
        else:
            # Dont forget to include when there is a car in front to stop
            dist = (self.pos[unsame_indexes] - self.len / 2) - (
                self.path[0][unsame_indexes] + node_width / 2
            )
            if len(self.path) == 1:
                dist -= park_dist
            if dist > 0:
                if dist - self.speed <= 0:
                    self.turn_state = 0
                    self.move_forward(self.speed - dist)
                    self.last_intersection = self.path.pop(0)

                    # Delete after
                    # self.save = (self.pos, self.gas)
                else:
                    self.move_forward(self.speed)
        """

    # Movement
    def move_forward(self, speed):
        mov_x = math.cos(self.angle) * speed
        mov_y = math.sin(self.angle) * speed

        self.pos = (self.pos[0] + mov_x, self.pos[1] + mov_y)

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

    def can_park(self, edge, pos, car_length):
        if not edge in self().keys():
            return True
        # Check collisions
        # horizontal or vertical
        coord_index = 0 if edge[0][1] == edge[1][1] else 1

        for park in self()[edge]:
            dist = abs(park[0][coord_index] - pos[coord_index])
            min_dist = park[1] / 2 + car_length / 2 + self.min_park_dist
            if dist < min_dist:
                return False

        return True

    def add_car(self, edge, pos, car_length):
        if not edge in self().keys():
            self.data[edge] = []
        self.data[edge].append((pos, car_length))

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
