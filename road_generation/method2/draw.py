from math import inf, radians
import pickle
import turtle
from roads import *
import os, sys
import random
import time


p = os.path.abspath(".")
sys.path.insert(1, p)

from graph import *


# Graph
graph = Graph([])
move_x = 1100
move_y = 700


def link_nodes(node1: tuple[float, float], node2: tuple[float, float]):
    node1_ = (round(node1[0] + move_x), round(node1[1] + move_y))
    node2_ = (round(node2[0] + move_x), round(node2[1] + move_y))

    graph.add_2directed_edges(node1_, node2_)


def add_graph_node(x, y):
    graph.add_nodes([(round(x + move_x), round(y + move_y))])


# Set up turtle
drawing = False
screen = turtle.Screen()
screen.bgcolor("black")
screen.title("Road Generation")
turt = turtle.Turtle()
turt.pencolor("white")
turt.speed(10)

# Important Variables
segments = 0
max_segments = 225
road_queue = []
hotspots = []
random.seed("james")  # 2190


# Helper Functions

# Drawing
def teleport(coord: tuple[float, float]):
    turt.penup()
    turt.goto(coord)
    turt.pendown()


def extend_road(road):
    global segments
    road.add_extension()

    x, y, angle = road.last_extension_info()
    angle = round(math.degrees(angle))
    if not road_mem.road_exists(x, y, angle):
        draw_new_road(road.extension[-2], road.extension[-1])

        # if (x, y) in road_mem.data:
        #   if len(road_mem.data[(x, y)]) == 1:
        #       # if abs(angle - list(road_mem.data[(x, y)])[0]) != 180:
        #       road.checkpoints.append((x, y))
        #       link_nodes((x, y), road.checkpoints[-2])
        #   else:
        #       road.checkpoints.append((x, y))
        #       link_nodes((x, y), road.checkpoints[-2])

        # add_graph_node(x, y)
        link_nodes(road.extension[-2], road.extension[-1])

        road_mem.add_road(x, y, angle)
        segments += 1
    else:
        road.extension.pop()


def execute_order(road):
    global segments
    char = road.choose_action()

    if char == "c":
        extend_road(road)
    elif char == "b":
        x, y, angle = road.new_branch_info(random.random() > 0.5)
        if not road_mem.road_exists(x, y, angle):
            # For now just a street
            road_queue.append(Street(x, y, angle))
            road_mem.add_road(x, y, angle)

        extend_road(road)
    elif char == "b2":
        for i in range(2):
            x, y, angle = road.new_branch_info(i == 0)
            if not road_mem.road_exists(x, y, angle):
                # For now just a street
                road_queue.append(Street(x, y, angle))
                road_mem.add_road(x, y, angle)

        extend_road(road)
    elif "s":
        if not (segments < 30 and len(road_queue) == 1):

            road_queue.remove(road)
    elif "t":
        x, y, angle = road.new_branch_info(random.random() > 0.5)

        if not road_mem.road_exists(x, y, angle):
            # For now just a street
            road_queue.append(Street(x, y, angle))
            road_mem.add_road(x, y, angle)

            road_queue.remove(road)
    elif "t2":
        for i in range(2):
            x, y, angle = road.new_branch_info(i == 0)

            if not road_mem.road_exists(x, y, angle):
                # For now just a street
                road_queue.append(Street(x, y, angle))
                road_mem.add_road(x, y, angle)

        road_queue.remove(road)


def draw_new_road(init_coord, new_coord):
    if not drawing:
        return
    teleport(init_coord)
    turt.goto(new_coord)


# Start
road_queue.append(Highway(0, 0, 0))
road_queue.append(Highway(0, 0, 90))
road_queue.append(Highway(0, 0, 180))
road_queue.append(Highway(0, 0, 270))
add_graph_node(0, 0)
road_mem.data[(0, 0)] = set([0, 90, 180, 270])


def create_network():
    while segments < max_segments and len(road_queue) > 0:
        for road in road_queue:
            execute_order(road)


def node_is_connectable(verified_node, neighbour_node, unwanted_dir):
    if (
        verified_node[0] == neighbour_node[0] or verified_node[1] == neighbour_node[1]
    ) and verified_node != neighbour_node:
        if unwanted_dir == "u":
            return verified_node[1] <= neighbour_node[1]
        elif unwanted_dir == "d":
            return verified_node[1] >= neighbour_node[1]
        elif unwanted_dir == "l":
            return verified_node[0] >= neighbour_node[0]
        elif unwanted_dir == "r":
            return verified_node[0] <= neighbour_node[0]
    else:
        return False


def final_connection(node):
    # Save unwanted directions
    node_ = (node[0] - move_x, node[1] - move_y)
    angle = next(iter(road_mem.data[node_]))

    unwanted_dir = "d"
    if angle == 0:
        unwanted_dir = "r"
    elif angle == 90:
        unwanted_dir = "u"
    elif angle == 180:
        unwanted_dir = "l"

    # Go through every existing node
    best_dist = math.inf
    best_node = None
    for other_node in road_mem.data.keys():
        if node_is_connectable(node_, other_node, unwanted_dir):
            dist = abs(node_[0] - other_node[0]) + abs(node_[1] - other_node[1])
            if dist < best_dist:
                best_dist = dist
                best_node = other_node

    if not best_node == None:
        print(node_, best_node)

        # Extend road to node
        angle = None

        if best_node[0] > node_[0] and best_node[1] == node_[1]:
            angle = 0
        elif best_node[0] < node_[0] and best_node[1] == node_[1]:
            angle = 180
        elif best_node[1] > node_[1] and best_node[0] == node_[0]:
            angle = 90
        else:
            angle = 270
        print(angle)
        extension = Road(node_[0], node_[1], angle)
        road_mem.add_road(node_[0], node_[1], angle)

        while extension.extension[-1] != best_node:
            extension.add_extension()
            x, y, angle = extension.last_extension_info()
            angle = round(math.degrees(angle))

            draw_new_road(extension.extension[-2], extension.extension[-1])

            link_nodes(extension.extension[-2], extension.extension[-1])

            road_mem.add_road(x, y, angle)

        # link_nodes(node_, best_node)


def clean_graph():
    # Add final connections

    for node in graph.nodes.copy():
        if len(graph.connections[node]) == 1:
            final_connection(node)

    # Remove unecessary nodes

    rerun = True
    while rerun:
        rerun = False
        for node in graph.nodes.copy():
            if len(graph.connections[node]) == 2:
                connection1 = graph.connections[node][0][0]
                connection2 = graph.connections[node][1][0]
                if connection1[0] == connection2[0] or connection1[1] == connection2[1]:
                    graph.remove_node(node)
                    graph.add_2directed_edges(connection1, connection2)
                    rerun = True
            elif len(graph.connections[node]) == 1:
                rerun = True
                graph.remove_node(node)


create_network()
clean_graph()

file_to_store = open("test2.pickle", "wb")
pickle.dump(graph, file_to_store)
file_to_store.close()


print("Done")
turtle.done()
