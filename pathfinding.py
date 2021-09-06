import random as rdn
from settings import *
from typing import Tuple
import math

# Get random node
def rand_node(graph):
    return list(graph.nodes)[rdn.randint(0, len(graph.nodes) - 1)]


def rand_graph_pos(graph, car_len):
    # Choose random node
    node = rand_node(graph)
    # Choose Connection
    connection = graph.connections[node][
        rdn.randint(0, len(graph.connections[node]) - 1)
    ]
    to = connection[0]
    # Choose start
    start = rdn.randint(
        car_len + node_width, connection[1].length - node_width - car_len
    )

    return (node, to, start)


# Turning objects
def get_abs_direction(start_node: Tuple[float, float], end_node: Tuple[float, float]):
    if start_node[0] == end_node[0]:  # Vertical
        if start_node[1] < end_node[1]:
            return "d"
        else:
            return "u"
    elif start_node[1] == end_node[1]:  # Horizontal
        if start_node[0] < end_node[0]:
            return "r"
        else:
            return "l"
    raise Exception("ERROR: UNVALID SET OF TUPLE PARAMETERS")


angle_to_dir = {0: "r", math.pi / 2: "d", math.pi: "l", math.pi * 3 / 2: "u"}

relative_dir = {
    "u": {"u": "u", "r": "r", "l": "l"},
    "r": {"r": "u", "u": "l", "d": "r"},
    "l": {"l": "u", "u": "r", "d": "l"},
    "d": {"d": "u", "r": "l", "l": "r"},
}

opposite_dir = {"u": "d", "r": "l", "l": "r", "d": "u"}

# PATHFINDING ALGORITHMS--------------------
# Data structures
# Data Tuple:
# 0: Node Coordinate
# 1: F cost or G cost depending on algo
# 2: Time (At some point in my life)
class Priority_Queue:
    def __init__(self):
        self.data = []

    def __call__(self):
        return self.data

    def insert(self, elem: tuple[tuple:[float, float], float]):
        self.data.append(elem)

    def poll(self):
        # METHOD 1
        # best = 0
        # best_dist = self()[0][1]
        # for i in range(1, len(self())):
        #    dist = self()[i][1]
        #    if dist < best_dist:
        #        best_dist = dist
        #        best = i
        # return self().pop(best)

        # METHOD 2
        # best = min(self(), key=lambda x: x[1])
        # return self().pop(self().index(best))

        # METHOD 3
        sorted(self(), key=lambda x: x[1])
        return self().pop(0)


class Dheap:
    def __init__(self, dnodes):
        self.dnodes = dnodes
        self.data = []

    def __call__(self):
        return self.data

    def get_child_index(self, parent_index, nth_child):
        return self.dnodes * parent_index + nth_child

    def get_parent_index(self, child_index):
        return math.floor((child_index - 1) / self.dnodes)

    def swap(self, index1, index2):
        self.data[index1], self.data[index2] = self.data[index2], self.data[index1]

    def insert(self, elem):
        self.data.append(elem)
        my_index = len(self.data) - 1

        while my_index > 0:
            par_ind = self.get_parent_index(my_index)
            if self()[par_ind][1] < self()[my_index][1]:
                break
            else:
                self.swap(my_index, par_ind)
                my_index = par_ind

    def poll(self):
        self.swap(0, -1)
        retrieved_data = self.data.pop()

        # Bubble down
        current_ind = 0
        list_len = len(self())
        while self.get_child_index(current_ind, 1) < list_len:
            # Get minimum key
            first_ind = self.get_child_index(current_ind, 1)
            best_ind = first_ind
            best_dist = self()[best_ind][1]
            for i in range(1, self.dnodes):
                if first_ind + i < list_len and self()[first_ind + i][1] < best_dist:
                    best_dist = self()[first_ind + i][1]
                    best_ind = first_ind + i
            # Swap if bigger than child
            if self()[current_ind][1] > best_dist:
                self.swap(current_ind, best_ind)
                current_ind = best_ind
            else:
                break

        return retrieved_data


# _________________________________________________________
def dijkstra(graph, start=tuple[float, float], end=tuple[float, float], start_dir=str):
    dist = {node: math.inf for node in graph.nodes}
    vis = set()
    prev = {}

    dist[start] = 0
    pq = Priority_Queue()
    pq.insert((start, 0))

    while len(pq()) > 0:
        data = pq.poll()
        node = data[0]
        dest_dist = data[1]

        if node == end:
            break
        elif node in vis:
            continue

        vis.add(node)

        for connection in graph.connections[node]:

            neighbour = connection[0]

            if neighbour in prev and prev[node] == neighbour:
                # print(node, prev[node], neighbour)
                continue
            elif len(vis) == 1 and opposite_dir[start_dir] == get_abs_direction(
                node, neighbour
            ):
                continue

            if neighbour in vis:
                continue

            current_dist = dest_dist + connection[1].length
            if current_dist < dist[neighbour]:
                dist[neighbour] = current_dist
                pq.insert((neighbour, current_dist))
                prev[neighbour] = node

    return (dist, prev)


# To do today
# Check if it is faster with dary heap and find a way to adjust d programatically
# Make the cars follow a pathfinding
# Think of a data structure that holds the current passings
# Think of a data structure that holds the future passings
# _________________________________________________________________________
# A*
def manhattan_dist(start, end):
    return (start[0] - end[0]) + (start[1] - end[1])


def a_star(graph, start=tuple[float, float], end=tuple[float, float], start_dir=str):
    dist = {node: math.inf for node in graph.nodes}
    prev = {}

    dist[start] = 0
    pq = Priority_Queue()
    pq.insert((start, 0))

    while len(pq()) > 0:
        node = pq.poll()[0]
        if node == end:
            break
        for neighbour, edge in graph.connections[node]:
            if neighbour in prev and prev[node] == neighbour:
                continue
            elif len(prev.keys()) == 0 and opposite_dir[start_dir] == get_abs_direction(
                node, neighbour
            ):
                continue
            current_dist = dist[node] + edge.length
            if current_dist < dist[neighbour]:
                dist[neighbour] = current_dist
                pq.insert((neighbour, current_dist + manhattan_dist(neighbour, end)))
                prev[neighbour] = node

    return (dist, prev)


# _________________________________________________________________________
def reverse_path(distances, previous, start, end):
    dist = distances[end]

    current_node = end
    path = [end]

    while current_node != start:
        current_node = previous[current_node]
        path.insert(0, current_node)

    return (path, dist)


#
def pathfind_dj(graph, start, end, start_dir):
    return reverse_path(*dijkstra(graph, start, end, start_dir), start, end)


def pathfind_as(graph, start, end, start_dir):
    return reverse_path(*a_star(graph, start, end, start_dir), start, end)
