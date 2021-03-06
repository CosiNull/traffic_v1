import random as rdn
import settings as stgs
import math

rdn.seed(stgs.seed)

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
        car_len + stgs.node_width, connection[1].length - stgs.node_width - car_len
    )

    return (node, to, start)


# Turning objects
def get_abs_direction(start_node=tuple, end_node=tuple):
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

    def insert(self, elem=tuple):
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


def dijstrka(graph, start, end, start_dir, true_goal):
    dist = {node: math.inf for node in graph.nodes}
    prev = {}

    dist[start] = 0
    pq = Priority_Queue()
    pq.insert((start, 0))

    while len(pq()) > 0:
        node = pq.poll()[0]
        if node == end:
            return (dist, prev)

        for neighbour, edge in graph.connections[node]:
            if node in prev and prev[node] == neighbour:
                continue
            elif node == start and opposite_dir[start_dir] == get_abs_direction(
                node, neighbour
            ):
                continue
            elif node == true_goal and neighbour == end:  # No U_turns towards the goal
                continue

            current_dist = dist[node] + edge.length
            if current_dist < dist[neighbour]:
                dist[neighbour] = current_dist
                pq.insert((neighbour, current_dist))
                prev[neighbour] = node

    return (None, None)


# _________________________________________________________________________
# A*
def manhattan_dist(start, end):
    return (start[0] - end[0]) + (start[1] - end[1])


"""
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
            if node in prev and prev[node] == neighbour:
                continue
            elif node == start and opposite_dir[start_dir] == get_abs_direction(
                node, neighbour
            ):
                continue

            current_dist = dist[node] + edge.length
            if current_dist < dist[neighbour]:
                dist[neighbour] = current_dist
                pq.insert((neighbour, current_dist + manhattan_dist(neighbour, end)))
                prev[neighbour] = node

    return (dist, prev)
"""


def a_star(graph, start, end, start_dir, true_goal):
    dist = {node: math.inf for node in graph.nodes}
    prev = {}

    dist[start] = 0
    pq = Priority_Queue()
    pq.insert((start, 0))

    while len(pq()) > 0:
        node = pq.poll()[0]
        if node == end:
            return (dist, prev)

        for neighbour, edge in graph.connections[node]:
            if node in prev and prev[node] == neighbour:
                continue
            elif node == start and opposite_dir[start_dir] == get_abs_direction(
                node, neighbour
            ):
                continue
            elif node == true_goal and neighbour == end:  # No U_turns towards the goal
                continue

            current_dist = dist[node] + edge.length
            if current_dist < dist[neighbour]:
                dist[neighbour] = current_dist
                pq.insert((neighbour, current_dist + manhattan_dist(neighbour, end)))
                prev[neighbour] = node

    return (None, None)


# _________________________________________________________________________
def reverse_path(distances, previous, start, end):
    if distances == None:
        return (None, None)

    dist = distances[end]

    current_node = end
    path = [end]

    while current_node != start:
        current_node = previous[current_node]
        path.insert(0, current_node)

    return (path, dist)


#
def pathfind_dj(graph, start, end, start_dir, true_goal):
    return reverse_path(*dijstrka(graph, start, end, start_dir, true_goal), start, end)


def pathfind_as(graph, start, end, start_dir, true_goal):
    return reverse_path(*a_star(graph, start, end, start_dir, true_goal), start, end)
