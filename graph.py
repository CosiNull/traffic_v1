class Edge:
    def __init__(self, length: float):
        self.length = length


class Node:
    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y


class Graph:
    def __init__(self, nodes: list):
        self.nodes = set(nodes)
        self.connections = {}
        self.node_data = {}

        for node in self.nodes:
            self.connections[node] = list()
            self.node_data[node] = Node(node[0], node[1])

    def add_nodes(self, nodes: list):
        for node in nodes:
            if not node in self.nodes:
                self.nodes.add(node)
                self.connections[node] = list()
                self.node_data[node] = Node(node[0], node[1])

    def add_directed_edge(self, start: tuple, to: tuple):
        distance = ((start[0] - to[0]) ** 2 + (start[1] - to[1]) ** 2) ** 0.5

        if not start in self.nodes:
            self.add_nodes([start])

        if not to in self.nodes:
            self.add_nodes([to])

        if all(map(lambda x: x[0] != to, self.connections[start])):
            self.connections[start].append([to, Edge(distance)])

    def add_2directed_edges(self, node1: tuple, node2: tuple):
        distance = ((node1[0] - node2[0]) ** 2 + (node1[1] - node2[1]) ** 2) ** 0.5

        if not node1 in self.nodes:
            self.add_nodes([node1])

        if not node2 in self.nodes:
            self.add_nodes([node2])

        if all(map(lambda x: x[0] != node2, self.connections[node1])):
            self.connections[node1].append([node2, Edge(distance)])

        if all(map(lambda x: x[0] != node1, self.connections[node2])):
            self.connections[node2].append([node1, Edge(distance)])

    def remove_edge(self, start: tuple, to: tuple):
        newConnectionDict = self.connections[start].copy()
        for index in range(len(self.connections[start])):
            if self.connections[start][index][0] == to:
                newConnectionDict.pop(index)
                break
        self.connections[start] = newConnectionDict

    def remove_node(self, node: tuple):
        self.nodes.remove(node)
        self.connections.pop(node)
        self.node_data.pop(node)

        for existingNode in self.nodes:
            self.remove_edge(existingNode, node)

    def get_edge(self, start: tuple, to: tuple):
        for index in range(len(self.connections[start])):
            if self.connections[start][index][0] == to:
                return self.connections[start][index]

    def d_heap_value(self):
        # d = edges/vertices
        num_edges = 0
        for connections in self.connections.values():
            num_edges += len(connections)

        return round(num_edges / len(self.nodes))
