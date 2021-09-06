import pygame as pg
from graph import *
import pickle

width = 1100
height = 700

pg.init()
screen = pg.display.set_mode((width, height))
graph = {}
with open("test3.pickle", "rb") as input_file:
    graph = pickle.load(input_file)

print((0, 0) in graph.nodes)

running = True


def visualize_graph(graph):
    visited = {}
    node = next(iter(graph.nodes))

    def dfs(current_node: tuple[float, float], graph):
        if not current_node in visited.keys():
            pg.draw.circle(screen, (255, 0, 0), current_node, 2)
            visited[current_node] = True
            # print(f"{current_node} : {graph.connections[current_node]}")
            for node_, length in graph.connections[current_node]:
                pg.draw.line(screen, (255, 255, 255), current_node, node_, 2)
                dfs(node_, graph)

    dfs(node, graph)


while running:
    for event in pg.event.get():
        if event.type == pg.QUIT:
            running = False

    # Game Loop
    pg.display.update()
    # screen.fill((0, 0, 0))

    for node in graph.nodes:
        pg.draw.circle(screen, (0, 255, 0), node, 4)

    visualize_graph(graph)
