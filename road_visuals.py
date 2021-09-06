# Settings
from settings import *
import pickle
import time
from cars import *

drawing = False

# Loading content and generating
with open(road_file, "rb") as f:
    road_network = pickle.load(f)

"""
lights = {
    node: Light(node)
    for node in road_network.nodes
    if len(road_network.connections[node]) > 2
}
"""
cars = [Car(road_network) for i in range(100)]

# Moving Functions
def check_keys():
    global move_x
    global move_y
    global scale_factor

    # Basic exit
    for event in pg.event.get():
        if event.type == pg.QUIT:
            pg.quit()
            # sys.exit() if sys is imported

    # Key presses
    keys = pg.key.get_pressed()
    if keys[pg.K_w] or keys[pg.K_UP]:
        move_y += move_speed
    if keys[pg.K_a] or keys[pg.K_LEFT]:
        move_x += move_speed
    if keys[pg.K_s] or keys[pg.K_DOWN]:
        move_y -= move_speed
    if keys[pg.K_d] or keys[pg.K_RIGHT]:
        move_x -= move_speed
    if keys[pg.K_z] and scale_factor < max_scale:
        scale_factor += scale_speed
    if keys[pg.K_x] and scale_factor > min_scale:
        scale_factor -= scale_speed


# Position handling
def adjust_pos(node=tuple[float, float], moveToMiddle=False):
    x = node[0]
    y = node[1]

    # Adjust to corner
    if moveToMiddle:
        x -= node_width / 2
        y -= node_width / 2

    # Position change
    x += move_x
    y += move_y

    # Scaling
    x -= width / 2
    y -= height / 2

    x *= scale_factor
    y *= scale_factor

    x += width / 2
    y += height / 2

    return (x, y)


def in_screen(node: tuple[float, float]):
    h_n_w = node_width * scale_factor / 2
    return (
        node[0] + h_n_w >= 0
        and node[0] - h_n_w <= width
        and node[1] + h_n_w >= 0
        and node[1] - h_n_w <= height
    )


def line_in_rect(node1: tuple[float, float], node2: tuple[float, float]):
    if node1[0] == node2[0]:  # Vertical (x the same)
        h_n_w = node_width * scale_factor / 2
        return (
            node1[0] + h_n_w >= 0
            and node1[0] - h_n_w <= width
            and not (
                (node1[1] - h_n_w >= height and node2[1] - h_n_w >= height)
                or (node1[1] + h_n_w <= 0 and node2[1] + h_n_w <= 0)
            )
        )
    elif node1[1] == node2[1]:  # Horizontal (y the same)
        h_n_w = node_width * scale_factor / 2
        return (
            node1[1] + h_n_w >= 0
            and node1[1] - h_n_w <= height
            and not (
                (node1[0] - h_n_w >= width and node2[0] - h_n_w >= width)
                or (node1[0] + h_n_w <= 0 and node2[0] + h_n_w <= 0)
            )
        )
    else:
        raise ValueError("ERROR: NON VALID NODE CONNECTION")


# Roads_________________________________________________________________________________________
def background():
    color = (213, 216, 219)
    screen.fill(color)


def draw_node(node=tuple[float, float]):
    n_w = round(node_width * scale_factor)
    x, y = adjust_pos(node, True)
    color = (10, 10, 10)
    if in_screen((x, y)):
        pg.draw.rect(screen, color, pg.Rect(x, y, n_w, n_w))


def draw_road(node1, node2):
    pos1 = adjust_pos(node1)
    pos2 = adjust_pos(node2)
    seperation_color = (232, 232, 232)

    color = (0, 0, 0)

    if line_in_rect(pos1, pos2):
        pg.draw.line(screen, color, pos1, pos2, round(node_width * scale_factor))
        pg.draw.line(
            screen, seperation_color, pos1, pos2, round(node_width * scale_factor / 10)
        )


def draw_graph():
    visited = set()

    def dfs(node=tuple[float, float]):
        if not node in visited:
            visited.add(node)
            for neighbour in road_network.connections[node]:
                draw_road(node, neighbour[0])
                dfs(neighbour[0])

    dfs(next(iter(road_network.nodes)))
    [draw_node(x) for x in road_network.nodes]


# Cars_________________________________________________________________________
def draw_cars():
    for car in cars.copy():
        if car.dead:
            cars.remove(car)
            break

        if in_screen(adjust_pos(car.pos)):
            points = [adjust_pos(point) for point in car.points]
            pg.draw.polygon(screen, car.color, points)


def update_cars():
    for car in cars:
        car.update()
        if car.state == 0:
            if car.path == None:
                if car.park_time > 0:
                    car.park_time -= 1
                else:
                    car.find_path(rand_node(road_network))
            else:
                car.enter_road()


# Initializing pygame_________________________________________________________
import pygame as pg

pg.init()
screen = pg.display.set_mode([width, height])
pg.display.set_caption("Simulation")

"""
res1 = []
res2 = []
for i in range(1000):
    print()
    start = rand_node(road_network)
    end = rand_node(road_network)

    save = time.time()
    print(pathfind1(road_network, start, end))
    milli1 = 1000 * (time.time() - save)
    print("Djisktra speed: ", milli1)

    save = time.time()
    print(pathfind2(road_network, start, end))
    milli2 = 1000 * (time.time() - save)
    print("A star speed: ", milli2)

    print()
    res1.append(milli1)
    res2.append(milli2)

print("Dijsktra Mean: ", sum(res1) / len(res1))
print("A star Mean: ", sum(res2) / len(res2))
"""
