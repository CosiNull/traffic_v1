# Settings
import settings as stgs
import cars as c
import pathfinding as pf
import traffic as trf
import random as rdn
import parking as pk

rdn.seed(stgs.seed)

drawing = False

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

cars = [
    c.Car(i, rdn.choice(colors), parktime=rdn.randint(0, 500), autonomous=False)
    for i in range(stgs.num_car)
]  # 300


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
        stgs.move_y += stgs.move_speed
    if keys[pg.K_a] or keys[pg.K_LEFT]:
        stgs.move_x += stgs.move_speed
    if keys[pg.K_s] or keys[pg.K_DOWN]:
        stgs.move_y -= stgs.move_speed
    if keys[pg.K_d] or keys[pg.K_RIGHT]:
        stgs.move_x -= stgs.move_speed
    if keys[pg.K_z] and stgs.scale_factor < stgs.max_scale:
        stgs.scale_factor += stgs.scale_speed
    if keys[pg.K_x] and stgs.scale_factor > stgs.min_scale:
        stgs.scale_factor -= stgs.scale_speed


# Position handling
def adjust_pos(node=tuple, moveToMiddle=False):
    x = node[0]
    y = node[1]

    # Adjust to corner
    if moveToMiddle:
        x -= stgs.node_width / 2
        y -= stgs.node_width / 2

    # Position change
    x += stgs.move_x
    y += stgs.move_y

    # Scaling
    x -= stgs.width / 2
    y -= stgs.height / 2

    x *= stgs.scale_factor
    y *= stgs.scale_factor

    x += stgs.width / 2
    y += stgs.height / 2

    return (x, y)


def in_screen(node=tuple):
    h_n_w = stgs.node_width * stgs.scale_factor / 2
    return (
        node[0] + h_n_w >= 0
        and node[0] - h_n_w <= stgs.width
        and node[1] + h_n_w >= 0
        and node[1] - h_n_w <= stgs.height
    )


def line_in_rect(node1=tuple, node2=tuple):
    if node1[0] == node2[0]:  # Vertical (x the same)
        h_n_w = stgs.node_width * stgs.scale_factor / 2
        return (
            node1[0] + h_n_w >= 0
            and node1[0] - h_n_w <= stgs.width
            and not (
                (node1[1] - h_n_w >= stgs.height and node2[1] - h_n_w >= stgs.height)
                or (node1[1] + h_n_w <= 0 and node2[1] + h_n_w <= 0)
            )
        )
    elif node1[1] == node2[1]:  # Horizontal (y the same)
        h_n_w = stgs.node_width * stgs.scale_factor / 2
        return (
            node1[1] + h_n_w >= 0
            and node1[1] - h_n_w <= stgs.height
            and not (
                (node1[0] - h_n_w >= stgs.width and node2[0] - h_n_w >= stgs.width)
                or (node1[0] + h_n_w <= 0 and node2[0] + h_n_w <= 0)
            )
        )
    else:
        raise ValueError("ERROR: NON VALID NODE CONNECTION")


# Roads_________________________________________________________________________________________
def background():
    color = (213, 216, 219)
    screen.fill(color)


def draw_node(node=tuple):
    n_w = round(stgs.node_width * stgs.scale_factor)
    x, y = adjust_pos(node, True)
    color = (10, 10, 10)
    if node == (1204, 752):
        color = (200, 0, 0)
    if in_screen((x, y)):
        pg.draw.rect(screen, color, pg.Rect(x, y, n_w, n_w))


def draw_road(node1, node2):
    pos1 = adjust_pos(node1)
    pos2 = adjust_pos(node2)
    seperation_color = (232, 232, 232)

    color = (0, 0, 0)

    if line_in_rect(pos1, pos2):
        pg.draw.line(
            screen, color, pos1, pos2, round(stgs.node_width * stgs.scale_factor)
        )
        pg.draw.line(
            screen,
            seperation_color,
            pos1,
            pos2,
            round(stgs.node_width * stgs.scale_factor / 10),
        )


def draw_graph():
    visited = set()

    def dfs(node=tuple):
        if not node in visited:
            visited.add(node)
            for neighbour in trf.road_network.connections[node]:
                draw_road(node, neighbour[0])
                dfs(neighbour[0])

    dfs(next(iter(trf.road_network.nodes)))
    [draw_node(x) for x in trf.road_network.nodes]


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
        if car.pause:
            continue
        if car.state == 0:
            if car.path == None:
                if car.park_time > 0:
                    car.park_time -= 1
                else:
                    goal = pf.rand_node(trf.road_network)
                    true_goal = rdn.choice(trf.road_network.connections[goal])[0]
                    while goal in {
                        car.start_nodes[0],
                        car.start_nodes[1],
                    } or true_goal in {
                        car.start_nodes[0],
                        car.start_nodes[1],
                    }:  # The while exists to avoid a strange bug where the cars can't identify the road that it is in
                        goal = pf.rand_node(trf.road_network)
                        true_goal = rdn.choice(trf.road_network.connections[goal])[0]
                    park_slot = pk.generate_goal(car.id, true_goal, goal)
                    if park_slot == None:
                        continue
                    dist, target_pos = park_slot
                    car.find_path(goal, true_goal, cars, dist, target_pos)
            else:
                car.enter_road()
        else:
            car.update()


# Initializing pygame_________________________________________________________
import pygame as pg

pg.init()
screen = pg.display.set_mode([stgs.width, stgs.height])
pg.display.set_caption("Simulation")
pg.event.set_allowed([pg.QUIT, pg.KEYDOWN, pg.KEYUP])
