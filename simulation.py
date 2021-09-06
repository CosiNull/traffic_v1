from road_visuals import *
from settings import *
from cars import *

c = 0
running = True
while running:
    pg.display.update()
    background()
    draw_graph()
    draw_cars()

    check_keys()

    if c > 180:
        update_cars()
    c += 1
