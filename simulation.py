import road_visuals as rv
import settings as stgs
import cars

c = 0
running = True
while running:
    stgs.time += 1
    rv.pg.display.update()
    rv.background()
    rv.draw_graph()
    rv.draw_cars()

    rv.check_keys()

    if c > 180:
        rv.update_cars()
    c += 1
