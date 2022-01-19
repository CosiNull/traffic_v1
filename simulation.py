import road_visuals as rv
import settings as stgs

running = True
while running:

    rv.pg.display.update()
    rv.background()
    rv.draw_graph()
    rv.draw_cars()
    rv.check_keys()

    for i in range(stgs.play_speed):
        stgs.time += 1
        rv.update_cars()

    if stgs.time > 4300:
        stgs.play_speed = 1
