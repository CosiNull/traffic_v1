import road_visuals as rv
import settings as stgs
import pickle

running = True
print(stgs.func)
while running:

    rv.pg.display.update()
    rv.background()
    rv.draw_graph()
    rv.draw_cars()
    rv.check_keys()

    for i in range(stgs.play_speed):
        stgs.time += 1
        rv.update_cars()

    if stgs.count == stgs.num_car:
        break
