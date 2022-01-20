import road_visuals as rv
import settings as stgs
import pickle

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

    if stgs.time > 15000:
        break

file_to_store = open("1NORM32200.pickle", "wb")
pickle.dump(stgs.car_evolution, file_to_store)
file_to_store.close()
