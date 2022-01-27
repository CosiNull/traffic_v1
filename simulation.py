import road_visuals as rv
import settings as stgs
import pickle
import parking as pk
import traffic as trf

running = True
print(stgs.func)

"""
while running:

    rv.pg.display.update()
    rv.background()
    rv.draw_graph()
    rv.draw_cars()
    rv.check_keys()
"""

for terrain_seed in range(10):
    with open(f"terrains/{terrain_seed}.pickle", "rb") as f:
        trf.road_network = pickle.load(f)
    stgs.car_evolution = []

    for seed in range(10):
        rv.rdn.seed(seed)
        stgs.time = 0
        stgs.count = 0
        pk.parking = pk.Parking_Lot()
        trf.junctions = trf.make_intersections_dict(trf.road_network)
        trf.roads = trf.make_roads_dict(trf.road_network)

        rv.cars = [
            rv.c.Car(
                i, rv.rdn.choice(rv.colors), rv.rdn.randint(0, 700), autonomous=False
            )
            for i in range(stgs.num_car)
        ]

        while running:
            for i in range(1):
                stgs.time += 1
                rv.update_cars()

            if stgs.count >= stgs.num_car:
                break
            elif stgs.time >= 8000:
                print()
                print(f"To Exclude seed:{seed}")
                print()
                break

        print(terrain_seed, seed, stgs.time)
        stgs.car_evolution.append(stgs.time)

    file_to_store = open(f"timed_test/{stgs.func}{terrain_seed}.pickle", "wb")
    pickle.dump(stgs.car_evolution, file_to_store)
    file_to_store.close()


print("done!")
