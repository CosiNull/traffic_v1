import road_visuals as rv
import settings as stgs
import future as fut
import cars

running = True
while running:
    stgs.time += 1

    rv.pg.display.update()
    rv.background()
    rv.draw_graph()
    rv.draw_cars()
    rv.check_keys()

    if stgs.time > 180:
        rv.update_cars()

    """
    if stgs.time % 150 == 149:
        for road in fut.roads.values():
            road.update_road()
        for intersection in fut.junctions.values():
            intersection.update_intersection()
    """
