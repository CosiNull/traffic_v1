import random
import math

random.seed(121)

road_queue = []
road_len = 52

# Road Class
class Road:
    def __init__(self, x, y, angle):
        self.init_coord = (x, y)
        self.extension = [(x, y)]

        self.angle = math.radians(simplify_angle(angle))
        self.angle_deviation = math.radians(90)
        self.inaccuracy = math.radians(0)
        self.length = road_len

        self.checkpoints = [(x, y)]

    probability = {"c": 1}

    def choose_action(self):
        sum = 0
        prob_list = []
        # Do a prob list and select random num
        for char in self.probability.keys():
            sum += self.probability[char]
            prob_list.append((char, sum))

        random_num = random.uniform(0, prob_list[-1][1])
        # Find good index
        for element in prob_list:
            if random_num <= element[1]:
                print(element[0])
                return element[0]

    def add_extension(self):
        newAngle = self.angle
        newAngle += random.uniform(-self.inaccuracy, self.inaccuracy)

        x = round(math.cos(newAngle) * self.length + self.extension[-1][0])
        y = round(math.sin(newAngle) * self.length + self.extension[-1][1])

        self.extension.append((x, y))

        return (x, y, round(math.degrees(newAngle)))

    def new_branch_info(self, right: bool):
        newAngle = self.angle
        turn = 1 if right else -1

        newAngle += self.angle_deviation * turn
        # newAngle += random.uniform(-self.inaccuracy, self.inaccuracy)

        return (
            self.extension[-1][0],
            self.extension[-1][1],
            round(math.degrees(newAngle)),
        )

    def last_extension_info(self):
        return (self.extension[-1][0], self.extension[-1][1], self.angle)

    def add_checkpoint(self):
        self.checkpoints.append(self.extension[-1])


class Street(Road):
    def __init__(self, x, y, angle):
        super().__init__(x, y, angle)

    probability = {"c": 0.70, "b": 0.11, "b2": 0.04, "s": 0.01, "t": 0.1, "t2": 0.03}


# Hot spots
class Hotspot:
    def __init__(self, x, y, r):
        self.x = x
        self.y = y
        self.r = r


# Highway
class Highway(Road):
    def __init__(self, x, y, angle):
        super().__init__(x, y, angle)
        self.inaccuracy = math.radians(0)

    probability = {
        "c": 0.82,
        "b": 0.1,
        "b2": 0.38,
        "t2": 0.02,
        "s": 0.04,
    }


# Simplify angle
def simplify_angle(angle):
    if angle < 0:
        return simplify_angle(angle + 360)
    elif angle >= 360:
        return simplify_angle(angle - 360)
    else:
        return angle


# Already built roads data stucture
class Road_Data:
    data = {}

    def add_road(self, x, y, angle):
        if not (x, y) in self.data.keys():
            self.data[(x, y)] = set([simplify_angle(angle)])
        else:
            self.data[(x, y)].add(simplify_angle(angle))

    def road_exists(self, x, y, angle):
        coordinate_exists = (x, y) in self.data.keys()

        if coordinate_exists:
            return simplify_angle(angle) in self.data[(x, y)]
        else:
            return False


road_mem = Road_Data()
