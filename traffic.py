# System |Initial Char: From| Second Char: To|
# Not including same direction turns because it might create a collision with the speed difference
no_conflicts = {
    # _________________________ UP
    ("u", "d"): {
        ("d", "u"),
        ("d", "r"),
        ("r", "u"),
        ("u", "d"),
    },  # From Up go straight
    ("u", "l"): {
        ("d", "r"),
        ("d", "u"),
        ("r", "u"),
        ("r", "d"),
        ("l", "r"),
        ("l", "u"),
        ("l", "d"),
        ("u", "l"),
    },  # From Up turn right
    ("u", "r"): {
        ("d", "l"),
        ("r", "u"),
        ("l", "d"),
        ("u", "r"),
    },  # From Up turn left
    # __________________________ RIGHT
    ("r", "l"): {
        ("l", "r"),
        ("l", "d"),
        ("u", "r"),
        ("r", "l"),
    },  # From down go straight
    ("r", "u"): {
        ("d", "r"),
        ("d", "l"),
        ("l", "r"),
        ("l", "d"),
        ("u", "l"),
        ("u", "r"),
        ("u", "d"),
        ("r", "u"),
    },  # From down turn right
    ("r", "d"): {
        ("d", "r"),
        ("l", "u"),
        ("u", "l"),
        ("r", "d"),
    },  # From right turn left
    # _______________________________DOWN
    ("d", "u"): {
        ("u", "d"),
        ("u", "l"),
        ("l", "d"),
        ("d", "u"),
    },  # From down go straight
    ("d", "r"): {
        ("u", "l"),
        ("u", "d"),
        ("l", "u"),
        ("l", "d"),
        ("r", "u"),
        ("r", "l"),
        ("r", "d"),
        ("d", "r"),
    },  # From down move right
    ("d", "l"): {
        ("u", "r"),
        ("l", "d"),
        ("r", "u"),
        ("d", "l"),
    },
    # __________________________________LEFT
    ("l", "r"): {
        ("r", "l"),
        ("r", "u"),
        ("d", "l"),
        ("l", "r"),
    },  # From down go straight
    ("l", "d"): {
        ("u", "l"),
        ("u", "r"),
        ("u", "l"),
        ("r", "u"),
        ("d", "r"),
        ("d", "l"),
        ("d", "u"),
        ("l", "d"),
    },  # From down turn right
    ("l", "u"): {
        ("u", "l"),
        ("r", "d"),
        ("d", "r"),
        ("l", "u"),
    },  # From right turn left
}


class Intersection:
    def __init__(self, x, y):
        self.x = x
        self.y = y

        self.entries = {"r": [], "l": [], "u": [], "d": []}
        self.crossing = []
