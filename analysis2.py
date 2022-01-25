import pickle
import numpy as np
import matplotlib.pyplot as plt

seeds = [0, 492, 999, 2015, 69696969]
# [45, 109, 205, 308, 403, 405, 509]
# [0, 492, 999, 2015, 69696969]
# [7, 420, 2000, 6969, 9422]
data = 3

x = range(1, 10101)
ys1 = np.empty((0, 10100))
for seed in seeds:
    with open(f"data{data}/dj{seed}.pickle", "rb") as f:
        x1 = np.array(pickle.load(f))
    y1 = np.arange(1, len(x1) + 1)

    new_y = np.interp(x, x1, y1)
    new_y = new_y.reshape((1, 10100))
    ys1 = np.concatenate((ys1, new_y), axis=0)

plt.plot(x, np.mean(ys1, axis=0), label="Dijkstra")

ys2 = np.empty((0, 10100))
for seed in seeds:
    with open(f"data{data}/mlt{seed}.pickle", "rb") as f:
        x1 = np.array(pickle.load(f))
    y1 = np.arange(1, len(x1) + 1)

    new_y = np.interp(x, x1, y1)
    new_y = new_y.reshape((1, 10100))
    ys2 = np.concatenate((ys2, new_y), axis=0)


plt.plot(x, np.mean(ys2, axis=0), label="HMAA*")


plt.title("Number of Cars Arrived Per Units of Time")
plt.ylabel("Cars Arrived")
plt.xlabel("Units of Time")
plt.legend(loc="upper left")

plt.show()
