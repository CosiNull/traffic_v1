import pickle
import numpy as np
import matplotlib.pyplot as plt

seed = 420

with open(f"data/dj{seed}.pickle", "rb") as f:
    x1 = np.array(pickle.load(f))
y1 = np.arange(1, len(x1) + 1)

plt.plot(x1, y1, label="Dijkstra")

with open(f"data/mlt{seed}.pickle", "rb") as f:
    x2 = np.array(pickle.load(f))
y2 = np.arange(1, len(x2) + 1)

plt.plot(x2, y2, label="Multi-agent A*")

plt.title("Number of Cars Arrived Per Units of Time")
plt.ylabel("Cars Arrived")
plt.xlabel("Units of Time")
plt.legend(loc="upper left")

plt.show()
