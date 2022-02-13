import pickle


with open(f"timed_test/dj{1000}.pickle", "rb") as f:
    dj = pickle.load(f)

with open(f"timed_test/mlt{1000}.pickle", "rb") as f:
    mlt = pickle.load(f)

improvements = []
print(dj, mlt)

for time_d, time_m in zip(dj, mlt):
    improvement = (time_d - time_m) / time_d
    improvements.append(improvement)

print(sum(improvements) / len(improvements))
