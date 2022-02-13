import pickle


averages = []

for terrain in range(10):

    with open(f"timed_test/dj{terrain}.pickle", "rb") as f:
        dj = pickle.load(f)

    with open(f"timed_test/mlt{terrain}.pickle", "rb") as f:
        mlt = pickle.load(f)

    improvements = []
    for time_d, time_m in zip(dj, mlt):
        improvement = (time_d - time_m) / time_d
        improvements.append(improvement)

    averages.append(sum(improvements) / len(improvements))

print()
print(averages)
print()
print(sum(averages) / len(averages))
