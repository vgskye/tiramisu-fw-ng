import matplotlib.pyplot as plt
import re
with open("timings_prec.log") as f:
    timings = [int(l.split()[-2][:-3]) / 16 for l in f.readlines() if l.endswith("/16 μs\n")]
    plt.hist(timings)
plt.show()