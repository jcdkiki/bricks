from matplotlib import pyplot as plt
import sys

names = []
files = []
for i in range(1, len(sys.argv)):
    names.append(sys.argv[i])
    files.append(open(sys.argv[i]).readlines())

plots = []

for f in files:
    xs = []
    ys = []
    for line in f:
        x, y = map(float, line.split())
        xs.append(x)
        ys.append(y)

    plots.append((xs, ys))

#plt.xscale("log")
plt.yscale("log")

for i, p in enumerate(plots):
    plt.plot(p[0], p[1], label=names[i])

plt.legend()
plt.savefig("plot.png")