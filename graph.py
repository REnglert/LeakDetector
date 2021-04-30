import numpy as np
import matplotlib.pyplot as plt

my_file = open("~/screenlog.0", "r")
content_list = my_file.readlines()

a = []
for i in content_list:
    i = i.rstrip('\n')
    if len(i) == 0:
        continue
    if int(i) >= 2000:
        continue
    a.append(int(i))

# Most common number
print(max(set(a), key=a.count))

# Define thresholds to plot
b = []
c = []
d = []
e = []
for _ in range(0, len(a)):
    b.append(int(15))
    c.append(int(50))
    d.append(int(90))
    e.append(int(200))

front = 33000
back = 40000

# Max val in range to help with determining thresholds
print(max(a[front:back]))

plt.plot(a[front:back], linestyle='dotted')

plt.plot(b[front:back], color='red', linestyle='-', label="quiet: [0, 15)")
plt.plot(c[front:back], color='green', linestyle='-', label="sink: [15, 50)")
plt.plot(d[front:back], color='orange',
         linestyle='-', label="toilet: (50, 90]")
plt.plot(e[front:back], color='purple', linestyle='-', label="tub: (90, 200]")

plt.xlabel('Time')
plt.ylabel('Amplitude')
plt.legend()
plt.show()
