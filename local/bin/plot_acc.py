#!/usr/bin/env python2
import matplotlib.pyplot as plt
import numpy as np
import sys

if len(sys.argv) != 2:
   print("Usage: %s <imu-output>" % sys.argv[0])
   sys.exit(1)

acc_x = []
acc_y = []
acc_z = []
infile = sys.argv[1]
with open(infile, 'r') as f:
   contents = f.readlines()
   for line in contents:
      if len(line) < 5:
         continue
      toks = line.split(',')
      acc_x.append(float(toks[3]))
      acc_y.append(float(toks[4]))
      acc_z.append(float(toks[5]))

low_x = min(acc_x)
high_x = max(acc_x)
low_y = min(acc_y)
high_y = max(acc_y)
low_z = min(acc_z)
high_z = max(acc_z)

x_label = "X min=%f max=%f range=%f" % (low_x, high_x, high_x-low_x)
y_label = "Y min=%f max=%f range=%f" % (low_y, high_y, high_y-low_y)
z_label = "Z min=%f max=%f range=%f" % (low_z, high_z, high_z-low_z)
plt.plot(acc_x, acc_y)
plt.ylabel(y_label)
plt.xlabel(x_label)
print(x_label)
print("  scale: %f" % (1.0 / (high_x-low_x)))
print("  offset: %f" % ((high_x+low_x)/2))
print(y_label)
print("  scale: %f" % (1.0 / (high_y-low_y)))
print("  offset: %f" % ((high_y+low_y)/2))
print(z_label)
print("  scale: %f" % (1.0 / (high_z-low_z)))
print("  offset: %f" % ((high_z+low_z)/2))
plt.show()

plt.plot(acc_z, acc_x)
plt.ylabel(x_label)
plt.xlabel(z_label)
plt.show()

plt.plot(acc_z, acc_y)
plt.ylabel(y_label)
plt.xlabel(z_label)
plt.show()



