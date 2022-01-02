#!/usr/bin/env python2
import matplotlib.pyplot as plt
import numpy as np
import sys

if len(sys.argv) != 2:
   print("Usage: %s <imu log file>" % sys.argv[0])
   sys.exit(1)

mag_x = []
mag_y = []
mag_z = []
infile = sys.argv[1]
with open(infile, 'r') as f:
   contents = f.readlines()
   for line in contents:
      if len(line) < 5:
         continue
      toks = line.split()
      mag_x.append(float(toks[7]))
      mag_y.append(float(toks[8]))
      mag_z.append(float(toks[9]))

low_x = min(mag_x)
high_x = max(mag_x)
low_y = min(mag_y)
high_y = max(mag_y)
low_z = min(mag_z)
high_z = max(mag_z)

x_label = "X min=%f max=%f range=%f" % (low_x, high_x, high_x-low_x)
y_label = "Y min=%f max=%f range=%f" % (low_y, high_y, high_y-low_y)
z_label = "Z min=%f max=%f range=%f" % (low_z, high_z, high_z-low_z)
plt.plot(mag_x, mag_y)
plt.xlabel(x_label)
plt.ylabel(y_label)
print(x_label)
print("  avg mag: %f" % (0.5 * (high_x-low_x)))
print("  offset: %f" % ((high_x+low_x)/2))
print(y_label)
print("  avg mag: %f" % (0.5 * (high_y-low_y)))
print("  offset: %f" % ((high_y+low_y)/2))
print(z_label)
print("  avg mag: %f" % (0.5 * (high_z-low_z)))
print("  offset: %f" % ((high_z+low_z)/2))
plt.show()

plt.plot(mag_z, mag_x)
plt.xlabel(z_label)
plt.ylabel(x_label)
plt.show()

plt.plot(mag_y, mag_z)
plt.xlabel(y_label)
plt.ylabel(z_label)
plt.show()



