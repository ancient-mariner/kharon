#!/usr/bin/env python3
""" averages values in IMU output log file. file assumed
to contain white-space columns of numbers representing time,
gyr (3 floats), acc (3 floats) and mag (3 floats). an optional 11th 
column (eg, temperature) is accepted 

Outputs gyr, acc and mag vectors
"""
import sys
import numpy as np

def usage():
   print("Usage: {} <imu log file to analyze>")
   print("")
   print("where file contains 10 or 11 white-space separated columns of values")
   sys.exit(1)

if len(sys.argv) != 2:
   usage()
infile = sys.argv[1]

f = open(infile, 'r')
if f is None:
   print("Unable to open '{}'".format(infile))
   sys.exit(1)

vals = None
val_count = 0
linenum = 0

for line in f.readlines():
   linenum += 1
   if line.startswith('#'):
      continue
   toks = line.strip().split()
   if len(toks) == 0:
      continue
   if vals is None:
      vals = np.zeros(len(toks))
   elif len(toks) != len(vals):
      print("Error: line {} has an unexpected number of values".format(linenum))
      break
   for i in range(len(toks)):
      vals[i] += float(toks[i].rstrip(','))
   val_count += 1

f.close()

vals /= val_count

if vals is None:
   print("No content found to sum")
   sys.exit(1)

if len(vals) != 10 and len(vals) != 11:
   print("Average value of each column")
   for i in range(len(vals)):
      print("  {:2d} {:.3f}".format(i, vals[i]))
else:
   print("Average time: {:3f}".format(vals[0]))
   g = vals[1:4]
   a = vals[4:7]
   m = vals[7:10]
   print("Gyr: {}".format(g))
   print("Acc: {}".format(a))
   print("Mag: {}".format(m))
#   print("Gyro: {:4f} {:4f} {:4f}".format(vals[1], vals[2], vals[3]))
#   print("Acc:  {:4f} {:4f} {:4f}".format(vals[4], vals[5], vals[6]))
#   print("Mag:  {:4f} {:4f} {:4f}".format(vals[7], vals[8], vals[9]))
   if len(vals) == 11:
      print("Average temp: {:.2f}".format(vals[10]))
   print("  -- normalized --")
   norm_g = np.linalg.norm(g)
   norm_a = np.linalg.norm(a)
   norm_m = np.linalg.norm(m)
   if norm_g > 0.0:
      g = g / np.linalg.norm(g)
   if norm_a > 0.0:
      a = a / np.linalg.norm(a)
   if norm_m > 0.0:
      m = m / np.linalg.norm(m)
   print("Gyr: {}".format(g))
   print("Acc: {}".format(a))
   print("Mag: {}".format(m))

