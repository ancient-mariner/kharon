#!/usr/bin/env python3
help_str = '''
Converts pixel offsets relative to beacon into lat,lon.

Input file should have one 'dx dy' offset per line. For each line in 
input file the lat,lon of that pixel will be printed to stdout. the output
should be compatible to copy-paste into a file that's fed into 
adjust_beacons.py

dif file should be ordered similarly to beacon list, simplifying search
logic and greatly increasing speed (read: 'should' means 'must)
'''
import sys
import math

def usage():
   print("usage: %s <beacon file> <offset list file> <beacon num>" % sys.argv[0])
   print(help_str)
   sys.exit(1)

if len(sys.argv) != 4:
   usage()
in_beacon = sys.argv[1]
delta_file = sys.argv[2]
beacon_num = int(sys.argv[3])

D2R = math.pi / 180.0


# list of beacons from input file
# format of each entry is [lon, lat]
beacons = []

################################################
# read beacons
with open(in_beacon, 'r') as f:
   print("Reading beacons in from '%s'" % in_beacon)
   content = f.readlines()
   for line in content:
      if line.startswith('#'):
         continue
      toks = line.split()
      beacons.append([float(toks[0]), float(toks[1])])

beacon = beacons[beacon_num]
print("Delta positions based on beacon %d at %f,%f" % (beacon_num,
                                                       beacon[0],
                                                       beacon[1]))

with open(delta_file, 'r') as f:
   print("Reading delta file '%s'" % delta_file)
   content = f.readlines()
   for line in content:
      if len(line.strip()) == 0:
         continue;
      if line.startswith('#'):
         continue
      toks = line.split()
      # each Y pixel is 1/720th of a degree
      lat = beacon[1] + (float(toks[1]) - 360.0) / 720.0
      lon = beacon[0] + (float(toks[0]) - 360.0) / 720.0 / math.sin(D2R * float(beacon[1]))
      #print("dx %s from %f = %f" % (toks[0], beacon[0], lon))
      #print("dy %s from %f = %f" % (toks[1], beacon[1], lat))
      print("A %f %f" % (lon, lat))


