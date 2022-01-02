#!/usr/bin/env python3
help_str = '''
takes output from default beacons (ie, a text file) plus a diff file (also,
text) and generates a new beacon file that merges in the differences.
convert_to_bin can be run on this new file.

dif format
<number> ignored
A lon lat         add beacon at lon,lat
D lon lat         delete beacon at lon,lat
M lon lat x y     move beacon at lon,lat by x,y pixels. this is equiv to A D

Note: a new beacon can only be added relative to an existing beacon if
the base beacon exists. this is to help prevent duplicat beacons from
being created, as there's presently no check against that. the output of 
this script can be used as its input, allowing a sequence of dif files to
be applied to the same beacon data.

dif file should be ordered similarly to beacon list, simplifying search
logic and greatly increasing speed (read: 'should' means 'must)
'''
import sys
import math

def usage():
   print("usage: %s <input beacon list> <dif file> <output file>" % sys.argv[0])
   print(help_str)
   sys.exit(1)

#in_beacon = "working.txt"
#in_diff = "vancouver.txt"
#outfile = "beacons.changed.txt"

D2R = math.pi / 180.0

if len(sys.argv) != 4:
   usage()
in_beacon = sys.argv[1]
in_diff = sys.argv[2]
outfile = sys.argv[3]

# list of beacons from input file
# format of each entry is [lon, lat]
beacons = []
# list of beacons that have had changes applied. this goes to output file
add_beacons = []
delete_beacons = []
# modification list
# format is an array containing: lon, lat, cmd [, dx, dy]
# where cmd is ADD or DELETE
ADD = 0
DELETE = 1
MOVED = 2

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


################################################
# read modifications
with open(in_diff, 'r') as f:
   print("Reading change list from '%s'" % in_diff)
   print("-------------------------------")
   content = f.readlines()
   for line in content:
      # if leading char is digit or '#' then continue
      if len(line.strip()) == 0:
         continue
      if line[0] in '0123456789#':
         continue
      toks = line.split()
      cmd = toks[0]
      lon = float(toks[1])
      lat = float(toks[2])
      # parse commands. convert 'M' to 'A' + 'D'
      if cmd == 'A':
         if len(toks) != 3:
            print("ADD command is unexpected format", toks)
            sys.exit(1)
         add_beacons.append([lon, lat, ADD])
      elif cmd == 'D':
         if len(toks) != 3:
            print("DEL command is unexpected format", toks)
            sys.exit(1)
         delete_beacons.append([lon, lat, DELETE])
      elif cmd == 'M':
         if len(toks) != 5:
            print("MOD command is unexpected format", toks)
            sys.exit(1)
         delete_beacons.append([lon, lat, DELETE])
         # adjust to new lat/lon
         new_lat = lat + float(toks[4]) / 720.0
         new_lon = lon + float(toks[3]) / 720.0 / math.sin(D2R * lat)
         add_beacons.append([new_lon, new_lat, MOVED])
      else:
         # uh oh
         print("Error parsing change command '%s'" % line)
         sys.exit(1)

if len(add_beacons) == 0 and len(delete_beacons) == 0:
   print("No changes to apply")
   sys.exit(0)


################################################
# apply modifications

add_idx = 0
delete_idx = 0
beacon_idx = 0
output_list = []
while beacon_idx < len(beacons):
   beacon = beacons[beacon_idx]
   if add_idx < len(add_beacons):
      create = add_beacons[add_idx]
   else:
      create = None
   if delete_idx < len(delete_beacons):
      remove = delete_beacons[delete_idx]
   else:
      remove = None
   # see if this beacon should be eliminated
   # if change is a delete then see if it matches present beacon
   if remove is not None:
      dx = beacon[0] - float(remove[0])
      dy = beacon[1] - float(remove[1])
      if abs(dx) < 0.001 and abs(dy) < 0.001:
         # call it a match. don't add this beacon to output list
         print("Deleting beacon at %f,%f" % (beacon[0], beacon[1]))
         delete_idx += 1
         beacon_idx += 1
         continue
   # if next new beacon is north of next existing beacon, add it
   if create is not None and create[1] <= beacon[1]:
      newby = []
      newby.append(float(create[0]))
      newby.append(float(create[1]))
      output_list.append(newby)
      add_idx += 1
      if create[2] == ADD:
         print("Creating beacon at %f,%f" % (newby[0], newby[1]))
      else:
         print("Moved beacon to %f,%f" % (newby[0], newby[1]))
      continue
   # if no change executed, put next beacon on output list
   print("Keeping beacon at %f,%f" % (beacon[0], beacon[1]))
   output_list.append(beacon)
   beacon_idx += 1


if delete_idx < len(delete_beacons):
   change = delete_beacons[delete_idx]
   print("#################################")
   print("Failed to delete %.4f,%.4f" % (change[0], change[1]))
   sys.exit(1)

# apply remaining changes
while add_idx < len(add_beacons):
   # make sure they're ADDs -- if we missed a delete that's bad
   change = add_beacons[add_idx]
   add_idx += 1
   if change[2] == DELETE:
      print("Failed to delete %.4f,%.4f" % (change[0], change[1]))
      sys.exit(1)
   newby = []
   newby.append(float(change[0]))
   newby.append(float(change[1]))
   output_list.append(newby)


with open(outfile, 'w') as f:
   print("-------------------------------")
   print("Writing new beacon file to '%s'" % outfile)
   for beacon in output_list:
      f.write("%f %f\n" % (beacon[0], beacon[1]))

