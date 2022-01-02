#!/usr/bin/env python2
import sys
import struct

"""
Reads in beacon text file and converts contents to indexed binary file.
Space for storing beacon neighbors is provided but is initialized to zero.

Format for each beacon record is:
   float32     longitude
   float32     latitude
   int32       num neighbors
   int32       <unused padding>
   repeated 8 times:
      int32_t  beacon index
      int32_t  beacon distance in meters
total length is 80 bytes (4+4 + 4+4 + 8*8)

num_neighbors values:
   -1 means the beacon hasn't been evaluated
   0 means the beacon has no neighbors or is otherwise invalid
"""

if len(sys.argv) != 3:
   print("Usage: %s <in txt file> <out file root>" % sys.argv[0])
   sys.exit(1)
infile = sys.argv[1]
outfile_root = sys.argv[2]

#infile = "../master/beacons.txt"
#outfile_root = "../master/beacons"

outfile_idx = outfile_root + ".idx"
outfile_bin = outfile_root + ".bin"


record = []
record.append(0.0)   # longitude
record.append(0.0)   # latitude
record.append(-1)    # num neighbors
record.append(0)     # index
for i in range(8):   
   record.append(0)    # nbr index
   record.append(0)    # nbr distance (m)

# array with number of beacons in the band, and the starting
#     offset of the first beacon in the band
# number of beacons stored first, and a later pass will calculate
#     starting offsets
idx = []
for i in range(0, 180):
   entry = {}
   entry["start"] = 0
   entry["count"] = 0
   idx.append(entry)


row = 0
row_len = 0
with open(outfile_bin, 'w') as fout:
   print("Writing to %s" % outfile_bin)
   with open(infile, 'r') as fin:
      print("Reading from %s" % infile)
      content = fin.readlines()
      print("Content len %d" % len(content))
      index = 0
      for line in content:
         if line.startswith('#'):
            continue
         toks = line.strip().split()
         lon = float(toks[0])
         lat = float(toks[1])
         lat_row = int(lat)
         if lat_row != row:
            print("row %d, len %d" % (lat_row, row_len))
            idx[row]["count"] = row_len
            row_len = 0
            if lat_row < row and row != 0:
               # beacons must be in order, at least in regards to what row
               #  they reside in (within a row order doesn't matter)
               # i.e., no beacons in row 40 then 41 then 40
               print("Processing row %d but encounted beacon from row %d"
                     % (row, lat_row))
               print("Most recent point %.4f,%.4f" % (lon, lat))
               print("Investigate '%s'" % infile)
               sys.exit(1)
            row = lat_row
         record[0] = lon
         record[1] = lat
         record[3] = index
         s = struct.pack('ffiiiiiiiiiiiiiiiiii', *record)
         fout.write(s)
         row_len += 1
         index += 1

# store count of last row
idx[row]["count"] = row_len

# get band offsets
offset = 0
for entry in idx:
   entry["start"] = offset
   offset += entry["count"]

# write index file
with open(outfile_idx, 'w') as fout:
   print("Writing to %s" % outfile_idx)
   for entry in idx:
#      print("%d %d" % (entry["start"], entry["count"]))
      data = [entry["start"], entry["count"]]
      print(data);
      s = struct.pack('ii', *data)
      fout.write(s)


