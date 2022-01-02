#!/usr/bin/env python3
import sys

def usage():
   print("Convert output of pyIGRF into even simpler format")
   print("")
   print("Usage: {} <igrf-generated-file> <output.txt>")
   print("")
   sys.exit(1)


if len(sys.argv) != 3:
   usage()

infile = sys.argv[1]
outfile = sys.argv[2]

with open(infile, 'r') as fin:
   with open(outfile, 'w') as fout:
      lines = fin.readlines()
      header = None
      for line in lines:
         # first line is header -- strip it
         if header is None:
            header = line
            continue
         # we're interested in the following columns
         #     0  lat
         #     1  lon
         #     2  declination
         #     3  inclination
         #     5  x
         #     6  y
         #     7  z
         toks = line.split()
         if len(toks) < 8:
            # we must be done -- exit
            break
         lat = toks[0]
         lon = toks[1]
         dec = toks[2]
         inc = toks[3]
         x = toks[5]
         y = toks[6]
         z = toks[7]
         fout.write('{} {} {} {} {} {} {}\n'.format(lon, lat, dec, inc, x, y, z))
   


