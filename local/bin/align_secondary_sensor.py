#!/usr/bin/env python3
HELP_STR = """
Script to take generate dev2ship matrices for a secondary sensors.
See calibration.md

NOTE: this script is based on the assumption that ACC and MAG have 
identical axes (e.g., as if from th same sensor)
Alignment can be achieved through 'axis_alignment' under dev/

NOTE2: the alignment of a device is based directly on its ACC/MAG
sensors. If a device has component elements, those must be aligned
relative to the device's ACC/MAG.

To calibrate a secondary sensor, create a file in
   <env>/<device>/sensors/secondary_alignment.txt

In that file should be 2 lines (blank lines and comment lines starting with
'#' are ignored)

   ship <acc-x> <acc-y> <acc-z> <heading>
   dev <acc-x> <acc-y> <acc-z> <mag-x> <mag-y> <mag-z>

where the values <acc-[xyz]> and <mag-[xyz]> should be the average sensor
values from ACC and MAG sensors when the ship is level and at rest.
The value <heading> is the compass heading (magnetic) of the ship's bow.
Both ship and device values must be provided. These should be measured
at the same time, to account for possible drift or movement.
For 'ship' acc values, this is the measured values of the primary sensor
corrected by that sensor's dev2ship matrix.
The ship's heading is internally converted to an approximate sensor 
reading, running the generated ship-north vector backwards through the 
primary sensor's dev2ship matrix.

It's important that all sensors must share the same axes. If sensors aren't
aligned physically, this can be done using 'axis_alignment' files under
dev/. Ideally this should be done before data acquisition, so all
stored data is correctly aligned. If not, a manual fix is required.

For example: 

   ship 0.05204437  -0.032328   0.998441 133
   dev 0.00415534   1.008014  0.01134745 0.27413 -0.65714   -0.1260482


Run the script 'align_secondary_sensor.py <env> <device> <primary device>'
This creates the following files <env>/<device>/sensors/:

   acc_dev2ship
   mag_dev2ship
   gyr_dev2ship

TODO these dev2ship files are identical. consolidate into single file

"""
import sys
import numpy as np
import os
import math
sys.path.insert(0, "/pinet/local/bin/")
from align_helper import (
      print_mat,
      print_vec,
      write_mat,
      read_mat,
      #build_gyr_matrix,
      build_rot_mat_from_yz,
      normalize_north,
      D2R, R2D
      )

def usage():
   print("Creates dev2ship matrices for secondary sensor")
   print("")
   print("Usage: {} <env> <device name> <primary>".format(sys.argv[0]))
   print("")
   sys.exit(1)

if len(sys.argv) == 1:
   print(HELP_STR)
   print("----------")
   usage()
elif len(sys.argv) != 4:
   usage()
ENV = sys.argv[1]
DEVICE = sys.argv[2]
PRIMARY = sys.argv[3]

# constants

ENV_ROOT_DIR = "/pinet/" + ENV + "/" + DEVICE + "/sensors/"
# make sure root directories exist
if not os.path.isdir(ENV_ROOT_DIR):
   print("Environment ({}) or device ({}) does not exist".format(ENV, DEVICE))
   print("Looking for {}".format(ENV_ROOT_DIR))
   usage()

# make sure input file exists
SENSOR_FILE = ENV_ROOT_DIR + "secondary_alignment.txt"
if not os.path.exists(SENSOR_FILE):
   print("Alignment input file {} does not exist".format(SENSOR_FILE))
   usage()

DEV2SHIP = ENV_ROOT_DIR + "{}_dev2ship"

########################################################################
########################################################################


def generate_primary_north(secondary_north, heading, dev2ship):
   """
   Takes ship heading and generates approximate magnetic vector
   for primary sensor from that. Primary sensor's magnetic readings
   are not reliable, esp. if sensor is near CG (CB) of ship.
   Secondary north is used to keep approximate primary mag vector
   with the same structure (eg, similar simulated position on earth)
   and the correct polarity (some mag sensors flip the field)

   secondary north is north (mag) vector from secondary sensor
   heading in degrees
   dev2ship is primary sensor alignment matrix
   """
   # make alias and simulated north
   sn = secondary_north
   sim_north = np.zeros(3)
   # get magnitude of horizontal and vertical components of field, so
   #  generated vector has similar structure
   horiz = math.sqrt(sn[0]*sn[0] + sn[2]*sn[2])
   # assume ship is level. match vertical components
   sim_north[1] = sn[1]
   rad = D2R * heading
   # equations are set for taking heading as input, not north
   sim_north[2] = horiz * math.cos(rad)
   sim_north[0] = horiz * math.sin(rad)
   print("sim_north: ", sim_north)
   # sim_north represents ship's alignment relative to world. apply reversed
   #  dev2ship matrix to convert this to sensor's alignment to world
   prim_north = np.matmul(np.transpose(dev2ship), sim_north)
   return prim_north


def read_input_file():
   """ 
   Read primary alignment file and returns normalized up vector and
   ship's heading for primary sensor, and normalized up and north vectors
   for secondary device
   description
   """
   prim_up_vec = None
   prim_north_vec = None
   dev_up_vec = None
   dev_north_vec = None
   gyr_vec = None
   err = 0
   with open(SENSOR_FILE, 'r') as f:
      lines = f.readlines()
      for line in lines:
         line = line.strip()
         # let empty or comment lines pass
         if len(line) < 1 or line.startswith('#'):
            continue
         toks = line.split()
         # load 3 types
         if toks[0] == 'ship':
            if len(toks) != 5:
               print("Parse error in {}: {} not recognized"
                     .format(SENSOR_FILE, line))
               err += 1
            else:
               prim_up_vec = np.zeros(3)
               prim_up_vec[0] = float(toks[1])
               prim_up_vec[1] = float(toks[2])
               prim_up_vec[2] = float(toks[3])
               heading = float(toks[4])
               print("ship")
               print("  up: {}".format(prim_up_vec))
               print("  heading: {}".format(heading))
         elif toks[0].startswith('dev'):
            if len(toks) != 7:
               print("Parse error in {}: {} not recognized"
                     .format(SENSOR_FILE, line))
               err += 1
            else:
               dev_up_vec = np.zeros(3)
               dev_up_vec[0] = float(toks[1])
               dev_up_vec[1] = float(toks[2])
               dev_up_vec[2] = float(toks[3])
               dev_north_vec = np.zeros(3)
               dev_north_vec[0] = float(toks[4])
               dev_north_vec[1] = float(toks[5])
               dev_north_vec[2] = float(toks[6])
               print("dev")
               print("  up: {}".format(dev_up_vec))
               print("  north: {}".format(dev_north_vec))
               dev_north_vec = normalize_north(dev_up_vec, dev_north_vec)
         elif toks[0] == 'gyr':
            # DEPRECATED
            if len(toks) != 4:
               print("Parse error in {}: {} not recognized"
                     .format(SENSOR_FILE, line))
               err += 1
            else:
               gyr_vec = [ '', '', '' ]
               gyr_vec[0] = toks[1]
               gyr_vec[1] = toks[2]
               gyr_vec[2] = toks[3]
         else:
            print("Parse error in {}: {} not a recognized field"
                  .format(SENSOR_FILE, toks[0]))
            err += 1
   if prim_up_vec is None:
      print("Input file {} missing ship description".format(SENSOR_FILE))
      err += 1
   if dev_up_vec is None:
      print("Input file {} missing device description".format(SENSOR_FILE))
      err += 1
   if err > 0:
      sys.exit(1)
   #
   prim_up_vec = prim_up_vec / np.linalg.norm(prim_up_vec)
   dev_up_vec = dev_up_vec / np.linalg.norm(dev_up_vec)
   dev_north_vec = dev_north_vec / np.linalg.norm(dev_north_vec)
   return prim_up_vec, heading, dev_up_vec, dev_north_vec


# read primary's dev2ship alignment
PRIMARY_ROOT_DIR = "/pinet/" + ENV + "/" + PRIMARY + "/sensors/"
# acc and mag share the same axes so this is primary's dev2ship
prim_dev2ship = read_mat(PRIMARY_ROOT_DIR + "acc_dev2ship")
#print_mat("prim_dev2ship", prim_dev2ship)

# get input data
prim_up, heading, dev_up, dev_north = read_input_file()
# generate approximate north vector from ship's heading
prim_north = generate_primary_north(dev_north, heading, prim_dev2ship)
# primary north must be orthogonal to primary up
prim_north = normalize_north(prim_up, prim_north)
print_vec("primary north", prim_north)

prim = build_rot_mat_from_yz(prim_up, prim_north)
#print_vec("pu", prim_up)
#print_vec("pn", prim_north)
#print_mat("prim", prim)

sec = build_rot_mat_from_yz(dev_up, dev_north)
#print_vec("su", dev_up)
#print_vec("sn", dev_north)
#print_mat("sec", sec)

# matrix to convert secondary coordinates to primary coord space
# matrix confirmed to limits of roundoff error, running secondary
#  up/north through matrix gives primary up/north
dev2prim = np.matmul(np.transpose(prim), sec)
#print_mat("d2p", dev2prim)

sec_dev2ship = np.matmul(prim_dev2ship, dev2prim)
#print_mat("sec_dev2ship", sec_dev2ship)

#############################################
# write output
# TODO consolidate these into a single file
write_mat(DEV2SHIP.format("acc"), sec_dev2ship)
write_mat(DEV2SHIP.format("mag"), sec_dev2ship)
write_mat(DEV2SHIP.format("gyr"), sec_dev2ship)

# build rotation matrix to convert acc/mag matrix to gyr
#gyr_mat = build_gyr_matrix(sec_dev2ship, gyr_vec)
#write_mat(DEV2SHIP.format("gyr"), gyr_mat)


""" algorithm test
uncomment following section. both ship2world matrices should
be the same and they should represent the heading of the ship,
in world space
"""
## correct dev2world vectors w/ dev2ship alignment matrix to get
##  ship2world
#up = prim_dev2ship.dot(prim_up)
#north = prim_dev2ship.dot(prim_north)
#prim_ship2world = build_rot_mat_from_yz(up, north)
#print_mat("prim_ship2world", prim_ship2world)
#
# use secondary device's up/north to generate ship2world
#sec2world = np.matmul(prim_dev2ship, dev2prim)
#sup = sec2world.dot(dev_up)
#snorth = sec2world.dot(dev_north)
#sec_ship2world = build_rot_mat_from_yz(sup, snorth)
#print_mat("sec_ship2world", sec_ship2world)

