#!/usr/bin/env python3
HELP_STR = """
Script to take generate dev2ship matrices for a primary attitude sensor.

With dev2ship, acc/mag/gyr readings can be converted from sensor space
to ship space, indicating virtual acc/mag/gyr readings of ship.

NOTE: this script is based on the assumption that ACC and MAG have 
identical axes (e.g., as if from th same sensor), and that GYR axes
are parallel to ACC/MAG axes, but perhaps reordered (e.g., X is Z, etc)

Data provided to script through input file. File contains the following:

   - accelerometer readings on each reported axis (vector)

   - alignment vector for line parallel to the longitudinal axis
   of the ship, in the direction of the bow

   - gyro axis alignment to axes of ACC/MAG (acc and mag are assumed
   to have the same axes)


The input file is:

   <env>/<device>/sensors/primary_alignment.txt

Format of the file is (blank lines and comment lines starting with '#' are
OK):

   up <acc-x> <acc-y> <acc-z>
   north <north-x> <north-y> <north-z>
   gyr <acc-axis-x> <acc-axis-y> <acc-axis-z>

where:
   <acc-[xyz]> should be the average sensor values from ACC when the ship
   is level and at rest

   <north-[xyz]> are manually entered values that approximate the direction
   of the bow on a line parallel to the longitudinal axis of the ship.
   The values must be described in the coordinate space of the sensor.
   For example, if the Y axis of the sensor is 10 degrees left of the
   bow, the X axis vertical, and the Z axis 100 degrees left of the bow,
   then X=0 (there is no X component to the longitidinal axis), with
   Y=cos(10), Z=cos(100) (i.e., a strong positive Y and a weak negative Z)

   <acc-axis-[xyz]> is the name ("x", "y" or "z") of the axis of the
   accelerometer that corresponds to the X Y and Z axis of the gyro.
   For example, "gyr Y Z X" means that the gyro's X axis aligns with
   the accelerometer Y axis, gyro's Y aligns with acc's Z, and Z with X.
   It is assumed (required) that the gyro, accelerometer and magnetometer
   are all aligned along their axes, even if the axes are different.
   NOTE
   THIS IS OBSOLETE: use 'axis_alignment' (under dev/) for each sensor 
   to align all sensor axes on acquisition. 

example:
   up 4.85536655e-04  9.97348674e-01 -7.27694146e-02
   north 0 1 0
   gyr x y z
   #gyr -x z y

Ship space has the bow as Z, up as Y and port side as X.

The following files are generated:
   
   acc_dev2ship
   mag_dev2ship
   gyr_dev2ship

in the folder <env>/<device>/sensors/

"""
import sys
import numpy as np
import os
import math
from align_helper import (
      print_mat,
      print_vec,
      write_mat,
      read_mat,
      build_gyr_matrix,
      build_rot_mat_from_yz,
      normalize_north,
      D2R, R2D
      )

def usage():
   print("Creates dev2ship matrices for IMU elements gyr, mag, acc")
   print("")
   print("Usage: {} <env> <device name>".format(sys.argv[0]))
   print("")
   sys.exit(1)

if len(sys.argv) == 1:
   print(HELP_STR)
   print("----------")
   usage()
elif len(sys.argv) != 3:
   usage()
ENV = sys.argv[1]
DEVICE = sys.argv[2]

# constants

ENV_ROOT_DIR = "/pinet/" + ENV + "/" + DEVICE + "/sensors/"
# make sure root directories exist
if not os.path.isdir(ENV_ROOT_DIR):
   print("Environment ({}) or device ({}) does not exist".format(ENV, DEVICE))
   print("Looking for {}".format(ENV_ROOT_DIR))
   usage()

# make sure input file exists
SENSOR_FILE = ENV_ROOT_DIR + "primary_alignment.txt"
if not os.path.exists(SENSOR_FILE):
   print("Alignment input file {} does not exist".format(SENSOR_FILE))
   usage()

DEV2SHIP = ENV_ROOT_DIR + "{}_dev2ship"

########################################################################
########################################################################


def read_input_file():
   """ Read primary alignment file and return triad of vectors: up, north, gyr
   """
   up_vec = None
   north_vec = None
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
         if len(toks) != 4:
            print("Parse error in {}: {} not recognized"
                  .format(SENSOR_FILE, line))
            err += 1
         # load 3 types
         if toks[0] == 'up':
            up_vec = np.zeros(3)
            up_vec[0] = float(toks[1])
            up_vec[1] = float(toks[2])
            up_vec[2] = float(toks[3])
         elif toks[0] == 'north':
            north_vec = np.zeros(3)
            north_vec[0] = float(toks[1])
            north_vec[1] = float(toks[2])
            north_vec[2] = float(toks[3])
         elif toks[0] == 'gyr':
            gyr_vec = [ '', '', '' ]
            gyr_vec[0] = toks[1]
            gyr_vec[1] = toks[2]
            gyr_vec[2] = toks[3]
         else:
            print("Parse error in {}: {} not a recognized field"
                  .format(SENSOR_FILE, toks[0]))
            err += 1
   if up_vec is None:
      print("Input file {} missing up description".format(SENSOR_FILE))
      err += 1
   if north_vec is None:
      print("Input file {} missing north description".format(SENSOR_FILE))
      err += 1
   if gyr_vec is None:
      print("Input file {} missing gyr description".format(SENSOR_FILE))
      err += 1
   if err > 0:
      sys.exit(1)
   #
   return up_vec, north_vec, gyr_vec


# get input data
up_vec, north_vec, gyr_vec = read_input_file()
# normalize up vector
up_vec = up_vec / np.linalg.norm(up_vec)
# get normalized orthogonal north vector
north_vec = normalize_north(up_vec, north_vec)
print("Up: {}".format(up_vec))
print("North: {}".format(north_vec))
print("Gyr: {}".format(gyr_vec))

# create rotation matrix for ACC and MAG
up_north_mat = build_rot_mat_from_yz(up_vec, north_vec)
write_mat(DEV2SHIP.format("acc"), up_north_mat)
write_mat(DEV2SHIP.format("mag"), up_north_mat)

# build rotation matrix to convert acc/mag matrix to gyr
gyr_mat = build_gyr_matrix(up_north_mat, gyr_vec)
write_mat(DEV2SHIP.format("gyr"), gyr_mat)


