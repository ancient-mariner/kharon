#!/usr/bin/env python3
HELP_STR = """
Script to take generate dev2ship matrices for a device's components.
For example, a tower can have 4 component cameras. Each component
is itself a device but will be called a component here for clarity.

Input to the script is the dev2ship matrix for the device and the
dev2parent rotations necessary to align each component to the device's space.

At present, only cam components are aligned. 
All acc/mag/gyr axis alignment is done separately, using xxx_alignment files.

Component names have the following format: <type>_<component name>
For example, cam_tower3


The components for a device are listed in:

   dev/<device>/components/

There's one file for each component. The contents of the file are 
not relevant here.
   
Each component requires a description of rotations necessary to rotate
the component to the parent device's space (that space defined by
the parent's ACC/MAG sensor). These files have names *_dev2parent and
are located in

   dev/<component>/sensors/

dev2parent files must contain a vector describing the necessary 
rotation to align the component with the device, represented as 
necessary rotations along X, Y and Z axes. 
rotations are performed first along Y, then X, then Z


Output rotation matrix is stored in the component's device folder:

   <env>/<component>/sensors/cam_dev2ship


Example usage: 'align_tertiary_sensors.py <env> tower'
(ie, run on full device, not device components)

"""
import sys
import numpy as np
import glob
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


DEV2SHIP = "dev2ship"
DEV2PARENT = "dev2parent"

DEV_SENSORS = "/pinet/dev/{}/sensors/"

def usage():
   print("Sets dev2ship for device components")
   print("Note that this is run on the parent device. The script finds")
   print("the device components to align")
   print("")
   print("Usage: %s <env> <parent device>".format(sys.argv[0]))
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

DEVICE_SENSOR_DIR = "/pinet/{}/{}/sensors/".format(ENV, DEVICE)
DEVICE_COMPONENT_DIR = "/pinet/dev/{}/components/".format(DEVICE)

def create_component_matrix(dev, name):
   """ 
   Takes name of component, loads dev2parent, converts that to matrix 
   and returns the matrix.
   Throws exception if file doesn't exist, or on parse error
   """
   # load vector
   fname = DEV_SENSORS.format(dev) + name
#   fname = "/pinet/dev/%s/sensors/%s" % (dev, name)
   print("Opening ", fname)
   with open(fname, 'r') as f:
      lines = f.readlines()
      for i in range(len(lines)):
         line = lines[i].strip()
         if line.startswith('#'):
            continue
         # split string on tabs or space, in case those are used
         toks = line.replace('\t', ' ').split(' ')
         arr = []
         for tok in toks:
            if len(tok) > 0:
               arr.append(float(tok))
         if len(arr) != 3:
            raise ValueError("Error parsing '%s'" % line)
   print(" -->> ", arr)
   # convert to matrix
   # rotate by Y then X then Z
   cs = math.cos(D2R * arr[0])
   sn = math.sin(D2R * arr[0])
   x = np.asarray((1., 0., 0., 0., cs, -sn, 0., sn, cs)).reshape(3,3)
#   print_mat("x:", x);
   #
   cs = math.cos(D2R * arr[1])
   sn = math.sin(D2R * arr[1])
   y = np.asarray((cs, 0., sn, 0., 1., 0., -sn, 0., cs)).reshape(3,3)
#   print_mat("y:", y);
   #
   cs = math.cos(D2R * arr[2])
   sn = math.sin(D2R * arr[2])
   z = np.asarray((cs, -sn, 0., sn, cs, 0., 0., 0., 1.)).reshape(3,3)
#   print_mat("z:", z);
   #
   rot = np.dot(np.dot(y, x), z)
   print_mat("rot", rot)
   return rot

########################################################################

# get device's dev2ship matrix. Use acc_dev2ship as the device's 
#  coordinate space is defined by the axes of its ACC/MAG sensors
dev2ship = read_mat(DEVICE_SENSOR_DIR + "acc_dev2ship")

# get components
# for each component, read dev2parent matrix and write it's dev2ship
comp_dir = "/pinet/dev/%s/components/*" % DEVICE
components = glob.glob(comp_dir)
for name in components:
    # get name
    if name.startswith('_'):
        continue
    if name.startswith('.'):
        continue
    fullname = os.path.basename(name)
    sensor_type = fullname[:4]
    component = fullname[4:]
    # read alignment vector
    try:
        child2dev = create_component_matrix(component, sensor_type + DEV2PARENT)
    except Exception:
        print("")
        print("")
        print("************ Error ************")
        print("Failed to load %s for %s" % (DEV2PARENT, fullname))
        print("")
        print("Please fix and rerun script")
        print("")
        sys.exit(1)
    child2ship = np.dot(dev2ship, child2dev)
    # write child2ship
    fname = "/pinet/{}/{}/sensors/{}dev2ship".format(ENV, component, sensor_type)
    write_mat(fname, child2ship)
    print("")
#    print("")
#    print_mat("child2dev", child2dev)
#    print_mat("dev2ship", dev2ship)
#    print_mat("child2ship", child2ship)
#    x = [1, 0, 0]
#    y = [0, 1, 0]
#    z = [0, 0, 1]
#    print("")
#    print_vec("child2dev x", np.dot(child2dev, x))
#    print_vec("child2dev y", np.dot(child2dev, y))
#    print_vec("child2dev z", np.dot(child2dev, z))
#    print("")
#    print_vec("dev2ship x", np.dot(dev2ship, x))
#    print_vec("dev2ship y", np.dot(dev2ship, y))
#    print_vec("dev2ship z", np.dot(dev2ship, z))
#    print("")
#    print_vec("child2ship x", np.dot(child2ship, x))
#    print_vec("child2ship y", np.dot(child2ship, y))
#    print_vec("child2ship z", np.dot(child2ship, z))
#    print("")



