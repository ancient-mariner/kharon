"""
Shared functions for align_* scripts
"""
import sys
import numpy as np
import os
import math

D2R = math.pi / 180.0
R2D = 180.0 / math.pi

########################################################################
########################################################################
# output functions

def print_mat(label, v):
   print(label)
   print("  [[%8.4f, %8.4f, %8.4f];" % (v[0][0], v[0][1], v[0][2]))
   print("   [%8.4f, %8.4f, %8.4f];" % (v[1][0], v[1][1], v[1][2]))
   print("   [%8.4f, %8.4f, %8.4f]]" % (v[2][0], v[2][1], v[2][2]))


def print_vec(label, v):
   print(label)
   print("  [%8.4f, %8.4f, %8.4f] " % (v[0], v[1], v[2]))


def write_mat(fname, mat):
   print("Writing %s" % fname)
   with open(fname, 'w') as f:
      f.write("%.4f %.4f %.4f\n" % (mat[0][0], mat[0][1], mat[0][2]))
      f.write("%.4f %.4f %.4f\n" % (mat[1][0], mat[1][1], mat[1][2]))
      f.write("%.4f %.4f %.4f\n" % (mat[2][0], mat[2][1], mat[2][2]))
      v = mat
      f.write("# [[%8.4f, %8.4f, %8.4f];" % (v[0][0], v[0][1], v[0][2]))
      f.write("  [%8.4f, %8.4f, %8.4f];" % (v[1][0], v[1][1], v[1][2]))
      f.write("  [%8.4f, %8.4f, %8.4f]]\n" % (v[2][0], v[2][1], v[2][2]))
#   print_mat(fname, mat)

def read_mat(fname):
   print("Reading %s" % fname)
   vec = []
   with open(fname, 'r') as f:
      content = f.readlines()
      for line in content:
         if len(line.strip()) <= 1:
            continue
         if line.strip().startswith('#'):
            continue
         row = np.zeros((3))
         toks = line.split()
         row[0] = float(toks[0])
         row[1] = float(toks[1])
         row[2] = float(toks[2])
         vec.append(row)
      if len(vec) != 3:
         print("Error parsing matrix. Contents:")
         print(content)
         sys.exit(1)
   mat = np.concatenate((vec[0], vec[1], vec[2])).reshape(3,3)
   return mat


########################################################################
########################################################################
# mathematical routines

def normalize_north(up, north):
   """ Takes up vector and approximate north vector and then projects
      north onto plane defined by up.
      Returns new north vector (unit vector)
   """
   up_vec = up / np.linalg.norm(up)
   north_vec = north / np.linalg.norm(north)
   # project north to plane defined by up
   north_orth = north_vec - np.dot(up_vec, north_vec) * up_vec
   # up and north must not be not parallel -- calibration at or
   #  near the poles will be inaccurate. 
   mag = np.linalg.norm(north)
   if mag < 0.25:
      print("WARNING: up and north vectors are too nearly parallel")
      print_vec("up", up_vec)
      print_vec("north", north_vec)
      # let it pass
   #
   north_orth = north_orth / np.linalg.norm(north_orth)
   return north_orth


def build_rot_mat_from_yz(y, z):
   y = y / np.linalg.norm(y)
   z = z / np.linalg.norm(y)
   x = np.cross(y, z)
#   print("Build rot mat from yz")
#   print_vec("x (%.3f)" % np.linalg.norm(x), x)
#   print_vec("y (%.3f)" % np.linalg.norm(y), y)
#   print_vec("z (%.3f)" % np.linalg.norm(z), z)
   mat = np.concatenate((x, y, z)).reshape(3,3)
   return mat
   

def convert_axis_to_vec(axis):
   """ 
   Returns unit vector along axis: "x", "y" or "z" with
   optional "+" or "-"
   """
   vec = np.zeros((3))
   if axis.endswith('x') or axis.endswith('X'):
      if axis.startswith('-'):
         vec[0] = -1
      else:
         vec[0] = 1
   elif axis.endswith('y') or axis.endswith('Y'):
      if axis.startswith('-'):
         vec[1] = -1
      else:
         vec[1] = 1
   elif axis.endswith('z') or axis.endswith('Z'):
      if axis.startswith('-'):
         vec[2] = -1
      else:
         vec[2] = 1
   else:
      print("Failed to recognize coordinate axis '{}'".format(axis))
      sys.exit(1)
   return vec


def build_gyr_matrix(up_north, gyr_vec):
   """
   Generate alignment matrix for gyro, given alignment matrix for acc/mag
   and alignment of gyro to acc/mag axes
   """
   mat = np.zeros((3,3))
   col0 = convert_axis_to_vec(gyr_vec[0])
   col1 = convert_axis_to_vec(gyr_vec[1])
   col2 = convert_axis_to_vec(gyr_vec[2])
   # build matrix. put columns in as rows and then transpose
   mat = np.concatenate((col0, col1, col2)).reshape(3,3)
   mat = np.transpose(mat)
   gyr_mat = np.matmul(up_north, mat)
   return gyr_mat


