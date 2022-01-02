#!/usr/bin/env python3
import sys
import os
import glob
"""
Creates an emulator environment to replay data recorded from an
   env_elanX recording (ie, hedgehog + mercury)
"""

########################################################################
# parse params and set names

def usage():
   print("%s <input dir> <start time> <length> <output dir>" % sys.argv[0])
   sys.exit(0)

# approx time required between frame acquisition and deliver to camera
#  app, in seconds
FRAME_PROCESSING_DELAY_SEC = 0.02

if len(sys.argv) != 5:
   usage()

try:
   in_dir = sys.argv[1]
   out_dir = sys.argv[4]
   start_t = float(sys.argv[2])
   dur = float(sys.argv[3])
except Exception:
   usage()

if not out_dir.endswith('/'):
   out_dir += '/'
if not in_dir.endswith('/'):
   in_dir += '/'

end_t = start_t + dur

hedge_imu = "hedgehog_imu"
cam_dirs = []
for i in range(4):
   cam_dirs.append("mercury%d_cam" % (i+1))
merc_gyrmag = "mercury1_gyrmag"
merc_acc = "mercury4_acc"
gopher_gps = "gopher_gps"

in_hedge_imu = in_dir + hedge_imu
in_cam_dirs = []
for i in range(4):
   in_cam_dirs.append(in_dir + cam_dirs[i])
in_merc_gyrmag = in_dir + merc_gyrmag
in_merc_acc = in_dir + merc_acc
in_gopher_gps = in_dir + gopher_gps

########################################################################
# initialize environment

# make sure input directory is present and has requisite files/dirs inside
def verify_in_dir():
   if not os.path.exists(in_dir):
      print("Input directory '%s' doesn't exist" % in_dir)
      sys.exit(1)
   if not os.path.exists(in_hedge_imu):
      print("Hedgehog IMU file '%s' doesn't exist" % in_hedge_imu)
      sys.exit(1)
   if not os.path.exists(in_merc_gyrmag):
      print("Mercury IMU file '%s' doesn't exist" % in_merc_gyrmag)
      sys.exit(1)
   if not os.path.exists(in_merc_acc):
      print("Mercury IMU file '%s' doesn't exist" % in_merc_acc)
      sys.exit(1)
   for name in in_cam_dirs:
      if not os.path.exists(name):
         print("Mercury camera directory '%s' doesn't exist" % name)
         sys.exit(1)
   print("Input files are present")

# make sure output directory is absent and then create it
def create_out_dir():
   try:
      os.makedirs(out_dir)
      print("Output directory created")
   except FileExistsError:
      print("Object already at output path '%s'" % out_dir)
      #sys.exit(1)

########################################################################
# copy data

def create_image_file(in_path, out_path):
   names = glob.glob(in_path + "/*.pgm")
   # sort by time (alphabetical sort doesn't work as '999.0' > '1000.0')
   names_x = []
   names_x_dict = {}
   for name in names:
      fname = name.split('/')[-1]
      t = float(fname[0:-4])
      key = "key/%010.3f" % t
      names_x.append(key)
      names_x_dict[key] = fname
   names_x.sort()
   earliest = None
   with open(out_path, "w") as f:
      f.write("%s/\n" % in_path)
      for name in names_x:
         fname = names_x_dict[name].split('/')[-1]
         t = float(fname[0:-4])
         if t < start_t:
            continue
         if t > end_t:
            break
         if earliest is None:
            earliest = t
         f.write("%s\n" % fname)
   return earliest
   

verify_in_dir()
create_out_dir()
first_frame = None
for i in range(len(in_cam_dirs)):
   src = in_cam_dirs[i]
   dest = out_dir + cam_dirs[i]
   t_new = create_image_file(src, dest)
   if t_new is None:
      print("No images found in %s over interval %.3f to %.3f" % 
            (src, start_t, end_t))
      continue
   t = t_new
   if first_frame is None or first_frame > t:
      first_frame = t

# to get start time for IMU data, get earliest frame and subtract
#  interval
imu_t = first_frame - FRAME_PROCESSING_DELAY_SEC

def copy_imu_file(src, dest, t0, t1):
   #print("Imu interval %f-%f" % (t0, t1))
   with open(src, 'r') as f:
      content = f.readlines()
   with open(dest, 'w') as f:
      for line in content:
         t = float(line.split(' ')[0])
         if t >= t0 and t <= t1:
            f.write(line)

def copy_gps_file(src, dest, t0, t1):
   #print("Imu interval %f-%f" % (t0, t1))
   with open(src, 'r') as f:
      content = f.readlines()
   with open(dest, 'w') as f:
      for line in content:
         t = float(line.split(' ')[0])
         if t >= t0 and t <= t1:
            f.write(line)

copy_imu_file(in_hedge_imu, out_dir + hedge_imu, imu_t, end_t)
copy_imu_file(in_merc_gyrmag, out_dir + merc_gyrmag, imu_t, end_t)
copy_imu_file(in_merc_acc, out_dir + merc_acc, imu_t, end_t)

copy_gps_file(in_gopher_gps, out_dir + gopher_gps, imu_t, end_t)


