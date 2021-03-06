/***********************************************************************
* This file is part of kharon <https://github.com/ancient-mariner/kharon>.
* Copyright (C) 2019-2022 Keith Godfrey
*
* kharon is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, version 3.
*
* kharon is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with kharon.  If not, see <http://www.gnu.org/licenses/>.
***********************************************************************/
#include <errno.h>
#include "dev_info.h"

// mag correction is performed on acquisition device
// when device is installed it can be subject to shipborne interference
// interference right now is assumed to be a simple XZ offset. in future
//    this should be converted to something more akin to compass card TODO
// function loads <env>/<device>/compass_correction if that file exists
//    and reads <x-offset <y-offset> from a single line as content
static void load_compass_correction(
      /* in out */       imu_class_type *imu
      )
{
   FILE *fp = open_config_file_ro2(get_environment(), imu->device_name,
         NULL, NULL, "compass_correction");
   if (fp == NULL) {
      goto end;
   }
   //
   char content[256];
   double x_offset = 0.0;
   double z_offset = 0.0;
   char * line = get_next_line(fp, content, 256);
   if (line != NULL) {
      errno = 0;
      char *endptr;
      x_offset = strtod(line, &endptr);
      if (errno != 0) {
         log_warn(imu->log, "Parse error reading x from compass "
               "correction '%s': %s", line, strerror(errno));
         goto end;
      }
      z_offset = strtod(endptr, NULL);
      if (errno != 0) {
         log_warn(imu->log, "Parse error reading z from compass "
               "correction '%s': %s", line, strerror(errno));
         goto end;
      }
   }
   log_info(imu->log, "Compass offset correction x=%.4f  z=%.4f",
         x_offset, z_offset);
   imu->x_mag_bias = x_offset;
   imu->z_mag_bias = z_offset;
end:
   if (fp != NULL) {
      fclose(fp);
   }
}

