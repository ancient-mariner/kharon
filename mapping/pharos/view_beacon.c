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
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include "routing/mapping.h"
#include "world_map.h"
#include "beacon.h"
#include "logger.h"

// builds and exports 5-second map


int main(int argc, char **argv)
{
   if (argc != 2) {
      printf("Usage %s <beacon index>\n", argv[0]);
      exit(1);
   }
   set_log_dir_string("/tmp/");
   set_world_map_folder_name("/opt/kharon/mapping/master/");
   image_size_type size = { .x=720, .y=720 };
   errno = 0;
   uint32_t idx = (uint32_t) strtol(argv[1], NULL, 10);
   if (errno != 0) {
      fprintf(stderr, "Error parsing beacon index '%s': %s\n", argv[1],
            strerror(errno));
      exit(1);
   }
   if (init_beacon_list() != 0) {
      printf("Failed to init beacon list. Put on crash helmet\n");
   }
   uint32_t num_records = get_tot_num_beacons();
   if (idx >= num_records) {
      fprintf(stderr, "Request index (%d) is higher than number of "
            "records (%d)\n", idx, num_records);
      exit(1);
   }
   /////////////////////////////////////////////
   path_map_type *path_map = create_path_map(size);
   beacon_record_type *record = get_beacon_record(idx);
   akn_position_type apos = { .akn_x = (double) record->akn_x,
      .akn_y = (double) record->akn_y };
   world_coordinate_type wpos = convert_akn_to_world(apos);
   load_world_5sec_map(wpos, path_map);
   load_beacons_into_path_map(path_map);
   write_depth_map(path_map, "x_beacon_map.pnm");
   return 0;
}

