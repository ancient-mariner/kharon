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
#include "logger.h"

// builds and exports 5-second map

#include "../world_map.c"

const world_coordinate_type SQUAL = { .lon=-122.51667f, .lat=48.7417 };
const world_coordinate_type CHUCK = { .lon=-122.49750, .lat=48.6810 };

const world_coordinate_type POINT_BOB = { .lon=-123.07f, .lat=48.965f };
const world_coordinate_type SUCIA = { .lon=-122.87f, .lat=48.757f };
const world_coordinate_type FH = { .lon=-122.993f, .lat=48.547f };
const world_coordinate_type SYDNEY = { .lon=-123.374f, .lat=48.641f };
const world_coordinate_type BHAM = { .lon=-122.523f, .lat=48.744f };
const world_coordinate_type ANACORTES = { .lon=-122.582f, .lat=48.517f };
const world_coordinate_type PT_TOWNSEND = { .lon=-122.765f, .lat=48.093f };
const world_coordinate_type PT_ANGELES = { .lon=-123.454f, .lat=48.148f };
const world_coordinate_type VICTORIA = { .lon=-123.444f, .lat=48.412f };

int main(int argc, char **argv)
{
   (void) argc;
   (void) argv;
   set_log_dir_string("/tmp/");
   image_size_type size = { .x=720, .y=720 };
   set_world_map_folder_name("/opt/kharon/mapping/master/");
   //
   path_map_type *path_map = create_path_map(size);
   if (init_beacon_list() != 0) {
      printf("Failed to init beacon list. Put on crash helmet\n");
   }
   load_world_5sec_map(FH, path_map);
   load_beacons_into_path_map(path_map);
   write_depth_map(path_map, "x_depth.pnm");
   return 0;
}

