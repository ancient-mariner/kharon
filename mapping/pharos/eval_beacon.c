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
#include <signal.h>
#include <errno.h>
#include "routing/mapping.h"
#include "world_map.h"
#include "beacon.h"
#include "logger.h"

// lists beacons and their neighbors, along with path distance

#define DEFAULT_MAP_FOLDER    "/opt/kharon/mapping/master/"

// first row to be listed
static uint32_t watch_beacon_ = 0;

static char map_folder_[STR_LEN] = { DEFAULT_MAP_FOLDER };

static void parse_command_line(int argc, char *argv[])
{
   if (argc != 2) {
      goto usage;
   }
   watch_beacon_ = (uint32_t) atol(argv[1]);
   printf("Evaluating beacon %d\n", watch_beacon_);
   return;
usage:
   printf("Displays information about one beacon (in %s)\n",
         DEFAULT_MAP_FOLDER);
   printf("\n");
   printf("Usage: %s <beacon number>\n", argv[0]);
   printf("\n");
   exit(1);
}


int main(int argc, char **argv)
{
   set_log_dir_string("/tmp/");
   parse_command_line(argc, argv);
   set_world_map_folder_name(map_folder_);
   if (init_beacon_list() != 0) {
      printf("Failed to init beacon list. Bailing out.\n");
      return -1;
   }
   beacon_record_type *beacon = get_beacon_record(watch_beacon_);
   printf("%6d\t%.4f,%.4f\n", beacon->index, (double) beacon->akn_x,
         (double) beacon->akn_y);
   printf(" nbr   idx    dist\n");
   for (int32_t i=0; i<beacon->num_neighbors; i++) {
      printf("  %d  %6d  %.2f\n", i, beacon->neighbors[i].nbr_index,
            (double) beacon->neighbors[i].path_weight);
   }
}

