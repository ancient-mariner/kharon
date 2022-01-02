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
static uint32_t start_row_ = 0;
// last row to list
static uint32_t last_row_ = 179;

static char map_folder_[STR_LEN] = { DEFAULT_MAP_FOLDER };


static void verify_setup(void)
{
   if (start_row_ >= 180) {
      fprintf(stderr, "Processing row must be on [0,179]\n");
      goto err;
   }
   // make sure folder name ends with '/'
   size_t len = strlen(map_folder_);
   if (len > 200) {
      fprintf(stderr, "Folder name is too long\n");
      goto err;
   }
   if (len == 0) {
      fprintf(stderr, "Folder name is missing\n");
      goto err;
   }
   if (map_folder_[len-1] != '/') {
      map_folder_[len] = '/';
      map_folder_[len+1] = 0;
   }
   set_world_map_folder_name(map_folder_);
   return;
err:
   exit(1);
}

static void parse_command_line(int argc, char *argv[])
{
   int opt;
   while ((opt = getopt(argc, argv, "f:r:e:h")) != -1) {
      switch (opt) {
         case 'f':
         {
            const char *name = optarg;
            strncpy(map_folder_, name, STR_LEN);
            break;
         }
         case 'h':
            goto usage;
         case 'r':
         {
            const char *row = optarg;
            errno = 0;
            start_row_ = (uint32_t) strtol(row, NULL, 10);
            if (errno != 0) {
               goto usage;
            }
            break;
         }
         case 'e':
         {
            const char *row = optarg;
            errno = 0;
            last_row_ = (uint32_t) strtol(row, NULL, 10);
            if (errno != 0) {
               goto usage;
            }
            break;
         }
         default:
            goto usage;
      };
   };
   // check extra arguments -- there should be none
   // one way to do this: for(; optind < argc; optind++) {
   if (optind < argc) {
      goto usage;
   }
   verify_setup();
   return;
   /////////////////////////////////////////////
usage:
   printf("Associates beacon with its neighbors, storing path distance\n");
   printf("Lists beacons in specified rows, showing neighbors and "
         "path distances\n");
   printf("\n");
   printf("Usage: %s [-f <map folder>] [-r <start row>] [-e <last row>] \n", 
         argv[0]);
   printf("\n");
   printf("where:\n");
   printf("   f  path to map folder. defaults to %s\n", DEFAULT_MAP_FOLDER);
   printf("   r  row to start processing from (defaults to 0)\n");
   printf("   e  last row to process (defaults to 179)\n");
   printf("   h  prints this descriptive output (ie, help)\n");
   exit(1);
}


int main(int argc, char **argv)
{
   set_log_dir_string("/tmp/");
   parse_command_line(argc, argv);
   if (init_beacon_list() != 0) {
      printf("Failed to init beacon list. Bailing out.\n");
      return -1;
   }
   uint32_t processing_row = start_row_;
   // process all beacons in each row
   while (processing_row <= last_row_) {
      uint32_t start_idx = get_start_idx(processing_row);
      uint32_t num_beacons = get_num_beacons(processing_row);
printf("Processing row %d -- %d has %d ######################\n", processing_row, start_idx, num_beacons);
printf("  start %d  num %d\n", start_idx, num_beacons);
      for (uint32_t i=start_idx; i<start_idx+num_beacons; i++) {
         beacon_record_type *beacon = get_beacon_record(i);
         if (beacon->num_neighbors <= 0) {
            // no neighbors so nothing to report
            continue;
         }
         printf("Beacon %d at %.4f,%.4f has neighbors:\n", beacon->index,
               (double) beacon->akn_x, (double) beacon->akn_y);
         for (int32_t b=0; b<beacon->num_neighbors; b++) {
            beacon_neighbor_type nbr = beacon->neighbors[b];
            beacon_record_type *neighbor = get_beacon_record(nbr.nbr_index);
            if (nbr.path_weight > 0.0f) {
               printf("    %.4f,%.4f (%d)  dist %.1f\n", 
                     (double) neighbor->akn_x, (double) neighbor->akn_y, 
                     nbr.nbr_index, (double) nbr.path_weight);
            }
         }
      }
      processing_row++;
   }
}

