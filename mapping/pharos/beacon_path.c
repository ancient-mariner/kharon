#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <errno.h>
#include "pin_types.h"
#include "logger.h"
#include "routing/mapping.h"
#include "world_map.h"
#include "beacon.h"


// builds beacon neighbor association
// one by one, loads map around beacon and stores neighbors and path weights
//    of neighbors in the beacon
// beacon.bin is loaded into memory, modified, and written back out
// only beacons with -1 neighbors are processed


#define DEFAULT_MAP_FOLDER    "/opt/kharon/mapping/master/"

// first row to be processed
static uint32_t start_row_ = 0;
// last row to process
static uint32_t last_row_ = 179;

static char map_folder_[STR_LEN] = { DEFAULT_MAP_FOLDER };

// quit flag. when this is set main loop will exit, writing present state
//    of beacons
static int32_t quit_ = 0;

static void beacon_signal_exit(int sig)
{
   static int int_count = 0;
   if (sig == SIGINT) {
      int_count++;
   }
   if (int_count > 2) {
      // induce hard exit
      exit(1);
   }
   printf("Exit signal received\n");
   quit_ = 1;
}


static int write_output_file(void)
{
   int rc = -1;
   char outfile[STR_LEN];
   // open files later. if first opens, we can assume that 2nd will
   sprintf(outfile, "%s%s.%d-%d", map_folder_, BEACON_FILE, 
         start_row_, last_row_);
   printf("Writing updated beacons to '%s'\n", outfile);
   if (quit_ != 0) {
      printf("---- SHORT WRITE ----\n");
   }
   FILE *ofp = fopen(outfile, "wb");
   if (ofp == NULL) {
      fprintf(stderr, "Bad news -- failed to write output file. %s\n",
            strerror(errno));
      goto end;
   }
   uint32_t size = get_tot_num_beacons() * BEACON_BIN_RECORD_SIZE_BYTES;
   void *buf = get_beacon_record(0);
   if (fwrite(buf, size, 1, ofp) != 1) {
      fprintf(stderr, "Uh, oh -- unspecified error writing output file.\n");
      goto end;
   }
   rc = 0;
end:
   if (ofp) {
      fclose(ofp);
   }
   return rc;
}


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
   while ((opt = getopt(argc, argv, "e:f:r:h")) != -1) {
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
   printf("\n");
   printf("Usage: %s [-f <map folder>] [-r <start row>] [-e <last row>] \n", 
         argv[0]);
   printf("\n");
   printf("where:\n");
   printf("   f  path to map folder. defaults to %s\n", DEFAULT_MAP_FOLDER);
   printf("   r  row to start processing from (defaults to 0)\n");
   printf("   e  last row to process (defaults to 179)\n");
   printf("   h  prints this help message\n");
   printf("Process exits gracefully on receiving SIGINT(2x) or SIGUSR1\n");
   exit(1);
}


int main(int argc, char **argv)
{
   set_log_dir_string("/tmp/");
   parse_command_line(argc, argv);
   image_size_type size = { .x=720, .y=720 };
   path_map_type *path_map = create_path_map(size);
   if (init_beacon_list() != 0) {
      printf("Failed to init beacon list. Bailing out.\n");
      return -1;
   }
   uint32_t processing_row = start_row_;
   // listen for signal to gracefully exit
   signal(SIGUSR1, beacon_signal_exit);
   signal(SIGINT, beacon_signal_exit);
   // process all beacons in each row
   while (processing_row <= last_row_) {
      uint32_t start_idx = get_start_idx(processing_row);
      uint32_t num_beacons = get_num_beacons(processing_row);
printf("Processing row %d -- %d has %d\n", processing_row, start_idx, num_beacons);
      if (quit_ != 0) {
printf("Quit detected -- bailing out\n");
         last_row_ = processing_row;
         goto done;
      }
      for (uint32_t i=start_idx; i<start_idx+num_beacons; i++) {
         if (quit_ != 0) {
printf("Quit detected -- bailing out\n");
            last_row_ = processing_row;
            goto done;
         }
         beacon_record_type *home_rec = get_beacon_record(i);
         if (home_rec->num_neighbors >= 0) {
            // record has been processed -- nothing more to do
            continue;
         }
         // build path map around this beacon. 
         akn_position_type apos = { .akn_x = (double) home_rec->akn_x, 
               .akn_y = (double) home_rec->akn_y };
         world_coordinate_type wpos = convert_akn_to_world(apos);
         load_world_5sec_map(wpos, path_map);
         load_beacons_into_path_map(path_map);
         path_map->destination = apos;
         path_map->dest_pix.x = 360;
         path_map->dest_pix.y = 360;
         trace_route_simple(path_map, wpos);
         // for all neighbors, if path weight is > 0, add to record
         uint32_t out_idx = 0;
printf("Beacon %d at %.4f,%.4f has neighbors: (of %d)\n", home_rec->index, (double) home_rec->akn_x, (double) home_rec->akn_y, path_map->num_beacons);
         for (uint32_t b=0; b<path_map->num_beacons; b++) {
            map_beacon_reference_type *ref = &path_map->beacon_ref[b];
            if (ref->index == b) {
               continue;   // don't add self to own list
            }
//            calculate_beacon_map_position(path_map, b);
            // calculate beacon map position
            world_coordinate_type beac_wpos = 
                  convert_akn_to_world(ref->coords);                      
            ref->pos_in_map = get_pix_position_in_map(path_map, beac_wpos); 
            //
            uint32_t map_node_idx = (uint32_t) (ref->pos_in_map.x + 
                  ref->pos_in_map.y * path_map->size.x);
printf("  neighbor %d at %d,%d\n", b, ref->pos_in_map.x, ref->pos_in_map.y);
            path_map_node_type *node = &path_map->nodes[map_node_idx];
            if (node->weight > 0.0f) {
printf("    %.4f,%.4f (%d)  wt %.1f\n", ref->coords.akn_x, ref->coords.akn_y, ref->index, (double) node->weight);
               home_rec->neighbors[out_idx].nbr_index = ref->index;
               home_rec->neighbors[out_idx].path_weight = node->weight;
               out_idx++;
            }
         }
//char buf[STR_LEN];
//sprintf(buf, "path_%d.pnm", i);
//write_path_map(path_map, buf);
if (out_idx == 0) {
   printf("Beacon at %.4f,%.4f (%d) no neighbors:\n", (double) home_rec->akn_x, (double) home_rec->akn_y, i);
}
         home_rec->num_neighbors = (int32_t) out_idx;
//write_depth_map(path_map, "a.pnm");
//write_path_map(path_map, "b.pnm");
//write_direction_map(path_map, "c.pnm");
//return 0;
      }
      processing_row++;
   }
done:
   return write_output_file();
}

