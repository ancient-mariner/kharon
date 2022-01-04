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
#include <string.h>
#include <stdlib.h>
#include <assert.h>
#include <sys/stat.h>

#include "world_map.h"

// create 15-second level-2 files for grid squares where passibility is
//    ambiguous

static map_level2_type **level2_maps_ = NULL;
static map_level1_type *level1_map_ = NULL;


// determine which grid squares deserve a level-2 file and create a map
//    in memory for that
static void prepare_level2_maps(
      /* in     */ const uint32_t num
      )
{
   // for each map grid, check depth range. if grid contains water and
   //    shallowness is above threshold, mark grid square for creating
   //    level2 map
   // create level2 map in memory to store data
   // only look at one gebco section at a time, to ease memory requirements
   // each section is 90x90 degrees
   // level 1 and 2 maps have origin in top left
   for (uint32_t y=0; y<90; y++) {
      for (uint32_t x=0; x<90; x++) {
         // offset index to account for section location
         uint32_t xdeg = x + GEBCO_LEFT[num];
         uint32_t ydeg = y + GEBCO_TOP[num];
         uint32_t global_idx = xdeg + ydeg * 360;
         uint32_t local_idx = x + y * 90;
         map_level1_square_type *square = &level1_map_->grid[global_idx];
//printf("%d,%d   range %d -> %d\n", x, y, square->low, square->high);
         // depth threshold is positive to represent depth
         // level1 map squares represent elevation, not depth. so
         //    negative is required in comparison
         if ((square->low < 0) &&
               (square->high > -SUBMAP_DEPTH_THRESHOLD_METERS)) {
            // mark this square has having a level2 map
            square->flags = (uint16_t) (square->flags | MAP_FLAG_LEVEL_2);
            // make a level2 map
            map_level2_type *square2 = calloc(1, sizeof **level2_maps_);
            level2_maps_[local_idx] = square2;
         }
      }
   }
//exit(1);
}


// read one gebco section file and set depth in corresponding level2 maps
static void read_gebco_level2(
      /* in     */ const char *gebco_dir,
      /* in     */ const uint32_t num
      )
{
   // create buffer to store one line of file data
   // 90 degrees and 240 samples per degree
   const uint32_t N_COLS = 90u * 240u;
   uint32_t max_len = N_COLS * 10;  // each sample, w/ WS, is <10 chars
   char *buf = malloc(max_len);
   //
   char path[BUF_LEN];
   sprintf(path, "%s%s", gebco_dir, GEBCO_FILES[num]);
   FILE *fp = fopen(path, "r");
   if (!fp) {
      fprintf(stderr, "Unable to open '%s' for reading\n", path);
      exit(1);
   }
   printf("Reading '%s'\n", path);
   // read header
   // ignore first 6 lines
   for (uint32_t i=0; i<6; i++) {
      fgets(buf, (int) max_len, fp);
   }
   for (uint32_t ydeg=0; ydeg<90; ydeg++) {
      for (uint32_t y=0; y<240; y++) {
         fgets(buf, (int) max_len, fp);
         char *line = buf;
         for (uint32_t xdeg=0; xdeg<90; xdeg++) {
            ////////////////////////////////////
            // get level2 map (if present)
            uint32_t map_idx = xdeg + ydeg * 90u;
            map_level2_type *level2 = level2_maps_[map_idx];
            ////////////////////////////////////
            // loop through all samples in this grid square along this
            //    raster line. 240 samples
            for (uint32_t x=0; x<240; x++) {
               // get next sample
               char *tok = strtok(line, " ");
               line = NULL;
               if (level2 != NULL) {
                  // level2 map present -- update square
                  // need to invert y, as file rows increase northward
                  //    while map increases going southward
                  uint32_t map2_idx = x + y * 240u;
                  map_level2_square_type *square = &level2->grid[map2_idx];
                  int val = atoi(tok);
                  if (val < 0) {
                     square->min_depth = encode_submap_depth((uint16_t) -val);
//if (square->min_depth == 254) {
//   printf("%d,%d depth code %d  gebco depth %d\n", x, y, square->min_depth, val);
//}
//assert(square->min_depth != 254);
                     assert(square->min_depth != 255);
                  } else {
                     square->min_depth = 0;
                  }
               }
            }
         }
      }
   }
   /////////////////////////////////////////////
   // clean up
   fclose(fp);
   free(buf);
}


// for each allocated level2 map, create an output file and write the
//    contents to there
static void create_level2(
      /* in     */ const char *root_dir,
      /* in     */ const uint32_t num
      )
{
   printf("Writing level-2 maps\n");
   for (uint32_t y=0; y<90; y++) {
      for (uint32_t x=0; x<90; x++) {
         uint32_t map_idx = x + y * 90u;
         map_level2_type *map2 = level2_maps_[map_idx];
         if (map2 != NULL) {
            // a level2 map exists here -- create it
            map_grid_num_type grid_pos;
            grid_pos.akn_x = (uint16_t) (x + GEBCO_LEFT[num]);
            grid_pos.akn_y = (uint16_t) (y + GEBCO_TOP[num]);
            write_map_level2(root_dir, grid_pos, map2);
            // free this grid square and update map grid
            free(map2);
            level2_maps_[map_idx] = NULL;
         }
      }
   }
}


int main(int argc, char **argv)
{
   printf("Building initial level-2 maps based on GEBCO data\n");
   int rc = -1;
   char gebco[BUF_LEN] = { 0 };
   char map_dir[BUF_LEN] = { 0 };
   uint32_t n_squares = 90u * 90u;
   level2_maps_ = calloc(n_squares, sizeof *level2_maps_);
   //
   if (argc != 3) {
      printf("Usage: %s <map dir> <gebco dir>\n", argv[0]);
      goto end;
   }
   terminate_folder_path(argv[1], map_dir);
   terminate_folder_path(argv[2], gebco);
   printf("Creating GEBCO level2 at '%s'\n", map_dir);
   printf("Reading GEBCO files from '%s'\n", gebco);
   //
   if ((level1_map_ = load_map_level1(map_dir, NULL)) == NULL) {
      fprintf(stderr, "Bailing out\n");
      goto end;
   }
   // build level2 maps
   for (uint32_t i=0; i<8; i++) {
      // see what level2 maps are required for this region
      prepare_level2_maps(i);
      // failures reading gebco results in fatal error, so no need to check
      //    for errors here
      read_gebco_level2(gebco, i);
      // save maps
      create_level2(map_dir, i);
   }
   write_map_level1(map_dir, level1_map_);
   rc = 0;
end:
   if (level2_maps_ != NULL) {
      free(level2_maps_);
   }
   return rc;
}


