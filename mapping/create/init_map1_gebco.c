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
#include <sys/stat.h>

#include "world_map.h"

// uses 15-second GEBCO map files to initialize a level-1 map
// for data, go to:
//    https://www.gebco.net/data_and_products/gridded_bathymetry_data/
// get the 'Esri ASCII raster format' files. these are 8 files that cover
//    180x90 degrees, with depth data stored as text
// those files should be stored in one directory, and the names of the
//    files should be listed in GEBCO_FILES
// there are hard-coded assumptions that the gebco files have 15-arc second
//    resolution


// reads one gebco section file and update depth in the map grid squares
static void read_gebco_level1(
      /* in out */       map_level1_type *map,
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
   for (uint32_t ydeg=GEBCO_TOP[num]; ydeg<GEBCO_TOP[num]+90; ydeg++) {
      for (uint32_t y=0; y<240; y++) {
         fgets(buf, (int) max_len, fp);
         char *line = buf;
         for (uint32_t xdeg=GEBCO_LEFT[num]; xdeg<GEBCO_LEFT[num]+90; xdeg++) {
            ////////////////////////////////////
            // get map grid
            uint32_t map_idx = xdeg + ydeg * 360u;
            map_level1_square_type *square = &map->grid[map_idx];
            ////////////////////////////////////
            // initialize or restore high/low values for this grid square
            int32_t high, low;
            if (y == 0) {
               // first sample in grid -- default to ridiculous values
               high = -100000;
               low = 100000;
            } else {
               // pull high and low from map's grid square
               high = square->high;
               low = square->low;
            }
            ////////////////////////////////////
            // loop through all samples in this grid square along this
            //    raster line. 240 samples
            for (uint32_t x=0; x<240; x++) {
               // get next sample
               char *tok = strtok(line, " ");
               line = NULL;
               int val = atoi(tok);
               high = val > high ? val : high;
               low = val < low ? val : low;
            }
//if (y == 239) {
//   printf("%d,%d range low %d   high %d\n", xdeg, ydeg, low, high);
//}
            // remember state for this grid square
            square->high = (int16_t) high;
            square->low = (int16_t) low;
         }
      }
   }
   /////////////////////////////////////////////
   // clean up
   fclose(fp);
   free(buf);
}


// create new level-1 map based on gebco data
static void init_level1(
      /* in     */ const char *gebco_dir,
      /* in     */ const char *root_dir
      )
{
   /////////////////////////////////////////////////////////////////////
   // prepare data
   char map_name[BUF_LEN];
   sprintf(map_name, "%s%s", root_dir, MAP_LEVEL_1_FILE_NAME);
   // see if level-1 file exists. if so, bail out
   if (file_exists(map_name)) {
      fprintf(stderr, "'%s' looks like an existing map directory. Refusing "
            "to re-init\n", root_dir);
      exit(1);
   }
   /////////////////////////////////////////////////////////////////////
   // process gebco data and write output
   map_level1_type *map1 = NULL;
   map1 = calloc(1, sizeof *map1);
   for (uint32_t i=0; i<8; i++) {
      read_gebco_level1(map1, gebco_dir, i);
   }
   write_map_level1(root_dir, map1);
   free(map1);
}


int main(int argc, char **argv)
{
   printf("Initializing level-1 map using GEBCO data\n");
   char gebco[BUF_LEN] = { 0 };
   char target[BUF_LEN] = { 0 };
   if (argc != 3) {
      printf("Usage: %s <map dir> <gebco dir>\n", argv[0]);
      return 1;
   }
   terminate_folder_path(argv[2], gebco);
   terminate_folder_path(argv[1], target);
   printf("Creating map at '%s'\n", target);
   printf("Reading GEBCO files from '%s'\n", gebco);
   init_level1(gebco, target);
   return 0;
}


