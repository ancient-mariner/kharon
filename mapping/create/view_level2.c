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
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "world_map.h"

// make sure file has expected extension
static void verify_map2_name(
      /* in    */ const char *fname
      )
{
   size_t len = strlen(fname);
   if (len <= 5) {
      fprintf(stderr, "'%s' is not a valid level2 file\n", fname);
      exit(1);
   }
   if (strcmp(".map2", &fname[len-5]) != 0) {
      fprintf(stderr, "'%s' is not a valid level2 file\n", fname);
      exit(1);
   }
}


int main(int argc, const char **argv)
{
   if (argc != 2) {
      printf("Usage: %s <level-2 map>\n", argv[0]);
      return 1;
   }
   const char *fname = argv[1];
   verify_map2_name(fname);
   FILE *fp = fopen(fname, "r");
   if (!fp) {
      fprintf(stderr, "Unable to open '%s'\n", fname);
      return 1;
   }
   map_level2_type map;
   fread(&map, sizeof map, 1, fp);
   fclose(fp);
   char name[BUF_LEN];
   sprintf(name, "%s.pgm", fname);
   printf("Writing to '%s'\n", name);
   FILE *ofp = fopen(name, "w");
   if (!ofp) {
      fprintf(stderr, "Unable to create '%s'\n", name);
      return 1;
   }
   fprintf(ofp, "P5\n%d %d\n255\n", MAP_LEVEL2_SIZE, MAP_LEVEL2_SIZE);
   uint8_t buf[MAP_LEVEL2_SIZE];
   for (uint32_t y=0; y<MAP_LEVEL2_SIZE; y++) {
      for (uint32_t x=0; x<MAP_LEVEL2_SIZE; x++) {
         uint32_t idx = (uint32_t) (x + y * MAP_LEVEL2_SIZE);
         int val = 255 - map.grid[idx].min_depth;
         if (val == 255) {
            val = 0;
         }
         buf[x] = (uint8_t) val;
         //printf("%d,%d     %d\n", x, y, map.grid[idx].min_depth);
      }
      fwrite(buf, 1, MAP_LEVEL2_SIZE, ofp);
   }
   fclose(ofp);
   return 0;
}

