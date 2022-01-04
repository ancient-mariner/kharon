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
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

// takes gebco 2020 ascii octant file and converts to 2-byte integer file
// 15-second resolution
// TODO give unit for stored value (iirc, GEBCO uses meters)

// IMPORTANT gebco files are _not_ useful for navigation. the values in
//    each 15-second grid appear to be the _average_ depth over that
//    grid, and _not_ the minimum depth. grid squares have been observed
//    that show a modest depth (eg, 10m) where in reality there are
//    rocks that break the surface. a known island is also not identified
//    on the maps (observations based on PNW/San Juan Islands area)

// they are still useful tools, such as for visualization or doing simulated
//    navigation

#define ASCII_COLS   21600
#define ASCII_ROWS   21600

int main(int argc, char **argv)
{
   int rc = -1;
   if (argc != 2) {
      printf("Converts gebco ascii file to int16 based binary\n");
      printf("Usage: %s <input file>\n", argv[0]);
      return 1;
   }
   /////////////////////////////////////////////////////////////////////
   // each token should be 6 or less chars, so 10 should be plenty
   uint32_t max_len = ASCII_COLS * 10;
   FILE *ofp = NULL;
   char *buf = malloc(max_len);
   const char *fname = argv[1];
   FILE *fp = fopen(fname, "r");
   if (!fp) {
      fprintf(stderr, "Unable to open '%s'\n", fname);
      goto err;
   }
   char out_name[256];
   int size = sprintf(out_name, "%s", fname);
   sprintf(&out_name[size-3], "bin");
   printf("Writing to '%s'\n", out_name);
   /////////////////////////////////
   ofp = fopen(out_name, "w");
   if (!ofp) {
      fprintf(stderr, "Unable to open '%s' for writing\n", out_name);
      goto err;
   }
   /////////////////////////////////////////////////////////////////////
   // read header
   // ignore first 6 lines
   for (uint32_t i=0; i<6; i++) {
      fgets(buf, (int) max_len, fp);
   }
   int16_t out_line[ASCII_COLS];
   for (uint32_t y=0; y<ASCII_ROWS; y++) {
      fgets(buf, (int) max_len, fp);
      char *line = buf;
      for (uint32_t x=0; x<ASCII_COLS; x++) {
         char *tok = strtok(line, " ");
         line = NULL;
         out_line[x] = (int16_t) atoi(tok);
      }
      fwrite(out_line, 2, ASCII_COLS, ofp);
   }
   rc = 0;
   goto end;
err:
   rc = -1;
end:
   if (fp) {
      fclose(fp);
      fp = NULL;
   }
   if (ofp) {
      fclose(ofp);
      ofp = NULL;
   }
   if (buf) {
      free(buf);
      buf = NULL;
   }
   return rc;
}

