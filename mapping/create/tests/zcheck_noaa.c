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
#include <assert.h>
#include <errno.h>
#include <stdlib.h>
#include <zlib.h>

#include "world_map.h"

// make sure that .xyz files represent depth as expected
// expectations are that non-headered xyz files (ie, files w/o a line
//    declaring columns) store depth as negative, while headered files
//    store depth as positive
// Jan 2020 -- 5 files 'failed' this check, but analysis shows that 4 are
//    in clear tidal areas (Turnagain Arm, Knik Arm, Chesapeake Bay, SE AK)
//    and looked correct, and sampling showed another that appeared
//    to be correct (SW AK -- H11064.xyz), although that one is
//    questionable. all other 7500 were as expected


// parse line from file with structure lon,lat,depth
static void parse_no_header_line(
      /* in     */ const char *line,
      /*    out */       world_coordinate_type *latlon,
      /*    out */       float *depth_met
      )
{
   char buf[BUF_LEN];
   strcpy(buf, line);
   //////////////////////////////////////////////////////////////////
   char *lon_str = strtok(buf, " \t,");
   char *lat_str = strtok(NULL, " \t,");
   char *depth_str = strtok(NULL, " \t,");
   //////////////////////////////////////////////////////////////////
   // parse it
   errno = 0;
   latlon->lat = strtof(lat_str, NULL);
   if (errno != 0) {
      fprintf(stderr, "Failed to parse latitude '%s'\n", lat_str);
      return;
   }
   latlon->lon = strtof(lon_str, NULL);
   if (errno != 0) {
      fprintf(stderr, "Failed to parse longitude '%s'\n", lon_str);
      return;
   }
   // depth in meters. for non-headered files, this should be <= 0
   *depth_met = -strtof(lon_str, NULL);
   if (errno != 0) {
      fprintf(stderr, "Failed to parse depth '%s'\n", depth_str);
      return;
   }
}


// parse line from file with structure survey_id,lat,lon,depth,...
static void parse_headered_line(
      /* in     */ const char *line,
      /*    out */       world_coordinate_type *latlon,
      /*    out */       float *depth_met
      )
{
   char buf[BUF_LEN];
   strcpy(buf, line);
   //////////////////////////////////////////////////////////////////
   strtok(buf, " \t,"); // survey -- ignore
   char *lat_str = strtok(NULL, " \t,");
   char *lon_str = strtok(NULL, " \t,");
   char *depth_str = strtok(NULL, " \t,");
   //////////////////////////////////////////////////////////////////
   // parse it
   errno = 0;
   latlon->lat = strtof(lat_str, NULL);
   if (errno != 0) {
      fprintf(stderr, "Failed to parse latitude '%s'\n", lat_str);
      return;
   }
   latlon->lon = strtof(lon_str, NULL);
   if (errno != 0) {
      fprintf(stderr, "Failed to parse longitude '%s'\n", lon_str);
      return;
   }
   // depth in meters. for headered files, this should be >= 0
   *depth_met = strtof(depth_str, NULL);
   if (errno != 0) {
      fprintf(stderr, "Failed to parse depth '%s'\n", depth_str);
      return;
   }
//printf("%s  %s  %s\n", lon_str, lat_str, depth_str);
}


// 'has_header' indicates if file being processed has header line. if
//    so, has_header=1, otherwise =0
static void process_xyz_file(
      /* in out */       gzFile gz_in,
      /* in     */ const char * first_line
      )
{
   char buf[BUF_LEN];
   uint32_t neg_cnt = 0;
   uint32_t pos_cnt = 0;
   if (first_line == NULL) {
      if (gzgets(gz_in, buf, sizeof(buf)) == NULL) {
         // empty file -- bail out
         return;
      }
   } else {
      strcpy(buf, first_line);
   }
   // read & process rest of file
   do {
      float depth_met = 0.0f;
      world_coordinate_type latlon;
      if (first_line != NULL) {
         parse_no_header_line(buf, &latlon, &depth_met);
      } else {
         parse_headered_line(buf, &latlon, &depth_met);
      }
      if (depth_met < 0.0f) {
         neg_cnt++;
      } else {
         pos_cnt++;
      }
   } while (gzgets(gz_in, buf, sizeof(buf)) != NULL);
   if (neg_cnt > pos_cnt) {
      printf("NEGATIVE DEPTH\n");
   }
}

static void read_xyz(
      /* in     */ const char *fname
      )
{
   gzFile gz_in = NULL;
   printf("Processing '%s'\n", fname);
   gz_in = gzopen(fname, "rb");
   if (!gz_in) {
      // don't treat as a fatal error -- provide alert and fail gracefully
      fprintf(stderr, "Unable to open gz file '%s'\n", fname);
      goto end;
   }
   // read header line
   char buf[BUF_LEN];
   gzgets(gz_in, buf, sizeof(buf));
   // header should be one of 2 types -- either present, which usually
   //    starts with 'survey_id,lat,long,depth,...' or a number, which
   //    will be <long> <lat> <depth>\n
   if (buf[0] == 's') {
      process_xyz_file(gz_in, NULL);
   } else if (buf[0] == '-') {
      process_xyz_file(gz_in, buf);
   } else {
      fprintf(stderr, "Header row of '%s' unrecognized:\n'%s'\n",
            fname, buf);
      goto end;
   }
end:
   if (gz_in != NULL) {
      gzclose(gz_in);
   }
}

// process xyz files
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
//


int main(int argc, char **argv)
{
   printf("Building 5sec level-3 maps based on NOAA data\n");
   int rc = -1;
   char *catalog = NULL;
   FILE *cat_fp = NULL;  // for catalog file
   // allocate for entire world. US maps shouldn't take enough
   //    space to overwhelm memory. if there's a problem add logic to
   //    add from only a specific region, or just # of files processed
   //    at once
   //
   if (argc != 2) {
      printf("Usage: %s <catalog>\n", argv[0]);
      printf("\nwhere 'catalog' is file containing list of xyz.gz files "
            "to read\n");
      goto end;
   }
   catalog = argv[1];
   //////////////////////////////////////
   cat_fp = fopen(catalog, "r");
   if (!cat_fp) {
      fprintf(stderr, "Unable to open catalog file '%s'\n", catalog);
      goto end;
   }
   char buf[BUF_LEN];
   while (fgets(buf, sizeof(buf), cat_fp)) {
      char *fname = strtok(buf, " \n\r\t");
      if ((fname == NULL) || (fname[0] == '#') || (fname[0] == 0)) {
         continue;
      }
      read_xyz(fname);
   }
end:
   if (cat_fp != NULL) {
      fclose(cat_fp);
   }
   return rc;
}


