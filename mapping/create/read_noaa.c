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

// create 5-second level-3 files for grid squares from NOAA data
// input is text file with each line starting with file name to
//    load. all files should be checked with noaa/read_gz to verify
//    format of file. file's header should either be 'survery_id,lat,...'
//    or #,#,#. In the first case, columns 1,2,3 are used for
//    lat,lon,depth, and in the later case, cols 2,1,3 are used for the
//    same
// ideally files listed in file will have some goegraphic order but
//    that's not necessarily the case. not a big deal. because files
//    can be goegraphically disordered, a limited number of noaa files
//    are processed at a time, to reduce overloading memory
// update level-1 and level-2 files as necessary

// much of the logic has been moved to a separate C file so that it
//    can be shared by noaa-chart and shoreline-processing code
#include "noaa_common.c"


////////////////////////////////////////////////////////////////////////
// process xyz files

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
   // depth in meters. for headered files, this should be >= 0, although
   //    some negative values are present (4 of 5 occur in area of shallow
   //    bays)
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
      map_subgrid_pos_type sub_pos;
      akn_position_type akn = convert_latlon_to_akn(latlon);
      map_grid_num_type grid_pos = convert_akn_to_grid(akn, &sub_pos);
      //////////////////////////////////////////////////////////////////
      // see if water is shallow enough in grid that we need to update
      //    anything
      if (depth_met < 0.0f) {
         // negative depth is positive elevation. call this 0 depth
         depth_met = 0.0f;
      }
      uint16_t depth = (uint16_t) depth_met;
      map_level1_square_type *square1 = get_square1(grid_pos);
      if ((square1->flags & SUBMAP_AVAILABLE_MASK) == 0) {
         // no submap. verify that depth is sufficient that
         //    we don't need one
         if ((depth > 0) && (depth < SUBMAP_DEPTH_THRESHOLD_METERS)) {
            // we're supposed to have a submap here. for now, report
            //    problem. if it's bad enough then find solution
            //    (either create submap or lower submap threshold so
            //    that exceptions like this are OK)
            no_submap_[grid_pos.akn_x + grid_pos.akn_y * 360] = depth;
         }
         continue;
      }
      int16_t elev = (int16_t) (-depth_met);
      if (square1->high < elev) {
         square1->high = elev;
         level1_modified_ = 1;
      } else if (square1->low > elev) {
         square1->low = elev;
         level1_modified_ = 1;
      }
      //////////////////////////////////////////////////////////////////
      uint8_t code = encode_submap_depth(depth);
if ((abs(elev) > 10000) || (code == 254)) {
   printf("Excess depth %d (%.1f) meters, code %d\n", elev, (double) depth_met, code);
   assert(1 == 0);
}
//printf("  depth %f (%d)   elevation %d   code %d\n", (double) depth_met, depth, elev, code);
//printf("%.5f,%.5f   %.1f   ", (double) latlon.lon, (double) latlon.lat, (double) elevation);
      map_level3_square_type *square3 = get_square3(grid_pos, sub_pos);
      if ((square3->min_depth == 255) || (code < square3->min_depth)) {
         square3->min_depth = code;
         // level3 is changed -- check to see if level 2 requires update
         map_level2_square_type *square2 = get_square2(grid_pos, sub_pos);
         if ((square2 != NULL) && (square2->min_depth < code)) {
            square2->min_depth = code;
            uint32_t world_idx =
                  (uint32_t) (grid_pos.akn_x + grid_pos.akn_y * 360);
            level2_modified_[world_idx] = 1;
         }
      }
   } while (gzgets(gz_in, buf, sizeof(buf)) != NULL);
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
   uint32_t n_squares = 360u * 180u;
   level2_maps_ = calloc(n_squares, sizeof *level2_maps_);
   level2_modified_ = calloc(n_squares, sizeof *level2_modified_);
   level3_maps_ = calloc(n_squares, sizeof *level3_maps_);
   //
   if (argc != 3) {
      printf("Usage: %s <map dir> <catalog>\n", argv[0]);
      printf("\nwhere 'catalog' is file containing list of xyz.gz files "
            "to read\n");
      goto end;
   }
   terminate_folder_path(argv[1], map_dir_);
   catalog = argv[2];
   //////////////////////////////////////
   printf("Creating level3 at '%s' using '%s'\n", map_dir_, catalog);
   //
   if ((level1_map_ = load_map_level1(map_dir_, NULL)) == NULL) {
      fprintf(stderr, "Bailing out\n");
      goto end;
   }
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
      check_resources(0);
   }
   // save everything
   check_resources(1);
end:
   for (int x=0; x<360; x++) {
      for (int y=0; y<180; y++) {
         int i = x + 360 * y;
         if (no_submap_[i] != 0) {
            fprintf(stderr, "Level3 depth shallow but no submap present. "
                  "%d,%d   depth %d\n", x, y, no_submap_[i]);
         }
      }
   }
   if (cat_fp != NULL) {
      fclose(cat_fp);
   }
   if (level2_maps_ != NULL) {
      free(level2_maps_);
   }
   if (level3_maps_ != NULL) {
      free(level3_maps_);
   }
   if (level2_modified_ != NULL) {
      free(level2_modified_);
   }
   return rc;
}


