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
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <string.h>
#include <math.h>
#include "world_map.h"

#include "../common.c"

static int32_t check_grid_pos(
      /* in     */ const double longitude,
      /* in     */ const double latitude,
      /* in     */ const int32_t grid_x,
      /* in     */ const int32_t grid_y,
      /* in     */ const char * label
      )
{
   world_coordinate_type wp;
   wp.lat = latitude;
   wp.lon = longitude;
   akn_position_type akn = convert_latlon_to_akn(wp);
   map_grid_num_type pos = convert_akn_to_grid(akn, NULL);
   if ((pos.akn_x == grid_x) && (pos.akn_y == grid_y)) {
      return 0;
   }
   fprintf(stderr, "%s grid error for %.1f, %.1f. Expected %d,%d: Got %d,%d\n",
         label, longitude, latitude, grid_x, grid_y, pos.akn_x, pos.akn_y);
   return 1;
}

static int32_t check_world_pos(
      /* in     */ const double akn_x,
      /* in     */ const double akn_y,
      /* in     */ const double world_x,
      /* in     */ const double world_y,
      /* in     */ const char * label
      )
{
   akn_position_type akn = { .akn_x = akn_x, .akn_y = akn_y };
   world_coordinate_type world = convert_akn_to_world(akn);
   if ((fabs(world_x - world.lon) > 0.001) ||
         (fabs(world_y - world.lat) > 0.001)) {
      fprintf(stderr, "%s akn->world error for %.1f, %.1f. Expected "
            "%.3f,%.3f, got %.3f,%.3f\n", label, akn_x, akn_y,
            world_x, world_y, world.lon, world.lat);
      return 1;
   }
   return 0;
}


static int32_t test_convert_world_coordinate_type(void)
{
   int32_t errs = 0;
   printf("testing convert_world_coordinate_type\n");
   /////////////////////////////////////////////////////////////
   errs += check_grid_pos(-122.5, 49.1, 57, 40, "YVR");
   // converts akn to world coord
   errs += check_world_pos(57.5, 40.9, -122.5+360.0, 49.1, "YVR");
   /////////////////////////////////////////////////////////////
   if (errs == 0) {
      printf("    passed\n");
   } else {
      printf(" ** %d error(s)\n", errs);
   }
   return errs;
}

////////////////////////////////////////////////////////////////////////

static int32_t test_file_exists(void)
{
   int32_t errs = 0;
   printf("testing file_exists\n");
   /////////////////////////////////////////////////////////////
   if (file_exists("/tmp/") == 0) {
      fprintf(stderr, "'/tmp/' said not to exist\n");
      errs++;
   }
   if (file_exists("/fubar/") != 0) {
      fprintf(stderr, "'/fubar/' said to exist\n");
      errs++;
   }
   /////////////////////////////////////////////////////////////
   if (errs == 0) {
      printf("    passed\n");
   } else {
      printf(" ** %d error(s)\n", errs);
   }
   return errs;
}

////////////////////////////////////////////////////////////////////////

static int32_t test_terminate_folder_path(void)
{
   int32_t errs = 0;
   printf("testing terminate_folder_path\n");
   /////////////////////////////////////////////////////////////
   char in[256];
   char out[256];
   strcpy(in, "/one/flew/over/the/nest");
   terminate_folder_path(in, out);
   size_t len_in = strlen(in);
   size_t len_out = strlen(out);
   if (out[len_out-1] != '/') {
      fprintf(stderr, "%s is not terminated by '/'\n", out);
      errs++;
   }
   if (len_out != (len_in + 1)) {
      fprintf(stderr, "'%s' is not one char longer than '%s' (%d vs %d)\n",
            out, in, (int32_t) len_in, (int32_t) len_out);
      errs++;
   }
   //
   strcpy(in, "/one/flew/over/the/nest/");
   terminate_folder_path(in, out);
   len_in = strlen(in);
   len_out = strlen(out);
   if (out[len_out-1] != '/') {
      fprintf(stderr, "%s is not terminated by '/'\n", out);
      errs++;
   }
   if (len_out != len_in) {
      fprintf(stderr, "'%s' is different length than '%s' (%d vs %d)\n",
            in, out, (int32_t) len_in, (int32_t) len_out);
      errs++;
   }
   /////////////////////////////////////////////////////////////
   if (errs == 0) {
      printf("    passed\n");
   } else {
      printf(" ** %d error(s)\n", errs);
   }
   return errs;
}

////////////////////////////////////////////////////////////////////////

static int32_t test_code_submap_depth(void)
{
   int32_t errs = 0;
   printf("testing [en|de]code_submap_depth\n");
   /////////////////////////////////////////////////////////////
   uint8_t code, expected_code;
   uint16_t depth, expected_depth;
   depth = 50;
   code = encode_submap_depth(depth);
   expected_code = 50;
   if (code != expected_code) {
      fprintf(stderr, "Encoding %d yielded %d, not %d\n", depth, code,
            expected_code);
      errs++;
   }
   expected_depth = 50;
   depth = decode_submap_depth(code);
   if (depth != expected_depth) {
      fprintf(stderr, "Decoding %d yielded %d, not %d\n", code, depth,
            expected_depth);
      errs++;
   }
   /////////////////////////////
   depth = 500;
   code = encode_submap_depth(depth);
   expected_code = 140;
   if (code != expected_code) {
      fprintf(stderr, "Encoding %d yielded %d, not %d\n", depth, code,
            expected_code);
      errs++;
   }
   expected_depth = 500;
   depth = decode_submap_depth(code);
   if (depth != expected_depth) {
      fprintf(stderr, "Decoding %d yielded %d, not %d\n", code, depth,
            expected_depth);
      errs++;
   }
   /////////////////////////////
   depth = 5000;
   code = encode_submap_depth(depth);
   expected_code = 194;
   if (code != expected_code) {
      fprintf(stderr, "Encoding %d yielded %d, not %d\n", depth, code,
            expected_code);
      errs++;
   }
   expected_depth = 5000;
   depth = decode_submap_depth(code);
   if (depth != expected_depth) {
      fprintf(stderr, "Decoding %d yielded %d, not %d\n", code, depth,
            expected_depth);
      errs++;
   }
   /////////////////////////////
   depth = 12000;
   code = encode_submap_depth(depth);
   expected_code = 254;
   if (code != expected_code) {
      fprintf(stderr, "Encoding %d yielded %d, not %d\n", depth, code,
            expected_code);
      errs++;
   }
   expected_depth = 11000;
   depth = decode_submap_depth(code);
   if (depth != expected_depth) {
      fprintf(stderr, "Decoding %d yielded %d, not %d\n", code, depth,
            expected_depth);
      errs++;
   }
   /////////////////////////////
   code = 255;
   expected_depth = 0xffff;
   depth = decode_submap_depth(code);
   if (depth != expected_depth) {
      fprintf(stderr, "Decoding %d yielded %d, not %d\n", code, depth,
            expected_depth);
      errs++;
   }
   /////////////////////////////////////////////////////////////
   if (errs == 0) {
      printf("    passed\n");
   } else {
      printf(" ** %d error(s)\n", errs);
   }
   return errs;
}

////////////////////////////////////////////////////////////////////////

static int32_t test_convert_akn_to_map_coord(void)
{
   int32_t errs = 0;
   printf("testing convert_akn_to_map_coord\n");
   /////////////////////////////////////////////////////////////
   akn_position_type pos;
   akn_position_type map_center;
   map_center.akn_x = 35.0;
   map_center.akn_y = 29.5;
   map_coordinate_type map_pos, expected;
   /////////////////////////////
   // x out of map
   pos.akn_x = 32.0;
   pos.akn_y = 29.5;
   if (convert_akn_to_map_coord(pos, map_center, &map_pos) == 0) {
      fprintf(stderr, "Falsely reported that %.3f,%.3f is in map centered "
            "at %.3f,%.3f\n", pos.akn_x, pos.akn_y, map_center.akn_x,
            map_center.akn_y);
      errs++;
   }
   pos.akn_x = 38.0;
   if (convert_akn_to_map_coord(pos, map_center, &map_pos) == 0) {
      fprintf(stderr, "Falsely reported that %.3f,%.3f is in map centered "
            "at %.3f,%.3f\n", pos.akn_x, pos.akn_y, map_center.akn_x,
            map_center.akn_y);
      errs++;
   }
   /////////////////////////////
   // y out of map
   pos.akn_x = 35.0;
   pos.akn_y = 28.99;
   if (convert_akn_to_map_coord(pos, map_center, &map_pos) == 0) {
      fprintf(stderr, "Falsely reported that %.3f,%.3f is in map centered "
            "at %.3f,%.3f\n", pos.akn_x, pos.akn_y, map_center.akn_x,
            map_center.akn_y);
      errs++;
   }
   pos.akn_y = 30.01;
   if (convert_akn_to_map_coord(pos, map_center, &map_pos) == 0) {
      fprintf(stderr, "Falsely reported that %.3f,%.3f is in map centered "
            "at %.3f,%.3f\n", pos.akn_x, pos.akn_y, map_center.akn_x,
            map_center.akn_y);
      errs++;
   }
   /////////////////////////////
   // x and y in map
   pos.akn_x = 34.5;
   pos.akn_y = 29.5;
   // 29.5 is 60.5 lat
   // scale is 2.0308
   // width is 2.0308 degs, half-width is 1.0154
   // 0.5 from center is 0.5154 from left, which is .50758th from left
   //    edge to center, which is 360 pixels, so 0.50758*360=182.73
   expected.x = 183;
   expected.y = 360;
   if (convert_akn_to_map_coord(pos, map_center, &map_pos) != 0) {
      fprintf(stderr, "Falsely reported that %.3f,%.3f is not in map "
            "centered at %.3f,%.3f\n", pos.akn_x, pos.akn_y,
            map_center.akn_x, map_center.akn_y);
      errs++;
   }
   if ((map_pos.x != expected.x) || (map_pos.y != expected.y)) {
      fprintf(stderr, "A Map position reported at %d,%d, not %d,%d\n",
            map_pos.x, map_pos.y, expected.x, expected.y);
      errs++;
   }



   /////////////////////////////////////////////////////////////
   if (errs == 0) {
      printf("    passed\n");
   } else {
      printf(" ** %d error(s)\n", errs);
   }
   return errs;
}

////////////////////////////////////////////////////////////////////////

static int32_t test_initialize_60x60_map(void)
{
   int32_t errs = 0;
   printf("testing initialize_60x60_map\n");
   /////////////////////////////////////////////////////////////
   world_coordinate_type wp;
   map_level3_type map;
   wp.lat = 88.0;
   if (initialize_60x60_map(wp, &map) == 0) {
      fprintf(stderr, "Map centered at latitude %.2f not flagged as "
            "autocompleted\n", wp.lat);
      errs++;
   }
   wp.lat = -85.0;
   if (initialize_60x60_map(wp, &map) == 0) {
      fprintf(stderr, "Map centered at latitude %.2f not flagged as "
            "autocompleted\n", wp.lat);
      errs++;
   }
   wp.lat = 5.0;
   if (initialize_60x60_map(wp, &map) != 0) {
      fprintf(stderr, "Map centered at latitude %.2f flagged as "
            "autocompleted\n", wp.lat);
      errs++;
   }
   /////////////////////////////////////////////////////////////
   if (errs == 0) {
      printf("    passed\n");
   } else {
      printf(" ** %d error(s)\n", errs);
   }
   return errs;
}

////////////////////////////////////////////////////////////////////////

static int32_t test_get_deg_per_nm(void)
{
   int32_t errs = 0;
   printf("testing get_deg_per_nm\n");
   /////////////////////////////////////////////////////////////
   world_coordinate_type wp;
   double dpnm, expected;
   wp.lon = 0.0;
   wp.lat = 0.0;
   dpnm = get_deg_per_nm(wp);
   expected = 0.0166673;
   if (fabs(dpnm - expected) > 0.0001) {
      fprintf(stderr, "Latitude %.3f has %.5f, expected %.5f\n",
            wp.lat, dpnm, expected);
      errs++;
   }
   //
   wp.lat = 30.0;
   dpnm = get_deg_per_nm(wp);
   expected = 0.0193432;
   if (fabs(dpnm - expected) > 0.0001) {
      fprintf(stderr, "Latitude %.3f has %.5f, expected %.5f\n",
            wp.lat, dpnm, expected);
      errs++;
   }
   //
   wp.lat = 60.0;
   dpnm = get_deg_per_nm(wp);
   expected = 0.0338462;
   if (fabs(dpnm - expected) > 0.0001) {
      fprintf(stderr, "Latitude %.3f has %.5f, expected %.5f\n",
            wp.lat, dpnm, expected);
      errs++;
   }
   /////////////////////////////////////////////////////////////
   if (errs == 0) {
      printf("    passed\n");
   } else {
      printf(" ** %d error(s)\n", errs);
   }
   return errs;
}

////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv)
{
   (void) argc;
   (void) argv;
   int32_t errs = 0;
   errs += test_convert_world_coordinate_type();
   errs += test_file_exists();
   errs += test_terminate_folder_path();
   errs += test_code_submap_depth();
   errs += test_convert_akn_to_map_coord();
   errs += test_initialize_60x60_map();
   errs += test_get_deg_per_nm();
   //////////////////
   printf("\n");
   if (errs == 0) {
      printf("--------------------\n");
      printf("--  Tests passed  --\n");
      printf("--------------------\n");
   } else {
      printf("**********************************\n");
      printf("**** ONE OR MORE TESTS FAILED ****\n");
      printf("**********************************\n");
      fprintf(stderr, "%s failed\n", argv[0]);
   }
   return (int) errs;
}

