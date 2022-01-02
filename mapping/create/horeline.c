#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <math.h>
#include <stdlib.h>
#include <zlib.h>

#include "world_map.h"

// update (and create as necessary) 5-second level-3 files with
//    shoreline data. data input should be a text file of shoreline
//    segments as generated in shoreline/
// this should be run after level-3 maps are already generated, although
//    as it's built off the code for read_noaa, it _should_ be OK to
//    run this independently

// much of the logic has been moved to a separate C file so that it
//    can be shared by noaa-chart and shoreline-processing code
#include "noaa_common.c"

// increment size on x and y axes. this is constant regardless of latitude
//    so horizontal segments will be tiny at high latitudes, but that's
//    just a performance issue and not worth optimizing to fix
// step size should be <5m at equator, as this will keep steps <7m for
//    diagonal (45deg) segments
// use much smaller step, to minimize areas of coast where 8-connected 
//    line can cross border
#define SEGMENT_SIZE_DEG   (0.75 * METER_TO_DEG)

////////////////////////////////////////////////////////////////////////
// 

static void push_shoreline_to_square(
      /* in     */ const map_grid_num_type grid_pos,
      /* in     */ const map_subgrid_pos_type sub_pos
      )
{
   // depth 
   map_level1_square_type *square1 = get_square1(grid_pos);
   // update map1 if necessary
   int16_t elev = 0;
   if (square1->high < elev) {
      square1->high = elev;
      level1_modified_ = 1;
   } else if (square1->low > elev) {
      square1->low = elev;
      level1_modified_ = 1;
   }
   if ((square1->flags & SUBMAP_AVAILABLE_MASK) != 0) {
      // check submap
      //////////////////////////////////////////////////////////////////
      uint8_t code = 0;
      map_level3_square_type *square3 = get_square3(grid_pos, sub_pos);
      if ((square3->min_depth == 255) || (code < square3->min_depth)) {
         square3->min_depth = code;
         // level3 is changed -- check to see if level 2 requires update
         map_level2_square_type *square2 = get_square2(grid_pos, sub_pos);
         if (square2->min_depth < code) {
            square2->min_depth = code;
            uint32_t world_idx = 
                  (uint32_t) (grid_pos.akn_x + grid_pos.akn_y * 360);
            level2_modified_[world_idx] = 1;
         }
      }
   }
}

static void draw_line(
      /* in     */ const double x0,
      /* in     */ const double y0,
      /* in     */ const double x1,
      /* in     */ const double y1
      )
{
   // break each segment into little pieces and iterate forward
   // ever time we enter a new level-3 grid square, push depth to that
   //    square
   double dx = x1 - x0;
   double dy = y1 - y0;
   double len = sqrt(dx*dx + dy*dy);
   double num_steps = len / SEGMENT_SIZE_DEG;
//printf("Segment from %.4f,%.4f to %.4f,%.4f, %d steps\n", x0, y0, x1, y1, (uint32_t) num_steps);
//printf("Len: %f deg   %f met\n", len, len * DEG_TO_METER);
//printf("Seg size: %f\n", SEGMENT_SIZE_DEG);
   dx /= num_steps;
   dy /= num_steps;
   // keep a double-precision running lat-lon for coordinate
   double pos_x = x0;
   double pos_y = y0;
   // last-updated level-3 grid square
   uint32_t last_idx3_x = 0;
   uint32_t last_idx3_y = 0;
   uint32_t idx3_x, idx3_y;
   for (uint32_t i=0; i<(uint32_t) num_steps; i++) {
      if (pos_x > 180.0) {
         pos_x -= 360.0;
      } else if (pos_x <= -180) {
         pos_x += 360.0;
      }
      world_coordinate_type latlon = 
            { .lon = (float) pos_x, .lat = (float) pos_y };
      akn_position_type akn = convert_latlon_to_akn(latlon);
      map_subgrid_pos_type sub_pos;
      map_grid_num_type grid_pos = convert_akn_to_grid(akn, &sub_pos);
      idx3_x = (uint32_t) (sub_pos.sub_x * (double) MAP_LEVEL3_SIZE);
      idx3_y = (uint32_t) (sub_pos.sub_y * (double) MAP_LEVEL3_SIZE);
      if ((last_idx3_x != idx3_x) || (last_idx3_y != idx3_y)) {
         // push depth to square
//printf("  shore at %d,%d   %d,%d  (%d)\n", pos.map1_x, pos.map1_y, pos.idx3_x, pos.idx3_y, i);
         push_shoreline_to_square(grid_pos, sub_pos);
         // remember position we just updated
         last_idx3_x = idx3_x;
         last_idx3_y = idx3_y;
      }
      // update lat-lon for next sub-segment
      pos_x += dx;
      pos_y += dy;
   }
}


static int parse_position(
      /* in     */       char *line,
      /*    out */       double *x,
      /*    out */       double *y
      )
{
   char *end_ptr;
   if (line[0] == '#') {
      return -1;
   }
   errno = 0;
   *x = strtof(line, &end_ptr);
   if (errno != 0) {
      fprintf(stderr, "Parse error in '%s'\n", line);
      exit(1);
   }
   *y = strtof(end_ptr, NULL);
   if (errno != 0) {
      fprintf(stderr, "Parse error in '%s'\n", line);
      exit(1);
   }
   return 0;
}

// 
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
//


int main(int argc, char **argv)
{
   printf("Update 5sec level-3 maps based on shoreline data\n");
   int rc = -1;
   char *vector = NULL;
   FILE *vec_fp = NULL;  // for vector file
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
      printf("Usage: %s <map dir> <vector-file>\n", argv[0]);
      printf("\nwhere 'vector-file' is file containing shoreline vector "
            "sectments\n");
      goto end;
   }
   terminate_folder_path(argv[1], map_dir_);
   vector = argv[2];
   //////////////////////////////////////
   printf("Updating level3 at '%s' using '%s'\n", map_dir_, vector);
   //
   if ((level1_map_ = load_map_level1(map_dir_, NULL)) == NULL) {
      fprintf(stderr, "Bailing out\n");
      goto end;
   }
   vec_fp = fopen(vector, "r");
   if (!vec_fp) {
      fprintf(stderr, "Unable to open vector file '%s'\n", vector);
      goto end;
   }
   char buf[BUF_LEN];
   // data in file is stored as points, with successive points forming a
   //    segment
   double x0, x1, y0, y1;  // segment coordinates
   x0 = 0.0;
   y0 = 0.0;
   // flag to indicate whether first point in segment is loaded or not
   int in_segment = 0;  
   while (fgets(buf, sizeof(buf), vec_fp) != NULL) {
      if (in_segment == 0) {
         // read start point
         parse_position(buf, &x0, &y0);
         // load draw-to point
         if (fgets(buf, sizeof(buf), vec_fp) == NULL) {
            goto end;
         }
         in_segment = 1;
      }
      // read draw-to point
      if (parse_position(buf, &x1, &y1) != 0) {
         // end-of-segment
         in_segment = 0;
         continue;
      }
      draw_line(x0, y0, x1, y1);
      x0 = x1;
      y0 = y1;
      check_resources(0);
   }
end:
   // save everything
   check_resources(1);
   for (int x=0; x<360; x++) {
      for (int y=0; y<180; y++) {
         int i = x + 360 * y;
         if (no_submap_[i] != 0) {
            fprintf(stderr, "Level3 depth shallow but no submap present. "
                  "%d,%d   depth %d\n", x, y, no_submap_[i]);
         }
      }
   }
   if (vec_fp != NULL) {
      fclose(vec_fp);
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


