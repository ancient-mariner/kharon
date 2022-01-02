#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>
#include <sys/stat.h>

#include "world_map.h"

#if !defined R2D
#define R2D    (180.0 / M_PI)
#define D2R    (M_PI / 180.0)
#endif   // R2D

const char GEBCO_FILES[8][128] = {
   GEBCO_0_0_0,
   GEBCO_1_90_0,
   GEBCO_2_180_0,
   GEBCO_3_270_0,
   GEBCO_4_0_90,
   GEBCO_5_90_90,
   GEBCO_6_180_90,
   GEBCO_7_270_90
};

// top-left coordinates of each section file
const uint32_t GEBCO_TOP[8] = { 0, 0, 0, 0, 90, 90, 90, 90 };
const uint32_t GEBCO_LEFT[8] = { 0, 90, 180, 270, 0, 90, 180, 270 };

// use preallocated buffers to load raw map data and copy out of it
static __thread map_level1_type tmp_map_1_;
static __thread map_level2_type tmp_map_2_;
static __thread map_level3_type tmp_map_3_;

static __thread map_level1_square_type tmp_square_1_;

////////////////////////////////////////////////////////////////////////

// converts akn position (ie, lat-lon based on dateline/north pole origin)
//    to map grid
map_grid_num_type convert_akn_to_grid(
      /* in     */ const akn_position_type akn_pos,
      /*    out */       map_subgrid_pos_type *sub
      )
{
   double akn_x_floor = floor(akn_pos.akn_x);
   double akn_y_floor = floor(akn_pos.akn_y);
   assert(akn_x_floor >= 0.0);
   assert(akn_x_floor < 360.0);
   assert(akn_y_floor >= 0.0);
   assert(akn_y_floor <= 180.0);
   //
   map_grid_num_type grid_pos;
   grid_pos.akn_x = (uint16_t) akn_x_floor;
   grid_pos.akn_y = (uint16_t) akn_y_floor;
   if (sub != NULL) {
      sub->sub_x = akn_pos.akn_x - akn_x_floor;
      sub->sub_y = akn_pos.akn_y - akn_y_floor;
   }
   return grid_pos;
}

world_coordinate_type convert_akn_to_world(
      /* in     */ const akn_position_type akn_pos
      )
{
   world_coordinate_type world;
   world.lon = akn_pos.akn_x - 180.0;
   if (world.lon < 0.0) {
      world.lon += 360.0;
   }
   world.lat = -(akn_pos.akn_y - 90.0);
   return world;
}


////////////////////////////////////////////////////////////////////////

// returns 1 if file exists and 0 otherwise
int file_exists(
      /* in     */ const char *name
      )
{
   struct stat buf;
   return (stat(name, &buf) == 0);
}


// copies directory name to new buffer, appending a path separator
//    to that buffer it if wasn't arleady there
void terminate_folder_path(
      /* in     */ const char *dir,
      /* in out */       char *proper_name
      )
{
   if ((dir == NULL) || (dir[0] == 0)) {
      fprintf(stderr, "Directory name is empty. Bailing out\n");
      exit(1);
   }
   size_t root_len = strlen(dir);
   // make sure length is smaller than max streng length, and that
   //    we have room to add a '/' to the end if necessary
   if (root_len >= BUF_LEN-2) {
      fprintf(stderr, "Buffer overflow detected (path name too long). "
            "Bailing out (recompile or shorten name)\n");
      fprintf(stderr, "Provided name: '%s'\n", dir);
      exit(1);
   }
   strcpy(proper_name, dir);
   if (dir[root_len-1] != '/') {
      proper_name[root_len] = '/';
      proper_name[root_len+1] = 0;
   }
}

////////////////////////////////////////////////////////////////////////
// load maps

map_level1_type * load_map_level1(
      /* in     */ const char *root_dir,
      /*    out */       map_level1_type *map
      )
{
//printf("Root dir: '%s'\n", root_dir);
   // make a local copy of the supplied map pointer
   map_level1_type *local_map = map;
   char map_name[BUF_LEN];
   sprintf(map_name, "%s%s", root_dir, MAP_LEVEL_1_FILE_NAME);
   FILE *map_fp = fopen(map_name, "r");
   if (map_fp == NULL) {
      fprintf(stderr, "Failed to open map file '%s'\n", map_name);
      local_map = NULL;
      goto end;
   }
   //printf("Reading level-1 map '%s'\n", map_name);
   if (local_map == NULL) {
      local_map = &tmp_map_1_;
   }
   fread(local_map, sizeof *local_map, 1, map_fp);
end:
   if (map_fp != NULL) {
      fclose(map_fp);
   }
   return local_map;
}

map_level2_type *load_map_level2(
      /* in     */ const char *root_dir,
      /* in     */ const map_grid_num_type grid_pos,
      /*    out */       map_level2_type *map
      )
{
   // make a local copy of the supplied pointer
   map_level2_type *tmp_map = map;
   char map_name[BUF_LEN];
   sprintf(map_name, "%s%s%d/%d_%d.%s", root_dir,
         LEVEL2_DIR_NAME, 10 * ((int) grid_pos.akn_y/10), 
         grid_pos.akn_x, grid_pos.akn_y, MAP_LEVEL_2_FILE_EXTENSION);
   FILE *fp = fopen(map_name, "r");
   if (fp == NULL) {
      fprintf(stderr, "Failed to open map file '%s'\n", map_name);
      tmp_map = NULL;
      goto end;
   }
//printf("Reading level-2 map '%s'\n", map_name);
   if (tmp_map == NULL) {
      tmp_map = &tmp_map_2_;
   }
   fread(&tmp_map->grid, sizeof *tmp_map, 1, fp);
end:
   if (fp != NULL) {
      fclose(fp);
   }
   return tmp_map;
}

// load level3 map. if 'in_map' is NULL then static memory is used to
//    store the map
//    NOTE: do not try to free this
// returns pointer to object on success, NULL on failure
map_level3_type *load_map_level3(
      /* in     */ const char *root_dir,
      /* in     */ const map_grid_num_type grid_pos,
      /*    out */       map_level3_type *map
      )
{
   // make a local copy of the supplied map pointer
   map_level3_type *tmp_map = map;
   char map_name[BUF_LEN];
   sprintf(map_name, "%s%s%d/%d_%d.%s", root_dir,
         LEVEL3_DIR_NAME, 10 * ((int) grid_pos.akn_y/10), grid_pos.akn_x, 
         grid_pos.akn_y, MAP_LEVEL_3_FILE_EXTENSION);
   FILE *fp = fopen(map_name, "r");
   if (fp == NULL) {
      fprintf(stderr, "Failed to open map file '%s'\n", map_name);
      tmp_map = NULL;
      goto end;
   }
//printf("Reading level-3 tmp_map '%s'\n", map_name);
   if (tmp_map == NULL) {
      tmp_map = &tmp_map_3_;
   }
   fread(&tmp_map->grid, sizeof *tmp_map, 1, fp);
end:
   if (fp != NULL) {
      fclose(fp);
   }
   return tmp_map;
}

// load maps
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
// write maps

void write_map_level1(
      /* in     */ const char *root_dir,
      /* in     */ const map_level1_type *map
      )
{
   char map_name[BUF_LEN];
   sprintf(map_name, "%s%s", root_dir, MAP_LEVEL_1_FILE_NAME);
   FILE *fp = fopen(map_name, "w");
   if (fp == NULL) {
      fprintf(stderr, "Failed to open map file '%s'\n", map_name);
      exit(1);
   }
   fwrite(map, sizeof *map, 1, fp);
   if (fp != NULL) {
      fclose(fp);
   }
}

void write_map_level2(
      /* in     */ const char *root_dir,
      /* in     */ const map_grid_num_type grid_pos,
      /* in     */ const map_level2_type *map
      )
{
   char map_name[BUF_LEN];
   sprintf(map_name, "%s%s%d/%d_%d.%s", root_dir,
         LEVEL2_DIR_NAME, 10 * ((int) grid_pos.akn_y/10), 
         grid_pos.akn_x, grid_pos.akn_y, MAP_LEVEL_2_FILE_EXTENSION);
   FILE *fp = fopen(map_name, "w");
   if (fp == NULL) {
      fprintf(stderr, "Failed to open map file '%s'\n", map_name);
      exit(1);
   }
   fwrite(map, sizeof *map, 1, fp);
   if (fp != NULL) {
      fclose(fp);
   }
}

void write_map_level3(
      /* in     */ const char *root_dir,
      /* in     */ const map_grid_num_type grid_pos,
      /* in     */ const map_level3_type *map
      )
{
   char map_name[BUF_LEN];
   sprintf(map_name, "%s%s%d/%d_%d.%s", root_dir,
         LEVEL3_DIR_NAME, 10 * ((int) grid_pos.akn_y/10), 
         grid_pos.akn_x, grid_pos.akn_y, MAP_LEVEL_3_FILE_EXTENSION);
   FILE *fp = fopen(map_name, "w");
   if (fp == NULL) {
      fprintf(stderr, "Failed to open map file '%s'\n", map_name);
      exit(1);
   }
   fwrite(map, sizeof *map, 1, fp);
   if (fp != NULL) {
      fclose(fp);
   }
}


// write maps
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
// utility

// convert depth, in meters, to value stored in level2 file, and convert
//    back again
// convert depth to value on [0,254]. note that meters is positive for
//    areas where there's non-zero depth. this distinction is important
//    because other areas use meters as elevation
uint8_t encode_submap_depth(
      /* in     */ const uint16_t meters
      )
{
   int val;
   if (meters < 100) {
      val = meters;
   } else if (meters < 600) {
      val = 100 + (meters - 100) / 10;
   } else {
      val = 150 + (meters - 600) / 100;
      if (val >= 255) {
         val = 254;
      }
   }
   return (uint8_t) val;
}

// convert value on [0,254] to depth, in meters
uint16_t decode_submap_depth(
      /* in     */ const uint8_t val
      )
{
   uint16_t depth;
   if (val < 100) {
      depth = val;
   } else if (val < 150) {
      depth = (uint16_t) (100 + (val - 100) * 10);
   } else if (val < 255) {
      depth = (uint16_t) (600 + (val - 150) * 100);
   } else {
      depth = 0xffff;   // unknown
   }
   return depth;
}


// convert lat-lon to akn-north representation
//    (ie, move origin from greenwich equator to date-line north pole)
akn_position_type convert_latlon_to_akn(
      /* in     */ const world_coordinate_type latlon
      )
{
   double akn_y = 90.0 - latlon.lat;
   double akn_x = 180.0 + latlon.lon;
   if (akn_x >= 360.0) {
      akn_x -= 360.0;
   }
   assert(akn_x >=  0.0);
   assert(akn_x < 360.0);
   assert(akn_y >=  0.0);
   assert(akn_y < 180.0);
   const akn_position_type pos = { .akn_x = akn_x, .akn_y = akn_y };
   return pos;
}


// computes map column for position with map located at map_center
// if position is out of bounds (off map) then returns -1, otherwise 
//    returns 0
// only requires that 'x' value be set in pos and map_center, as scale
//    explicitly provided
static int32_t convert_akn_to_map_column(
      /* in     */ const akn_position_type pos,
      /* in     */ const akn_position_type map_center,
      /* in     */ const double scale,
      /*    out */       uint16_t *out_col
      )
{
   // recheck for wrap as AKN value could have been adjusted across
   //    date line
   double akn_x = pos.akn_x;
   if (akn_x < 0.0) {
      akn_x += 360.0;
   } else if (akn_x >= 360.0) {
      akn_x -= 360.0;
   }
   assert(akn_x >= 0.0);
   assert(akn_x < 360.0);
   assert(map_center.akn_x >= 0.0);
   assert(map_center.akn_x < 360.0);
   //
   double dx_deg = akn_x - map_center.akn_x;
   if (fabs(dx_deg) > 180.0) {
      // wraps around the pole. unwrap
      if (dx_deg < 0.0) {
         dx_deg += 360.0;
      } else {
         dx_deg -= 360.0;
      }
   }
   double dx_nm = scale * dx_deg * 60.0;
   if (fabs(dx_nm) >= 30.0) {
      goto invalid_col;
   }
   // distance from left edge
   double dx_left_nm = dx_nm + 30.0;
   assert(dx_left_nm >= 0.0);
   // convert from miles to columns
   double dx_left_col = dx_left_nm * (double) (MAP_LEVEL3_SIZE / 60);
   *out_col = (uint16_t) round(dx_left_col);
   if (*out_col >= MAP_LEVEL3_SIZE) {
      goto invalid_col;
   }
   //
   return 0;
invalid_col:
   return -1;
}


// computes map row for position with map located at map_center
// if position is out of bounds (off map) then returns -1, otherwise 
//    returns 0
// only requires that 'y' value be set in pos and map_center
static int32_t convert_akn_to_map_row(
      /* in     */ const akn_position_type pos,
      /* in     */ const akn_position_type map_center,
      /*    out */       uint16_t *out_row
      )
{
   assert(pos.akn_y > 1.0);
   assert(pos.akn_y < 179.0);
   assert(map_center.akn_y > 1.0);
   assert(map_center.akn_y < 179.0);
   //
   double dy_deg = pos.akn_y - map_center.akn_y;
   double dy_nm = dy_deg * 60.0;
   if (fabs(dy_nm) >= 30.0) {
      goto out_of_bounds;
   }
   // distance from top edge
   double dy_top_nm = dy_nm + 30.0;
   assert(dy_top_nm >= 0.0);
   double dy_top_row = dy_top_nm * (double) (MAP_LEVEL3_SIZE / 60);
   *out_row = (uint16_t) round(dy_top_row);
   if (*out_row >= MAP_LEVEL3_SIZE) {
      goto out_of_bounds;
   }
//printf("lat %.6f -> row %.6f -> %d\n", pos.akn_y, dy_top_row, *out_row);
   return 0;
out_of_bounds:
   return -1;
}


// computes row and column for position with map located at map_center
// if coordinate is w/in map, 0 is returned, otherwise if outside of 
//    map area, -1 is returned
// it is assumed/required that map center is more than 0.5 degs away
//    from north pole
int32_t convert_akn_to_map_coord(
      /* in     */ const akn_position_type pos,
      /* in     */ const akn_position_type map_center,
      /*    out */       map_coordinate_type *map_pos
      )
{
   if (convert_akn_to_map_row(pos, map_center, &map_pos->y) < 0) {
      goto out_of_bounds;
   }
   // get longitude scale for this position and get column
   double scale = cos(D2R * (pos.akn_y - 90.0));
   if (convert_akn_to_map_column(pos, map_center, scale, &map_pos->x) < 0) {
      goto out_of_bounds;
   }
   return 0;
out_of_bounds:
   return -1;
}


// utility
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
// build composite map from cached submaps

// find left and right bounds for map view
// loop through upper maps, project each extracted map position (1x1 deg)
//    to output world map (60x60nm), then repeat w/ lower maps


// read maps at this position into static storage buffers
// returns 0 on success, 1 on failure
static int32_t load_map_buffers(
      /* in     */ const char *root_dir,
      /* in     */ const map_grid_num_type grid_pos
      )
{
   if (load_map_level1(root_dir, NULL) == NULL) {
//printf("NO LEVEL 1 MAP\n");
      goto fail;
   }
   uint32_t world_idx = (uint32_t) (grid_pos.akn_x + grid_pos.akn_y * 360);
   tmp_square_1_ = tmp_map_1_.grid[world_idx];
//printf("MAP1 %d,%d   high %d low %d\n", grid_pos.akn_x, grid_pos.akn_y, tmp_square_1_.high, tmp_square_1_.low);
   // check for submaps
   if (tmp_square_1_.flags & MAP_FLAG_LEVEL_2) {
      if (load_map_level2(root_dir, grid_pos, NULL) == NULL) {
//printf("LEVEL 2 fail\n");
         goto fail;
      }
      if (tmp_square_1_.flags & MAP_FLAG_LEVEL_3) {
         if (load_map_level3(root_dir, grid_pos, NULL) == NULL) {
//printf("LEVEL 3 fail\n");
            goto fail;
         }
      }
   }
   return 0;
fail:
   return -1;
}


// get depth data at poxition x,y from static map buffers
static uint8_t get_depth_from_buffers(
      /* in     */ const uint32_t x,
      /* in     */ const uint32_t y
      )
{
   uint8_t depth = 0;
   assert(x < 720);
   assert(y < 720);
   // if there's submap data, use it
   if (tmp_square_1_.flags & MAP_FLAG_LEVEL_2) {
      int32_t idx2 = ((int32_t) x)/3 + ((int32_t) y/3) * 240;
      // there's always level2 depth if there's a submap
      depth = tmp_map_2_.grid[idx2].min_depth;
      if (tmp_square_1_.flags & MAP_FLAG_LEVEL_3) {
         uint32_t idx3 = x + y * 720;
         uint8_t depth3 = tmp_map_3_.grid[idx3].min_depth;
         // if depth known for level 3, use it
         if (depth3 != 255) {
            depth = depth3;
         }
      }
   } else {
      // no submap -- use highest value reported in level1 map
      if (tmp_square_1_.high < 0) {
         depth = encode_submap_depth((uint16_t) (-tmp_square_1_.high));
//printf("encoded depth %d (high %d  low %d)\n", depth, tmp_square_1_.high, tmp_square_1_.low);
      }
   }
   return depth;
}

// returns 1 if map is initialized and complete (i.e., if near pole)
//    and 0 otherwise
static int32_t initialize_60x60_map(
      /* in     */ const world_coordinate_type latlon,
      /*    out */       map_level3_type *map
      )
{
   // initialize storage
   assert(map != NULL);
   memset(map, 0, sizeof *map);
   // if map is near either pole, set fixed depth value and call it good
   if (latlon.lat > 87.0) {
      // arctic ocean
      // depth at pole is quite deep (>3000m) but Lomonosov Ridge is w/in
      //    one degree of it, and that raises to ~1000m depth. depth is
      //    largely irrelevant for this use case if it's over 250m, so
      //    assigning everything around the pole a constant value and not
      //    worrying about underwater ridges and basins should be fine.
      memset(map, 155, sizeof *map);
      goto all_done;
   } else if (latlon.lat < -84.0) {
      // antarctica. this area is land and/or ice
      memset(map, 0, sizeof *map);
      goto all_done;
   }
   // set all map points to 'unknown' depth
   memset(map, 255, sizeof *map);
   return 0;
all_done:
   return 1;
}


// returns max number of degees per nautical mile for map centered at 
//    this latitude
static double get_deg_per_nm(
      /* in     */ const world_coordinate_type latlon
      )
{
   // get output map left and right bounds, in longitude
   // if in northern hemisphere, add 0.5 degs to center and get left/right,
   //    else subtract 0.5
   double far_edge;
   if (latlon.lat < 0.0) {
      far_edge = fabs(latlon.lat - 0.5);
   } else {
      far_edge = fabs(latlon.lat + 0.5);
   }
   // number of degees per nautical mile at this latitude
   //    degrees at this latitude (=360) / NMs circumference (=360*60*cos(lat))
   //       360 / (360*60*cos(lat))
   //         1 / (60 * cos(lat))
   double deg_per_nm = 1.0 / (60.0 * cos(D2R * far_edge));
   return deg_per_nm;
}


// builds 60x60nm map w/ data from all map levels, centered at latlon
// data stored in 'map' (which cannot be NULL). if an error occurs during
//    loading, return value is NULL, otherwise it's 'map'
map_level3_type * build_60x60_map(
      /* in     */ const char *root_dir,
      /* in     */ const world_coordinate_type latlon,
      /*    out */       map_level3_type *map
      )
{
   // initialize storage
   if (initialize_60x60_map(latlon, map) == 1) {
      goto end;
   }
   /////////////////////////////////////////////////////////////////////
   // get output map left and right bounds, in longitude
   double deg_per_nm = get_deg_per_nm(latlon);
//printf("deg/nm: %.3f\n", deg_per_nm);
   // number of degrees offset left/right at this latitude for 30nm
   double half_width_deg = deg_per_nm * 30.0;
//printf("deg half-width %.3f\n", half_width_deg);
   // get lateral bounds of map to pull from. convert bounds to AK-north
   const akn_position_type center = convert_latlon_to_akn(latlon);
   const double left_akn_deg = floor(center.akn_lon - half_width_deg);
   const double right_akn_deg = ceil(center.akn_lon + half_width_deg);
   const double top_akn_deg = floor(center.akn_lat - 0.5);
//printf("BOUNDS LR %.4f,%.4f    T %.4f\n", left_akn_deg, right_akn_deg, top_akn_deg);
   const int32_t left_akn_deg_int = (int32_t) left_akn_deg;
   const int32_t right_akn_deg_int = (int32_t) right_akn_deg;
   const int32_t top_akn_deg_int = (int32_t) top_akn_deg;
   // sanity check -- just make sure values aren't insane
   assert(left_akn_deg_int > -360);
   assert(left_akn_deg_int < 360);
   assert(right_akn_deg_int > -180);
   assert(right_akn_deg_int < 720);
   assert(top_akn_deg_int >= 0);
   assert(top_akn_deg_int < 180);
   // iterate through map grids to get data to create 60x60 view
   // map iteration uses Alaska-north coordinate system, as grids
   //    are based on those. iteration w/in grid starts at top-left, which
   //    is consistent with akn system, so within-grid offsets can be added
   //    to grid's left/top bounds
   for (int32_t map_y=top_akn_deg_int; map_y<=top_akn_deg_int+1; map_y++) {
//printf("MAP Y %d\n", map_y);
      map_grid_num_type pos_akn;
      pos_akn.akn_y = (uint16_t) map_y;
      for (int32_t map_x=left_akn_deg_int; map_x<=right_akn_deg_int; map_x++) {
//printf("  MAP X %d\n", map_x);
         if (map_x < 0) {
            pos_akn.akn_x = (uint16_t) (map_x + 360);
         } else if (map_x >= 360) {
            pos_akn.akn_x = (uint16_t) (map_x - 360);
         } else {
            pos_akn.akn_x = (uint16_t) map_x;
         }
         if (load_map_buffers(root_dir, pos_akn) != 0) {
            fprintf(stderr, "Failed to load map buffers\n");
            // TODO handle error better -- this is an internal error
            //    and ought to be fatal
            assert(1 == 0);
         }
         // copy content from input map buffers to output map
         for (uint32_t y=0; y<720; y++) {
            // get latitude -- this is the top of the input map grid 
            //    plus 1/720 for each row, as 1-deg grid is 720 rows high
            //    and akn_deg value is increasing going down
            akn_position_type in_akn_deg;
            in_akn_deg.akn_y = (double) map_y + (double) y / 720.0;
            // offset from this akn-lat to input grid top
            double dy_deg = in_akn_deg.akn_y - center.akn_lat;
            if (fabs(dy_deg) >= 0.5) {
               // row is outside of output map (output map is 1.0 deg tall)
               if (dy_deg >= 0.5) {
                  // below the bottom -- nothing left to do
                  break;
               }
               continue;
            }
            // get output map row
            map_coordinate_type out_map_pos;
            if (convert_akn_to_map_row(in_akn_deg, center, 
                  &out_map_pos.y) != 0) {
               continue;
            }
            assert(out_map_pos.y < 720);  // y is unsigned, so no neg check
            // get adjustment scale for converting lon degs to lat degs
            //    at this latitude
            double scale = cos(D2R * (in_akn_deg.akn_y - 90.0));
//printf(" in_akn_y_deg %.6f  out_y_row %d\n", in_akn_deg.akn_y, out_map_pos.y);
            //
            for (uint32_t x=0; x<720; x++) {
               in_akn_deg.akn_x = (double) map_x + (double) x / 720.0;
//printf("  in %.3f,%.3f   left %.3f   x %d\n", in_akn_deg.akn_x, in_akn_deg.akn_y, left_akn_deg, x);
               // measure horizontal offet from input grid square to present
               //    location
               // left_akn_deg and in_x_akn_deg are in AKN coords
               double dx_deg = in_akn_deg.akn_x - center.akn_lon;
               // convert deg offset to NMs by converting to equivalent lat
               //    and *60 from lat degs to NM
               double dx_nm = scale * dx_deg * 60.0;
//printf("    dx_deg %.4f    dx_nm %.6f\n", dx_deg, dx_nm);
               if (fabs(dx_nm) < 30.0) {
                  // position is in output map. push depth value to map
                  if (convert_akn_to_map_column(in_akn_deg, center, 
                        scale, &out_map_pos.x) != 0) {
//printf("      out of bounds  in_deg %.4f  ctr_deg %.4f\n", in_akn_deg.akn_lon, center.akn_lon);
                     continue;
                  }
                  assert(out_map_pos.x < 720);
//printf("      pos %d\n", out_map_pos.x);
                  uint32_t out_idx = 
                        (uint32_t) (out_map_pos.x + out_map_pos.y * 720);
                  // if output map point is unknown, set it to depth from this
                  //    input map point. otherwise, select higher value from
                  //    both to store in output map
                  uint8_t depth = get_depth_from_buffers(x, y);
                  uint8_t existing_depth = map->grid[out_idx].min_depth;
//printf("    out %d,%d (%d)  exist %d   depth %d\n", out_map_pos.x, out_map_pos.y, out_idx, existing_depth, depth);


                  // if existing depth unknown, or this is shallower
                  //    than previous min depth, update depth
                  if ((existing_depth == 255) || (depth < existing_depth)) {
                     map->grid[out_idx].min_depth = depth;
                  }
               } else if (dx_nm >= 30.0) {
                  // to right of output map. nothing more to do this row
//printf("    dx_nm = %.3f\n", dx_nm);
                  break;
               }
            }
         }
      }
   }
   /////////////////////////////////////////////////////////////////////
end:
   return map;
}

