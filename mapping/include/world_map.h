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
#if !defined(WORLD_MAP_H)
#define WORLD_MAP_H
#include <stdint.h>
#include <math.h>
#include "pin_types.h"

#if !defined(STR_LEN)
#define STR_LEN      256
#endif   // STR_LEN

// GEBCO data is stored in level-2 maps
extern const char GEBCO_FILES[8][128];
#define GEBCO_0_0_0     "gebco_2020_n90.0_s0.0_w-180.0_e-90.0.asc"
#define GEBCO_1_90_0    "gebco_2020_n90.0_s0.0_w-90.0_e0.0.asc"
#define GEBCO_2_180_0   "gebco_2020_n90.0_s0.0_w0.0_e90.0.asc"
#define GEBCO_3_270_0   "gebco_2020_n90.0_s0.0_w90.0_e180.0.asc"
#define GEBCO_4_0_90    "gebco_2020_n0.0_s-90.0_w-180.0_e-90.0.asc"
#define GEBCO_5_90_90   "gebco_2020_n0.0_s-90.0_w-90.0_e0.0.asc"
#define GEBCO_6_180_90  "gebco_2020_n0.0_s-90.0_w0.0_e90.0.asc"
#define GEBCO_7_270_90  "gebco_2020_n0.0_s-90.0_w90.0_e180.0.asc"

// declination file is stored in world map folder, which is set
//    during configuration
#define DECLINATION_FILE   "magnetic.txt"

#define BEACON_FILE        "beacons.bin"
#define BEACON_IDX_FILE    "beacons.idx"

// top-left coordinates of each section file
extern const uint32_t GEBCO_TOP[8];
extern const uint32_t GEBCO_LEFT[8];

// BUF_LEN is length of arrays used for path and file names
#if !defined(BUF_LEN)
#define BUF_LEN 256
#endif   // BUF_LEN

#define MAP_LEVEL2_SIZE    240
#define MAP_LEVEL3_SIZE    720u

// level 1 covers entire world
#define NUM_MAP_LEVEL1_SQUARES   (360 * 180)
// level 2+ are maps that cover 1x1 degree lat/lon
#define NUM_MAP_LEVEL2_SQUARES   (MAP_LEVEL2_SIZE * MAP_LEVEL2_SIZE)
#define NUM_MAP_LEVEL3_SQUARES   (MAP_LEVEL3_SIZE * MAP_LEVEL3_SIZE)

// depth threshold such that, if water somewhere in a world grid square
//    is shallower than this, submaps are created for that square
#define SUBMAP_DEPTH_THRESHOLD_METERS   (65)

#define MAP_LEVEL_1_FILE_NAME    "world.map1"
#define MAP_LEVEL_2_FILE_EXTENSION    "map2"
#define MAP_LEVEL_3_FILE_EXTENSION    "map3"

// make sure that dir name ends with '/'
#define LEVEL2_DIR_NAME    "15sec/"
#define LEVEL3_DIR_NAME    "5sec/"

// approx conversion between meters and degrees (latitude)
// consider the earth to be a sphere, as rough approximation
//    (accurate to %)
// there are 180 degrees latitude, so 10800 minutes = 10800nm.
// there are 6080 feet per nautical mile, so 6566400 feet.
// there are .3048 meters per foot, so 20014387 meters, thus
//    circumference is 40028774 meters
// this is close to wikipedia's values: 40,075km along equator and
//    40,007km over poles
#define DEG_TO_METER          (40007863.0 / 360.0)
#define METER_TO_DEG          (360.0 / 40007863.0)

////////////////////////////////////////////////////////////////////////

union map_coordinate {
   struct {
      uint16_t x;
      uint16_t y;
   };
   uint32_t all;
};
typedef union map_coordinate map_coordinate_type;


// specifies grid unit in level-1 map (i.e., which 60x60nm, or 1x1deg,
//    square)
// integral degree coordinates, with x on [0,360) and y on [0,180), with
//    date line intersecting north pole as origin 0,0
// map grid coords are alaska-north
union map_grid_num {
   struct {
      uint16_t akn_x;
      uint16_t akn_y;
   };
   uint32_t all;
};
typedef union map_grid_num map_grid_num_type;


// position within 1x1 deg map
// value is on [0,1), with 0 the top|left edge
union map_subgrid_pos {
   struct {
      double sub_x;
      double sub_y;
   };
};
typedef union map_subgrid_pos map_subgrid_pos_type;


////////////////////////////////////////////////
// level 1

// available resolution of level2 map
#define MAP_FLAG_LEVEL_2    0x01
#define MAP_FLAG_LEVEL_3    0x02
#define MAP_FLAG_LEVEL_4    0x04   // not used presently -- 1sec resolution

#define SUBMAP_AVAILABLE_MASK  0x0f

// GEBCO maps are not navigable but still provide useful information so
//    they're included. 15sec data is based on GEBCO. higher resolution
//    is provided in areas where navigable data is present

union map_level1_square {
   struct {
      // high and low are elevations observed in this grid square
      // depth is negative, land >=0
      // values in meters
      int16_t low;
      int16_t high;
      //
      uint16_t flags;
      uint16_t reserved;
   };
   uint64_t all;
};
typedef union map_level1_square map_level1_square_type;

struct map_level1 {
   map_level1_square_type grid[NUM_MAP_LEVEL1_SQUARES];
};
typedef struct map_level1 map_level1_type;


////////////////////////////////////////////////
// level-N maps
// these are structurally identical. a different type is used
//    for each level to prevent accidental mixing. different
//    types have different sizes, so size doesn't need to be
//    explicitly represented in data structure

// depth representation
// depth is stored as unsigned byte
// unknown depth marked as 0xffff (ie, 65535).  if depth unknown then
//    need to get value from higher level map
// to compress data, different depth ranges are used
//    0-100    depth in 1-meter increments
//    100-150  depth in 10-meter increments
//    150-254  depth in 100-meter increments
//    255      depth unknown
// reverse mapping
//    depth < 100             depth
//    100 <= depth < 600      100 + (depth-100) / 10
//    600 <= depth            150 + (depth-600) / 100
// max representable depth is 11km, which is approx the depth of the
//    marianas trench, which should be sufficient

// map grid elements

struct map_level2_square {
   uint8_t min_depth;   // depth reported for this grid square
};
typedef struct map_level2_square map_level2_square_type;

struct map_level3_square {
   uint8_t min_depth;   // depth reported for this grid square
};
typedef struct map_level3_square map_level3_square_type;

// maps

struct map_level2 {
   map_level2_square_type grid[NUM_MAP_LEVEL2_SQUARES];
};
typedef struct map_level2 map_level2_type;

struct map_level3 {
   map_level3_square_type grid[NUM_MAP_LEVEL3_SQUARES];
};
typedef struct map_level3 map_level3_type;

//
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
// API

////////////////////////////////////////////////
// utility functions
// returns 1 if file exists and 0 otherwise
int file_exists(
      /* in     */ const char *name
      );


// copies directory name to new buffer, appending a path separator
//    to that buffer it if wasn't arleady there
void terminate_folder_path(
      /* in     */ const char *dir,
      /* in out */       char *proper_name
      );

////////////////////////////////////////////////
// map reading

// loads level1 map. if 'map' is null, memory is allocated
// returns NULL on failure, pointer to map on success

// the following functions load maps of the specified levels
// 'map' is the memory that the map data should be loaded into. if 'map'
//    is NULL then memory is allocated
// returns the pointer to the map on success, and NULL on failure

map_level1_type * load_map_level1(
      /* in     */ const char *root_dir,
      /*    out */       map_level1_type *map
      );

map_level2_type * load_map_level2(
      /* in     */ const char *root_dir,
      /* in     */ const map_grid_num_type grid_pos,
      /*    out */       map_level2_type *map
      );

map_level3_type * load_map_level3(
      /* in     */ const char *root_dir,
      /* in     */ const map_grid_num_type grid_pos,
      /* in out */       map_level3_type *map
      );

// the following functions write the supplied maps to disk
// these hard-fail on error (ie, exit(1))

void write_map_level1(
      /* in     */ const char *root_dir,
      /* in     */ const map_level1_type *map
      );

void write_map_level2(
      /* in     */ const char *root_dir,
      /* in     */ const map_grid_num_type grid_pos,
//      /* in     */ const map_grid_num_type pos,
      /* in     */ const map_level2_type *map
      );

void write_map_level3(
      /* in     */ const char *root_dir,
      /* in     */ const map_grid_num_type grid_pos,
//      /* in     */ const map_grid_num_type pos,
      /* in     */ const map_level3_type *map
      );


//////////////

// reads declination (and inclination) for given position
// returns 0 on success, 1 on error. on success, declination holds
//    approximate magnetic deviation
int load_declination(
      /* in     */ const char *fname,
      /* in     */ const world_coordinate_type latlon,
      /*    out */       double *declination,
      /*    out */       double *inclination
      );


// builds 60x60nm map w/ data from all map levels
// constructed map is centered at latlon
map_level3_type * build_60x60_map(
      /* in     */ const char *root_dir,
      /* in     */ const world_coordinate_type latlon,
      /* in out */       map_level3_type *map
      );


// convert depth, in meters, to value stored in submaps, and convert
//    back again
uint8_t encode_submap_depth(
      /* in     */ const uint16_t meters
      );

uint16_t decode_submap_depth(
      /* in     */ const uint8_t val
      );

// convert lat-lon to akn-north representation
//    (ie, move origin from greenwich equator to date-line north pole)
akn_position_type convert_latlon_to_akn(
      /* in     */ const world_coordinate_type latlon
      );


// converts akn position (ie, lat-lon based on dateline/north pole origin)
//    to map grid
// if sub is not NULL, relative position in grid square is returned,
//    with 0,0 being top-left of square and 1,1 bottom right
map_grid_num_type convert_akn_to_grid(
      /* in     */ const akn_position_type akn_pos,
      /*    out */       map_subgrid_pos_type *sub
      );


// converts akn position to world coordinate (ie, standard lat,lon)
world_coordinate_type convert_akn_to_world(
      /* in     */ const akn_position_type akn_pos
      );


// computes row and column for position with map located at map_center
// if coordinate is w/in map, 0 is returned, otherwise if outside of
//    map area, -1 is returned
// it is assumed/required that map center is more than 0.5 degs away
//    from north pole
int32_t convert_akn_to_map_coord(
      /* in     */ const akn_position_type pos,
      /* in     */ const akn_position_type map_center,
      /*    out */       map_coordinate_type *map_pos
      );

#endif   // WORLD_MAP_H
