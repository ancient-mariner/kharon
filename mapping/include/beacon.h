#if !defined(BEACON_H)
#define BEACON_H
#include "world_map.h"

// don't create beacons in areas where the magnetic inclination is
//    too vertical and magnetic compass is not reliable
#define BEACON_INCLINATION_LIMIT    84.0

// offset (from north pole) to use when generating default beacons,
//    and the step size to the next beacon row
#define DEFAULT_LAT_OFFSET_DEG   (1.0 / 8.0)
#define DEFAULT_LAT_STEP_DEG     (1.0 / 4.0)

// for AKN_Y below this latitude, don't bother making beacons, because
//    positions at that latitude correspond to Antarctica landmass
// the southernmost shoreline is ~175 akn-degs, but that's covered by Ross
//    ice sheet. put a southern limit further north
#define DEFAULT_LAT_SOUTHERN_BOUNDS_AKN    (170.0)

// for each beacon row, horizontal steps will be the largest interval
//    that evenly divides row and is less than LON_STEP
#define DEFAULT_LON_STEP_MET     ((19.0 / 60.0) * DEG_TO_METER)


////////////////////////////////////////////////////////////////////////
// beacon index and bin files

// bin file holds 180 rows, one for each degree latitude, with each row
//    having an unordered list of 0 or more beacon positions. along with
//    each position is a list of other neighboring beacons and the distance
//    to them
// each beacon entry in the bin file is of constant size
// a formatted struct is the better way to read the binary data in, but
//    as it's output by a different source (ie, python), read it in as
//    an array, as all values are 32-bit. there shouldn't

// max number of beacons that a beacon will store as its neighbors
#define MAX_BEACON_NEIGHBORS       8

// max number of beacons represented in path map
#define MAX_PATH_MAP_BEACONS        12

// IMPORTANT the format for beacon_distance and beacon_record must be
//    matched by python script that generates beacon.bin

// distance to neighboring beacon
union beacon_neighbor {
   struct {
      uint32_t nbr_index;
      float path_weight;
   };
   uint64_t all;
};
typedef union beacon_neighbor beacon_neighbor_type;

//// distance of beacon from point in space (e.g., map center)
//union beacon_distance_from {
//   struct {
//      uint32_t beacon_index;
//      float distance_met;
//   };
//   uint64_t all;
//};
//typedef union beacon_distance_from beacon_distance_from_type;


// data structure that's stored in beacons.bin
struct beacon_record {
   float akn_x;
   float akn_y;
   int32_t num_neighbors; // -1 if uninitialized/invalid
//   int32_t num_records; // -1 if uninitialized/invalid
   uint32_t index;         // index of this record
   beacon_neighbor_type neighbors[MAX_BEACON_NEIGHBORS];
};
typedef struct beacon_record beacon_record_type;
#define BEACON_BIN_RECORD_SIZE_BYTES      (80)
_Static_assert(sizeof(beacon_record_type) == BEACON_BIN_RECORD_SIZE_BYTES, 
      "Beacon record size is wrong. Reading beacon.bin should fail");

#define BEACON_STACK_SIZE        16384


// the index file holds the offset for the first record in each row
//    as well as the number of entries in that row (note that offset
//    is number of records, not number of bytes)
struct beacon_index_record {
   uint32_t offset_to_first;
   uint32_t num_records;
};
typedef struct beacon_index_record beacon_index_record_type;
#define BEACON_IDX_RECORD_SIZE_BYTES      (8)
_Static_assert(sizeof(beacon_index_record_type) == 
      BEACON_IDX_RECORD_SIZE_BYTES, 
      "Beacon idx record size is wrong. Reading beacon.idx should fail");

////////////////////////////////////////////////////////////////////////

struct world_map;
typedef struct world_map world_map_type;
struct path_map;
typedef struct path_map path_map_type;

int load_beacons(
      /* in     */ const world_map_type *world_map,   // for position
      /* in out */       path_map_type *path_map      // store beacon info
      );

uint32_t get_tot_num_beacons(void);

uint32_t get_start_idx(
      /* in     */ const uint32_t row
      );

uint32_t get_num_beacons(
      /* in     */ const uint32_t row
      );

beacon_record_type * get_beacon_record(
      /* in     */ const uint32_t idx
      );

// find which beacons are in world/path map. store reference to them
//    in the path map, including where they appear and what their
//    weights are
void load_beacons_into_path_map(
      /* in out */       path_map_type *path_map
      );

#endif   // BEACON_H

