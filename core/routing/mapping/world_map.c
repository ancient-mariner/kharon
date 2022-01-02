#if !defined(WORLD_MAP_C)
#define WORLD_MAP_C
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <string.h>
#include "pinet.h"
#include "pin_types.h"
#include "lin_alg.h"
#include "logger.h"
#include "routing/mapping.h"
// from mapping/include/
#include "world_map.h"

static char world_map_folder_[STR_LEN] = { 0 };

static log_info_type *log_ = NULL;

#include "share.c"
#include "beacon.c"
#include "path_map.c"
#include "zoutput.c"


static void mark_land_adjacency(
      /*    out */       path_map_type *path_map
      )
{
   image_size_type size = path_map->size;
   // immediate adjacency
   for (uint32_t y=1; y<size.y-1u; y++) {
      for (uint32_t x=1; x<size.x-1u; x++) {
         uint32_t idx = (uint32_t) (x + y * size.x);
         map_feature_node_type *node = &path_map->feature_nodes[idx];
         if (node->depth_meters <= MIN_TRAVERSABLE_DEPTH_METERS) {
            // mark neighbors as being non-passable
            path_map->feature_nodes[idx-size.x-1].land_cnt++;
            path_map->feature_nodes[idx-size.x  ].land_cnt++;
            path_map->feature_nodes[idx-size.x+1].land_cnt++;
            path_map->feature_nodes[idx       -1].land_cnt++;
            path_map->feature_nodes[idx         ].land_cnt++;
            path_map->feature_nodes[idx       +1].land_cnt++;
            path_map->feature_nodes[idx+size.x-1].land_cnt++;
            path_map->feature_nodes[idx+size.x  ].land_cnt++;
            path_map->feature_nodes[idx+size.x+1].land_cnt++;
         }
      }
   }
   // adjacency once removed
   for (uint32_t y=2; y<size.y-2u; y++) {
      for (uint32_t x=2; x<size.x-2u; x++) {
         int32_t idx = (int32_t) (x + y * size.x);
         map_feature_node_type *node = &path_map->feature_nodes[idx];
         if (node->depth_meters <= MIN_TRAVERSABLE_DEPTH_METERS) {
            // mark neighbors as being adjacent non-passable
            path_map->feature_nodes[idx-2*size.x-1].near_cnt++;
            path_map->feature_nodes[idx-2*size.x  ].near_cnt++;
            path_map->feature_nodes[idx-2*size.x+1].near_cnt++;
            // 
            path_map->feature_nodes[idx-  size.x-2].near_cnt++;
            path_map->feature_nodes[idx-  size.x+2].near_cnt++;
            //
            path_map->feature_nodes[idx-        -2].near_cnt++;
            path_map->feature_nodes[idx-        +2].near_cnt++;
            //
            path_map->feature_nodes[idx+  size.x-2].near_cnt++;
            path_map->feature_nodes[idx+  size.x+2].near_cnt++;
            //
            path_map->feature_nodes[idx+2*size.x-1].near_cnt++;
            path_map->feature_nodes[idx+2*size.x  ].near_cnt++;
            path_map->feature_nodes[idx+2*size.x+1].near_cnt++;
         }
      }
   }
   //  33333
   // 3322233
   // 3211123
   // 3210123
   // 3211123
   // 3322233
   //  33333
   // adjacency twice removed
   for (uint32_t y=3; y<size.y-3u; y++) {
      for (uint32_t x=3; x<size.x-3u; x++) {
         int32_t idx = (int32_t) (x + y * size.x);
         map_feature_node_type *node = &path_map->feature_nodes[idx];
         if (node->depth_meters <= MIN_TRAVERSABLE_DEPTH_METERS) {
            // mark neighbors as being near non-passable
            path_map->feature_nodes[idx-3*size.x-2].near_cnt++;
            path_map->feature_nodes[idx-3*size.x-1].near_cnt++;
            path_map->feature_nodes[idx-3*size.x  ].near_cnt++;
            path_map->feature_nodes[idx-3*size.x+1].near_cnt++;
            path_map->feature_nodes[idx-3*size.x+2].near_cnt++;
            // 
            path_map->feature_nodes[idx-2*size.x-3].near_cnt++;
            path_map->feature_nodes[idx-2*size.x+2].near_cnt++;
            path_map->feature_nodes[idx-2*size.x-2].near_cnt++;
            path_map->feature_nodes[idx-2*size.x+3].near_cnt++;
            //
            path_map->feature_nodes[idx-1*size.x-3].near_cnt++;
            path_map->feature_nodes[idx-1*size.x+3].near_cnt++;
            //
            path_map->feature_nodes[idx-        -3].near_cnt++;
            path_map->feature_nodes[idx-        +3].near_cnt++;
            //
            path_map->feature_nodes[idx+1*size.x-3].near_cnt++;
            path_map->feature_nodes[idx+1*size.x+3].near_cnt++;
            //
            path_map->feature_nodes[idx+2*size.x-3].near_cnt++;
            path_map->feature_nodes[idx+2*size.x+2].near_cnt++;
            path_map->feature_nodes[idx+2*size.x-2].near_cnt++;
            path_map->feature_nodes[idx+2*size.x+3].near_cnt++;
            //
            path_map->feature_nodes[idx+3*size.x-2].near_cnt++;
            path_map->feature_nodes[idx+3*size.x-1].near_cnt++;
            path_map->feature_nodes[idx+3*size.x  ].near_cnt++;
            path_map->feature_nodes[idx+3*size.x+1].near_cnt++;
            path_map->feature_nodes[idx+3*size.x+2].near_cnt++;
         }
      }
   }
}


// loads depth map and sets path_map->center
void load_world_5sec_map(
      /* in     */ const world_coordinate_type map_center,
      /* in out */       path_map_type *path_map
      )
{
   // make sure map is properly created
   world_coordinate_type center = map_center;
   if (center.lon < 0) {
      center.lon += 360.0;
   }
akn_position_type apos = convert_latlon_to_akn(center);
printf("Loading 5-sec map at %.5f,%.5f  (%.5f,%.5f)\n", center.x_deg, center.y_deg, apos.akn_x, apos.akn_y);
   check_world_coordinate(center, __func__);
   assert(path_map->size.x == 720);
   assert(path_map->size.y == 720);
   // any old beacons are stale on map load. at best it's confusing to 
   //    let them stay around
   path_map->num_beacons = 0; 
   /////////////////////////////////////////////
   // (re)set variables
   path_map->center.x_deg = center.x_deg;
   path_map->center.y_deg = center.y_deg;
//   path_map->node_height.degrees = 1.0f / 720.0f;
//   path_map->node_width.degrees = 1.0f / 720.0f;
   path_map->map_width.meters = (float) (60.0 * NM_TO_METERS);
   path_map->map_height.meters = (float) (60.0 * NM_TO_METERS);
   path_map->node_width.meters = path_map->map_width.meters / 720.0;
   path_map->node_height.meters = path_map->map_height.meters / 720.0;
   // nodes are 'square' in the sense of degrees so no latitude correction
   //    is necessary. this is OK up to Anchorage latitude, but perhaps 
   //    starts to fail above Prudoe
   /////////////////////////////////////////////
   // fetch map from database
   map_level3_type map3;
   build_60x60_map(world_map_folder_, center, &map3);
   double decl, incl;
   char declination_fname[STR_LEN];
   snprintf(declination_fname, STR_LEN, "%s%s", 
         world_map_folder_, DECLINATION_FILE);
   if (load_declination(declination_fname, center, &decl, &incl) != 0) {
      // for now, treat inability to read declination as fatal error
      // TODO treat as soft err and have fallback
      log_err(log_, "Failed to read/parse declination file '%s'", 
            DECLINATION_FILE);
      hard_exit(__FILE__, __LINE__);
   }
   path_map->declination.degrees = decl;
   path_map->inclination.degrees = incl;
   log_info(log_, "Declination of %.1f,%.1f read at %.2f degrees",
         center.lon, center.lat, path_map->declination.degrees);
//   uint32_t n_squares = 720 * 720;
   uint32_t idx = 0;
   for (uint32_t y=0; y<720; y++) {
      for (uint32_t x=0; x<720; x++) {
         uint8_t code = map3.grid[idx].min_depth;
         map_feature_node_type *square = &path_map->feature_nodes[idx];
         square->depth_meters = (int16_t) decode_submap_depth(code);
//if (square->depth_meters > 10000) {
//   printf("%d,%d has excessive depth. code %d, depth %d\n", x, y, code, square->depth_meters);
//}
         square->cnt_all = 0;
         idx++;
      }
   }
   //
   mark_land_adjacency(path_map);
}


void set_world_map_folder_name(
      /* in     */ const char *name
      )
{
   strncpy(world_map_folder_, name, STR_LEN);
   size_t len = strlen(world_map_folder_);
   assert(len < STR_LEN-2);
   assert(len > 0);
   // make sure folder name terminates with '/'
   if (world_map_folder_[len-1] != '/') {
      world_map_folder_[len] = '/';
      world_map_folder_[len+1] = 0;
   }
}

const char* get_world_map_folder_name(void)
{
   // make sure folder name was initialized 
   // TODO implement less fragile logic -- ie, don't assert-crash in
   //    event of configuration oversight
   if (world_map_folder_[0] == 0) {
      fprintf(stderr, "Configuration error: world map folder not defined\n");
      fprintf(stderr, "Add set_world_map_folder() in config file\n");
   }
   assert(world_map_folder_[0] != 0);
   return world_map_folder_;
}

#endif   // WORLD_MAP_C
