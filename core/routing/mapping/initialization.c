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
#if !defined(INITIALIZATION_C)
#define INITIALIZATION_C

////////////////////////////////////////////////////////////////////////
// initialization and reset

void init_route_info(
      /*    out */       route_info_type *info,
      /* in     */ const world_coordinate_type start_point,
//      /* in     */ const meters_per_second_type pref_speed,
      /* in     */ const world_coordinate_type destination,
      /* in     */ const meter_type destination_radius
      )
{
   //
   info->sug_heading.tru.angle32 = 0;
   info->sug_heading_score = 1.0;
   // set otto course score as neg until driver engaged, even if otto sleeping
   info->autopilot_course.tru.angle32 = 0;
   info->autopilot_course_score = -1.0;
   info->measured_heading.tru.angle32 = 0;
   info->measured_heading_score = 1.0;
//   //
//   info->sug_speed = pref_speed;
//   info->pref_speed = pref_speed;
//   info->present_speed.mps = 0.0;
   //
   info->flags_all = 0;
   //
   info->start_position = start_point;
   if (info->start_position < 0.0) {
      info_start_position += 360.0;
   }
   info->destination = destination;
   if (info->destination.lon < 0.0) {
      info->destination.lon += 360.0;
   }
   info->destination_radius = destination_radius;
}


static void init_route_map(
      /* in out */       route_map_type *route_map
      )
{
   // make sure x and x dimensions are odd and equal
   assert(route_map->size.x & 1);
   assert(route_map->size.y & 1);
   assert(route_map->size.x == route_map->size.y);
   /////////////////////////////////////////////////
   uint32_t idx = 0;
   int32_t rad = route_map->size.x / 2;
   for (int32_t y=-rad; y<=rad; y++) {
      // closest distance to node vertically
      double dy = (double) abs(y) - 0.5;
      if (dy < 0.0) {
         dy = 0.0;
      }
      for (int32_t x=-rad; x<=rad; x++) {
         route_map_node_type *node = &route_map->nodes[idx++];
         node->pos.x = (int16_t) x;
         node->pos.y = (int16_t) y;
         // initialize node fields
         if ((x | y) == 0) {     // if x and y are zero
            node->distance.radians = 0.0;
            node->radial.angle16 = 0;
            node->radial_left_edge.angle16 = 0;
            node->radial_right_edge.sangle16 = -1;
         } else {
            // distance of node from center
            double dist_abs = sqrt((double) (x*x + y*y));
            node->distance.radians = dist_abs / (double) (route_map->size.x/2);
            // radial of node
            double theta_deg = R2D * atan2((double) x, (double) -y);
            CVT_DEG_TO_BAM16(theta_deg, node->radial);
//            node->radial.sangle16 = (int16_t) (theta_deg * DEG_TO_BAM16);
            // left and right radials of node
            // width of node is based on node's center, not node's nearest
            //    edge, and nearest edge will have wider arc than center.
            // estimate closest point
            double dx = (double) abs(x) - 0.5;
            if (dx < 0.0) {
               dx = 0.0;
            }
            // this is more pronounced with closer nodes than farther nodes
            // approximate a correcting factor based on node's distance
            double dist_closest = sqrt(dx * dx + dy * dy);
            double half_arc_deg = R2D * atan(0.5 / dist_closest);
            bam16_type half_arc;
            CVT_DEG_TO_BAM16(half_arc_deg, half_arc);
//            uint16_t half_arc_ss = (uint16_t) (DEG_TO_BAM16 * half_arc_deg);
            node->radial_left_edge.angle16 =
                  (uint16_t) (node->radial.angle16 - half_arc.angle16);
            node->radial_right_edge.angle16 =
                  (uint16_t) (node->radial.angle16 + half_arc.angle16);
//printf("%d,%d   dist=%f   theta=%f   center=%d   left=%d   right=%d\n", x, y, (double) node->distance.radians, (double) theta_deg, node->radial.angle16, node->radial_left_edge.angle16, node->radial_right_edge.angle16);
         }
      }
   }
}

route_map_type * create_route_map(
      /* in     */ const meter_type node_width
      )
{
   route_map_type *map = calloc(1, sizeof *map);
   map->size.x = ROUTE_MAP_WIDTH;
   map->size.y = ROUTE_MAP_HEIGHT;
   uint32_t n_elements = (uint32_t) (map->size.x * map->size.y);
   map->nodes = malloc(n_elements * sizeof *map->nodes);
   map->node_width = node_width;
   //
   init_route_map(map);
   //
   return map;
}


// vessel is at center
// update route node values. includes assigning route nodes to world and
//    path map nodes. computes time to reach other route nodes, and thus
//    underlying world nodes
// also resets rest of node's state values
// TODO interpolate depth in near-shore world nodes, to stay further away
//    from edges of nodes containing hazards
static void reset_route_nodes(
      /* in     */ const path_map_type *path_map,
      /* in out */       route_map_type *route_map,
      /* in     */ const vessel_position_info_type *vessel_info,
      /* in     */ const route_info_type *route_info
      )
{
   const world_coordinate_type vessel_pos = vessel_info->position;
   const meters_per_second_type vessel_speed = vessel_info->speed;
   // 'radius' of routing map (ie, width/2 in meters)
   const meter_type routing_radius =
         { .meters = route_map->node_width.meters * route_map->size.x / 2 };
   // 'radius' of routing node. radius here is to corners, not sides. this
   //    will result in overlap between nodes but as we're only looking at
   //    intersecting paths, that will provide some safety cushion
   const meter_type route_node_radius =
         { .meters = route_map->node_width.meters / 1.414 };
printf(" vessel pos %.6f,%.6f  speed %.2f heading %.1f  xy mps:%.3f,%.3f\n", (double) vessel_pos.x_deg, (double) vessel_pos.y_deg, (double) vessel_speed.mps, (double) vessel_info->true_heading.tru.angle32 * BAM32_TO_DEG, (double) vessel_info->xy_motion.x_mps, (double) vessel_info->xy_motion.y_mps);
//printf(" routing radius %.1f met    node radius %.1f met\n", (double) routing_radius.meters, (double) route_node_radius.meters);
   /////////////////////////////
   // get meter offset of vessel from world map center
   // world and plan maps are rectangular nautical miles while routing map is
   //    rectangular meters, so no distortion correction necessary (world
   //    and plan used to be degrees, in which case there was distortion)
   // routing map is smaller than world/plan maps and has smaller grids
   image_size_type size = route_map->size;
   meter_type dx, dy;
//fprintf(stderr, "ves pos: %.3f,%.3f\n", vessel_pos.x_deg, vessel_pos.y_deg);
//fprintf(stderr, "map center: %.3f,%.3f\n", path_map->center.x_deg, path_map->center.y_deg);
   calc_meter_offset(path_map->center, vessel_pos, &dx, &dy, __func__);
//printf("  vessel at %.4f,%.4f,   world map center offset (m) %.1f,%.1f\n", vessel_pos.lon, vessel_pos.lat, dx.meters, dy.meters);
// convert meters to pixels (grid squares)
int32_t x_offset_pix =
      (int32_t) floor(dx.meters / path_map->node_width.meters);
// positive dy is up, whereas here it should be down as array origin is
//    in top-left
int32_t y_offset_pix =
      (int32_t) floor(-dy.meters / path_map->node_height.meters);
printf("    node offset %d,%d\n", 360+x_offset_pix, 360+y_offset_pix);
   // distance to top/left edge of route map, relative to world map center
   // positive dx is rightward
   // positive dy should be downward (array origin is top-left) but returned
   //    value has +dy as upward. need to invert it
   double route_left_offset_met = dx.meters -
         (double) (size.x / 2) * ROUTE_MAP_NODE_WIDTH_METERS;
   double route_top_offset_met = -dy.meters -
         (double) (size.y / 2) * ROUTE_MAP_NODE_WIDTH_METERS;
//printf("Relative top/left of route map  %.1f,%.1f (met)\n", route_left_offset_met, route_top_offset_met);
//printf("Relative top/left of route map  %.1f,%.1f (met)\n", 360.0+floor(route_left_offset_met/path_map->node_width.meters), 360.0+floor(route_top_offset_met/path_map->node_width.meters));
   // top/left inset of route map in world map
   double route_left_inset_met =
         (double) (path_map->size.x/2) * path_map->node_width.meters +
         route_left_offset_met;
   double route_top_inset_met =
         (double) (path_map->size.y/2) * path_map->node_height.meters +
         route_top_offset_met;
//printf("Route inset left %.1fm  top %.1fm\n", route_left_inset_met, route_top_inset_met);
   /////////////////////////////
   // update nodes, with route map center being vessel position
assert(size.x == 255);
assert(size.y == 255);
   uint32_t route_idx = 0;
   for (uint32_t y=0; y<size.y; y++) {
      double y_inset_met = route_top_inset_met + y * ROUTE_MAP_NODE_WIDTH_METERS;
      // y index in world map
      uint32_t map_y_pos = (uint32_t)
            floor(y_inset_met / (double) path_map->node_height.meters);
      uint32_t y_row_idx = map_y_pos * path_map->size.x;
      assert(map_y_pos < 720);
      for (uint32_t x=0; x<size.x; x++) {
         route_map_node_type *node = &route_map->nodes[route_idx++];
         double x_inset_met =
               route_left_inset_met + x * ROUTE_MAP_NODE_WIDTH_METERS;
         // x index in world map
         uint32_t map_x_pos = (uint32_t)
               floor(x_inset_met / (double) path_map->node_width.meters);
         assert(map_x_pos < 720);
         uint32_t map_idx = map_x_pos + y_row_idx;
         //
         node->world_node_idx = map_idx;
         node->world_pos.x = (uint16_t) map_x_pos;
         node->world_pos.y = (uint16_t) map_y_pos;
assert(node->world_node_idx < 720 * 720);
//path_map_node_type *wn = &path_map->nodes[map_idx];
////printf("%d,%d -> map_idx %d (%d,%d)  depth %d\n", x, y, map_idx, map_x_pos, map_y_pos, wn->depth_meters);
//printf("%d,%d -> map_idx %d (%d,%d   %.2f,%.2f)  depth %d\n", x, y, map_idx, map_x_pos, map_y_pos, x_inset_met / (double) path_map->node_width.meters, y_inset_met / (double) path_map->node_height.meters, wn->depth_meters);
         ///////////////////////////////////////////////////////////////
         // rest of state values
         // default to "don't go to this node". that will be overriden
         //    when updating terrain viaiblity
         node->terrain_score = 0.0001;
         // time to arrive at and leave node
         double dist_near_m = node->distance.radians * routing_radius.meters
               - route_node_radius.meters;
         if (dist_near_m < 0.0) {
            dist_near_m = 0.0;
         }
         double dist_far_m = node->distance.radians * routing_radius.meters
               + route_node_radius.meters;
assert(node->distance.radians >= 0.0);
         if (vessel_speed.mps > 0.0) {
            node->arrival_dt_sec = dist_near_m / vessel_speed.mps;
            node->exit_dt_sec = dist_far_m / vessel_speed.mps;
//printf("    arrive %.3f   exit %.3f    speed %.3f   dist_near %.3f   dist_far %.3f\n", node->arrival_dt_sec, node->exit_dt_sec, vessel_speed.mps, dist_near_m, dist_far_m);
//printf(  "%d,%d  arrive %.3f   exit %.3f    speed %.3f   dist_near %.3f   dist_far %.3f\n", x, y, node->arrival_dt_sec, node->exit_dt_sec, vessel_speed.mps, dist_near_m, dist_far_m);
         } else {
            // if vessel has no speed then it's possible that the speed
            //    indicator is broken. use default speed
            assert(route_info->default_speed.mps > 0.0);
            node->arrival_dt_sec = dist_near_m / route_info->default_speed.mps;
            node->exit_dt_sec = dist_far_m / route_info->default_speed.mps;
//printf("    arrive %.3f   exit %.3f    def speed %.3f   dist_near %.3f   dist_far %.3f\n", node->arrival_dt_sec, node->exit_dt_sec, route_info->default_speed.mps, dist_near_m, dist_far_m);
//printf("  %d,%d  arrive %.3f   exit %.3f    def speed %.3f   dist_near %.3f   dist_far %.3f\n", x, y, node->arrival_dt_sec, node->exit_dt_sec, route_info->default_speed.mps, dist_near_m, dist_far_m);
         }
      }
//if (y&1) {
//   printf("\n");
//}
   }
////   // spherical distortion means there's going to be different numbers of
////   //    degrees for lat and lon
//   double node_height_deg = (double) route_map->node_width.meters *
//         METER_TO_DEG_LAT;
//   double node_width_deg = node_height_deg;  // close enough approximation
//   // TODO check that vessel isn't too near to world border
//   // to handle this we'll need to signal an error that causes a new map
//   //    to load. ideally we won't need to do that as map will have already
//   //    been adjusted so a simple assert will work here
//   //
//   //
}


static void clear_viability_radials(
      /* in out */       route_map_type *route_map
      )
{
   radial_viability_type *radials = route_map->radials;
   //////////
   // reset radials to 1.0 then update w/ route node values
   for (uint32_t i=0; i<NUM_ROUTE_RADIALS; i++) {
      for (uint32_t ival=0; ival<NUM_VIABILITY_INTERVALS; ival++) {
         radials[i].terrain_score[ival] = 1.0;
         radials[i].stand_on_score[ival] = 1.0;
         radials[i].give_way_score[ival] = 1.0;
      }
   }
}


// initialization and reset
////////////////////////////////////////////////////////////////////////
#endif      // INITIALIZATION_C
