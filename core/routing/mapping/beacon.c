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
#include "pin_types.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <errno.h>
#include "beacon.h"
#include "routing/mapping.h"
#include "lin_alg.h"

// storage for beacon path map
// there should only be one mapping system operating in this memory space
//    so don't bother with thread-specific storage
static beacon_index_record_type *beacon_indices_ = NULL;
static beacon_record_type *beacon_list_ = NULL;

static float *beacon_weights_ = NULL;
static uint32_t tot_num_beacons_ = 0;


// beacon pathfinding
static uint32_t beacon_stack_write_idx_ = 0;
static uint32_t beacon_stack_read_idx_ = 0;
static uint32_t beacon_index_stack_[BEACON_STACK_SIZE];

////////////////////////////////////////////////////////////////////////

uint32_t get_tot_num_beacons(void)
{
   return tot_num_beacons_;
}

uint32_t get_start_idx(
      /* in     */ const uint32_t row
      )
{
   assert(row < 180);
   return beacon_indices_[row].offset_to_first;
}

// number of beacons in specified row
uint32_t get_num_beacons(
      /* in     */ const uint32_t row
      )
{
   assert(row < 180);
   return beacon_indices_[row].num_records;
}

beacon_record_type * get_beacon_record(
      /* in     */ const uint32_t idx
      )
{
   return &beacon_list_[idx];
}

////////////////////////////////////////////////////////////////////////

int init_beacon_list(void)
{
   if (beacon_list_ != NULL) {
      return 0;
   }
   int rc = -1;
   FILE *bin_fp = NULL;
   FILE *idx_fp = NULL;
   uint64_t num_bytes;
   const char *map_root = get_world_map_folder_name();
   /////////////////////////////////////////////////////////////////////
   // index list
   char idx_name[STR_LEN];
   snprintf(idx_name, STR_LEN, "%s%s", map_root, BEACON_IDX_FILE);
   idx_fp = fopen(idx_name, "r");
   if (idx_fp == NULL) {
      fprintf(stderr, "Unable to open beacon index file '%s': %s\n",
            idx_name, strerror(errno));
      goto end;
   }
   fseek(idx_fp, 0, SEEK_END);
   num_bytes = (uint64_t) ftell(idx_fp);
   // 180 degress latitude
   uint64_t expected_bytes = 180l * sizeof *beacon_indices_;
   if (num_bytes != expected_bytes) {
      // oh shit
fprintf(stderr, "Beacon index is wrong size (%ld, expected %ld)",
      num_bytes, expected_bytes);
      log_err(log_, "Beacon index is wrong size (%ld, expected %ld)",
            num_bytes, expected_bytes);
      goto end;
   }
   fseek(idx_fp, 0, SEEK_SET);
   beacon_indices_ = malloc(num_bytes);
   if (fread(beacon_indices_, num_bytes, 1, idx_fp) != 1) {
fprintf(stderr, "Failed read beacon index\n");
      log_err(log_, "Failed read beacon index");
      goto end;
   }
   /////////////////////////////////////////////////////////////////////
   // beacon list
   char bin_name[STR_LEN];
   // open bin file and advance to read point
   snprintf(bin_name, STR_LEN, "%s%s", map_root, BEACON_FILE);
   bin_fp = fopen(bin_name, "r");
   if (bin_fp == NULL) {
fprintf(stderr, "Unable to open beacon data file '%s': %s\n", bin_name, strerror(errno));
      log_err(log_, "Unable to open beacon data file '%s': %s",
            bin_name, strerror(errno));
      goto end;
   }
   // get file size, create storage buffer, then read into it
   fseek(bin_fp, 0, SEEK_END);
   num_bytes = (uint64_t) ftell(bin_fp);
   fseek(bin_fp, 0, SEEK_SET);
   if ((beacon_list_ = malloc(num_bytes)) == NULL) {
fprintf(stderr, "Failed to allocate %ld bytes for beacon list\n", num_bytes);
      log_err(log_, "Failed to allocate %ld bytes for beacon list", num_bytes);
      goto end;
   }
   if (fread(beacon_list_, num_bytes, 1, bin_fp) != 1) {
fprintf(stderr, "Failed read on beacon list\n");
      log_err(log_, "Failed read on beacon list");
      goto end;
   }
   /////////////////////////////////////////////////////////////////////
   // weight array
   tot_num_beacons_ = (uint32_t) (num_bytes / BEACON_BIN_RECORD_SIZE_BYTES);
//printf("NUM BEACONS %d\n", tot_num_beacons_);
   beacon_weights_ = malloc(tot_num_beacons_ * sizeof *beacon_weights_);
   rc = 0;
end:
   if (idx_fp != NULL) {
      fclose(idx_fp);
   }
   if (bin_fp != NULL) {
      fclose(bin_fp);
   }
   return rc;
}

// init
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
// beacon pathfinding


// move path_map stack contents down, resetting read position to index 0
// this purges already-read elements
static void shrink_beacon_path_stack(void)
{
   uint32_t stack_len =
         (uint32_t) (beacon_stack_write_idx_ - beacon_stack_read_idx_);
printf("Shrinking path-map stack. Len=%d. Purging %d elements\n", stack_len, beacon_stack_read_idx_);
   log_info(log_, "Shrinking beacon path-map stack. Len=%d. Purging %d elements",
         stack_len, beacon_stack_read_idx_);
   memmove(beacon_index_stack_, &beacon_index_stack_[beacon_stack_read_idx_],
         stack_len * sizeof *beacon_index_stack_);
   beacon_stack_read_idx_ = 0;
   beacon_stack_write_idx_ = stack_len;
}


// evaluate neighbor beacon. update it's path weight if necessary and
//    if so than add to stack so its neighbors can be evaluated
static void add_beacon_to_stack(
      /* in     */ const beacon_record_type *root_beacon,
      /* in     */ const float weight_to_neighbor,
      /* in     */ const beacon_record_type *neighbor
      )
{
printf("  Check add %d to stack\n", neighbor->index);
   if (neighbor->num_neighbors <= 0) {
      // neighbor not initialized or it doesn't recognize that it has
      //    neighbors. ignore it
printf("    no neighbors\n");
      goto end;
   }
   float root_weight = beacon_weights_[root_beacon->index];
   float nbr_weight = beacon_weights_[neighbor->index];
   float path_weight = root_weight + weight_to_neighbor;
printf("    weight %.1f  (root %.1f  path %.1f  proposed %.1f\n", (double) nbr_weight, (double) root_weight, (double) weight_to_neighbor, (double) path_weight);
   if ((nbr_weight < 0.0f) || (nbr_weight > path_weight)) {
printf("    Weight is now %.1f. Adding %d\n", (double) path_weight, neighbor->index);
      // update neighbor's path weight and add to stack to re-evaluate
      //    its neighbors
      beacon_weights_[neighbor->index] = path_weight;
      beacon_index_stack_[beacon_stack_write_idx_++] = neighbor->index;
   }
end:
   ;
}


static void process_next_stack_beacon(void)
{
   assert(beacon_stack_read_idx_ < beacon_stack_write_idx_);
   uint32_t stack_idx = beacon_stack_read_idx_++;
   uint32_t root_idx = beacon_index_stack_[stack_idx];
   beacon_record_type *root_beacon = get_beacon_record(root_idx);
   // signed int32 for i as num_neighbors is signed (neg there
   //    means unprocessed)
printf("beacon %d has %d neighbors\n", root_beacon->index, root_beacon->num_neighbors);
   for (int32_t i=0; i<root_beacon->num_neighbors; i++) {
      beacon_neighbor_type *nbr = &root_beacon->neighbors[i];
      beacon_record_type *neighbor = get_beacon_record(nbr->nbr_index);
      add_beacon_to_stack(root_beacon, nbr->path_weight, neighbor);
   }
}

// beacon pathfinding
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

// calculate display location of arbitrary position
static image_coordinate_type calculate_map_position_apos(
      /* in     */ const path_map_type *path_map,
      /* in     */ const akn_position_type apos
      )
{
   world_coordinate_type wpos = convert_akn_to_world(apos);
   return get_pix_position_in_map(path_map, wpos);
}


//// calculate beacon's display locations
//// beacon_num is number of beacon in path map, ie, on [0,8)
//image_coordinate_type calculate_beacon_map_position(
//      /* in     */ const path_map_type *path_map,
//      /* in     */ const uint32_t beacon_num    // beacon # in path map
//      )
//{
//   assert(beacon_num < MAX_BEACON_NEIGHBORS);
//   return calculate_map_position_apos(path_map,
//         path_map->beacon_ref[beacon_num].coords);
////   akn_position_type apos = path_map->beacon_ref[beacon_num].coords;
////   world_coordinate_type wpos = convert_akn_to_world(apos);
////   image_coordinate_type map_pos = get_pix_position_in_map(path_map, wpos);
////   path_map->beacon_ref[beacon_num].pos_in_map = map_pos;
//}


// calculate destination's display location
static void calculate_destination_map_position(
      /* in out */       path_map_type *path_map
      )
{
   world_coordinate_type wpos = convert_akn_to_world(path_map->destination);
   image_coordinate_type map_pos = get_pix_position_in_map(path_map, wpos);
   path_map->dest_pix = map_pos;
}

// resort path-map's beacon list, or rather, partial resort
// only do one pass, from list rear, as list is mostly sorted already
//    and the last entry is the one that can be out of order
static void resort_beacons(
      /* in out */       path_map_type *path_map
      )
{
   if (path_map->num_beacons <= 1) {
      return;
   }
   map_beacon_reference_type *beacon_ref = path_map->beacon_ref;
   uint32_t end_slot = path_map->num_beacons - 1u;
//printf("Resort\n");
//for (uint32_t i=0; i<path_map->num_beacons; i++) {
//   const map_beacon_reference_type *ref = &beacon_ref[i];
//   printf("  %d\tidx %6d\t%.4f,%.4f\n", i, ref->index, ref->coords.akn_x, ref->coords.akn_y);
//}
   while (end_slot > 0) {
      uint32_t prev_slot = end_slot - 1;  // slot before end
      float end_dist = beacon_ref[end_slot].center_dist_met;
      float prev_dist = beacon_ref[prev_slot].center_dist_met;
      if (end_dist < prev_dist) {
         // swap
         map_beacon_reference_type tmp = beacon_ref[prev_slot];
         beacon_ref[prev_slot] = beacon_ref[end_slot];
         beacon_ref[end_slot] = tmp;
      } else {
         // no swap necessary as position is correct (ie, smaller
         //    values after and higher before). nothing more to do
         break;
      }
      // move 'end' one back from end of list and check it
      end_slot = prev_slot;
   }
//printf(" ->\n");
//for (uint32_t i=0; i<path_map->num_beacons; i++) {
//   const map_beacon_reference_type *ref = &path_map->beacon_ref[i];
//   printf("  %d\tidx %6d\t%.4f,%.4f\n", i, ref->index, ref->coords.akn_x, ref->coords.akn_y);
//}
}

// if specified beacon is closer to map center than those already stored
//    in path map, add it to path map's beacon list and resort the list
static void add_beacon_to_list(
      /* in     */ const akn_position_type map_center,
      /* in     */ const beacon_record_type *candidate,
      /* in     */ const uint32_t beacon_idx,
      /* in     */ const double lat_correction,
      /* in out */       path_map_type *path_map
      )
{
   // maps are 60x60nm and latitude distortions need to be considered
   float dx = fabsf(candidate->akn_x - (float) map_center.akn_x);
   float dy = fabsf(candidate->akn_y - (float) map_center.akn_y);
   if (dx > 180.0f) {
      dx = 360.0f - dx;
   }
   assert(dx <= 180.0f);
   assert(dx >= 0.0f);
   dx *= (float) lat_correction;   // what dx would be in degrees at equator
//float dd = sqrtf(dx*dx + dy*dy);
//printf("center %.4f,%.4f   check %.4f,%.4f  (delta %.4f,%.4f   %.4f)\n", map_center.akn_x, map_center.akn_y, (double) candidate->akn_x, (double) candidate->akn_y, (double) dx, (double) dy, (double) dd);
   if ((dx >= 0.5f) || (dy >= 0.5f)) {
      // if more than 0.5 deg then more than 1/2 map height|width, so is
      //    outside of map
      goto end;
   }
   /////////////////////////////////////////////
   float dist = sqrtf(dx*dx + dy*dy);
   // if beacon is too close to map center (e.g., if beacon is near/at
   //    center, which happens when tracing beacons) then ignore it
   if (dist < 1.0f/60.0f) {
      goto end;
   }
   const uint32_t n = path_map->num_beacons;
   //
   if (n < MAX_BEACON_NEIGHBORS) {
      // make sure beacon is w/in map
      akn_position_type apos;
      apos.akn_x = (double) candidate->akn_x;
      apos.akn_y = (double) candidate->akn_y;
      image_coordinate_type pix = calculate_map_position_apos(path_map, apos);
      if ((pix.x < MAP_LEVEL3_SIZE) && (pix.y < MAP_LEVEL3_SIZE)) {
         // beacon is w/in map
         // list isn't full yet. add record to end of list then resort
         map_beacon_reference_type *ref = &path_map->beacon_ref[n];
         ref->center_dist_met = dist;
         ref->index = beacon_idx;
         ref->coords = apos;
         ref->pos_in_map = pix;
//printf("Adding (%d) %.4f,%.4f  dist %.4f,%.4f->%.4f  at %d,%d\n", n, ref->coords.akn_x, ref->coords.akn_y, (double) dx, (double) dy, (double) dist, pix.x, pix.y);
         path_map->num_beacons++;
      }
   } else {
      map_beacon_reference_type *ref =
            &path_map->beacon_ref[MAX_BEACON_NEIGHBORS-1];
      if (dist < ref->center_dist_met) {
         // beacon is closer than the farthest beacon in list
         // make sure beacon is w/in map
         akn_position_type apos;
         apos.akn_x = (double) candidate->akn_x;
         apos.akn_y = (double) candidate->akn_y;
         image_coordinate_type pix =
               calculate_map_position_apos(path_map, apos);
         if ((pix.x < MAP_LEVEL3_SIZE) && (pix.y < MAP_LEVEL3_SIZE)) {
            // closer and w/in map. replace end of list and resort
//printf("  purge %.4f,%.4f  %.4f\n", ref->coords.akn_x, ref->coords.akn_y, (double) ref->center_dist_met);
            ref->center_dist_met = dist;
            ref->index = beacon_idx;
            ref->coords = apos;
            ref->pos_in_map = pix;
//printf("Adding (%d) %.4f,%.4f  dist %.4f,%.4f->%.4f  at %d,%d\n", n-1, ref->coords.akn_x, ref->coords.akn_y, (double) dx, (double) dy, (double) dist, pix.x, pix.y);
         }
      }
   }
   resort_beacons(path_map);
end:
//printf("  distances: ");
//for (uint32_t i=0; i<path_map->num_beacons; i++) {
//   printf("%.4f ", (double) path_map->beacon_ref[i].center_dist_met);
//}
//printf("\n");
   ;
}


// finds range of beacons that have similar latitude as map center
// returns (by pointer) index of first record adn num records
// function returns 0 on success, -1 on failure
static void get_beacon_indices(
      /* in     */ const akn_position_type map_center,
      /*    out */       uint32_t *first_record,
      /*    out */       uint32_t *num_records
      )
{
   uint32_t row_low = (uint32_t) floor(map_center.akn_y);
   uint32_t row_high = (uint32_t) round(map_center.akn_y);
   assert(row_high <= 180);
   // get the rows for this position. path map will extend into
   //    prev or next row, as beacon bands are the same height as
   //    path map (ie, 1-deg).
   if (row_low == row_high) {
      // position is early in row (band). need to check previous
      //    for beacons
      if (row_low > 0) {
         row_low--;
      }
   } if (row_high == 180) {
      // make sure we don't overflow off bottom of list
      // this can only happen when over antarctica landmass, but no
      //    point in leaving a memory corruption risk even if it should
      //    never happen
      row_high = 179;
   }
   // pull data from idx list
   beacon_index_record_type *record = &beacon_indices_[row_low];
   *first_record = record->offset_to_first;
   *num_records = record->num_records;
   if (row_low != row_high) {
      record = &beacon_indices_[row_high];
      *num_records += record->num_records;
   }
}



// create path map among beacons. returns 0 on success and -1 on failure
static int trace_beacon_paths(
      /* in out */       path_map_type *path_map,
      /* in     */ const world_coordinate_type destination
      )
{
printf("TRACE BEACON\n");
   int rc = -1;
   assert(beacon_list_ != NULL); // make sure beacons initialized
   world_coordinate_type dest = destination;
   if (dest.lon < 0.0) {
      dest.lon += 360.0;
   }
   // init beacon path weights to invalid (ie, -1)
   for (uint32_t i=0; i<tot_num_beacons_; i++) {
      beacon_weights_[i] = -1.0f;
   }
   // trace paths with destination at center of map
   load_world_5sec_map(dest, path_map);
   trace_route_simple(path_map, dest);
   calculate_destination_map_position(path_map);
   load_beacons_into_path_map(path_map);
   if (path_map->num_beacons == 0) {
      // no beacons found near destination. this might be OK if vessel
      //    is close enough and there's a clear route between them,
      //    but for now consider this an error -- all navigable regions
      //    should have beacons near them, and if there are none
      //    that's a good filter for lack of navigability
fprintf(stderr, "No accessible beacons found near destination. It appears to be unreachable\n");
      log_err(log_, "No accessible beacons found near destination. It "
            "appears to be unreachable");
      goto end;
   }
   // for each beacon in path map, pull weight from it's position in
   //    the map and use those for path seeds
   // add those beacons to the stack so they can be traced outward from
   beacon_stack_read_idx_ = 0;
   beacon_stack_write_idx_ = 0;
   for (uint32_t i=0; i<path_map->num_beacons; i++) {
      image_coordinate_type pos = path_map->beacon_ref[i].pos_in_map;
      uint32_t map_idx = (uint32_t) (pos.x + pos.y * MAP_LEVEL3_SIZE);
      float weight = path_map->nodes[map_idx].weight;
      if (weight > 0.0f) {
         uint32_t beacon_idx = path_map->beacon_ref[i].index;
//printf("ADD %d   %.3f,%.3f   wt %.1f\n", beacon_idx, path_map->beacon_ref[i].coords.akn_x, path_map->beacon_ref[i].coords.akn_y, (double) weight);
         beacon_weights_[beacon_idx] = weight;
         // add beacon to stack
         beacon_index_stack_[beacon_stack_write_idx_++] = beacon_idx;
      }
   }
   // trace path
   while (beacon_stack_read_idx_ < beacon_stack_write_idx_) {
      if (beacon_stack_write_idx_ >= BEACON_STACK_SIZE) {
         shrink_beacon_path_stack();
      }
      process_next_stack_beacon();
   }
   rc = 0;
end:
//printf("end trace beacon\n");
//printf("BEACON WEIGHTS\n");
//for (uint32_t i=0; i<tot_num_beacons_; i++) {
//   beacon_record_type *rec = &beacon_list_[i];
//   printf("Beacon %d  (%.4f,%.4f) \twt %.1f\n", i, (double) rec->akn_x, (double) rec->akn_y, (double) beacon_weights_[i]);
//}
   return rc;
}


// find which beacons are in world/path map. store reference to them
//    in the path map, including where they appear and what their
//    weights are
void load_beacons_into_path_map(
      /* in out */       path_map_type *path_map
      )
{
   path_map->num_beacons = 0; // clean the slate
   akn_position_type map_center = convert_latlon_to_akn(path_map->center);
   // get index to first beacon record, and number of records to read
   //    from there
   uint32_t first_record, num_records;
   get_beacon_indices(map_center, &first_record, &num_records);
printf("Search indices %d to %d\n", first_record, first_record+num_records);
   // add beacons to path map. push all into map -- only the closest ones
   //    will stick
   const uint32_t start_idx = first_record;
   const uint32_t end_idx = first_record + num_records;
   const double lat_scale = sin(D2R * map_center.akn_y);
//printf("Lat scale %.3f for akn-lat %.4f\n", lat_scale, map_center.akn_y);
   for (uint32_t i=start_idx; i<end_idx; i++) {
      beacon_record_type *beacon_info = &beacon_list_[i];
//printf("Beacon %d  %f,%f\n", i, beacon_info->akn_x, beacon_info->akn_y);
      add_beacon_to_list(map_center, beacon_info, i, lat_scale, path_map);
   }
   // get beacon display locations
   calculate_destination_map_position(path_map);
   for (uint32_t i=0; i<path_map->num_beacons; i++) {
      map_beacon_reference_type *ref = &path_map->beacon_ref[i];
      ref->pos_in_map =
            calculate_map_position_apos(path_map, ref->coords);
   }
   // get beacon weights
   for (uint32_t i=0; i<path_map->num_beacons; i++) {
      uint32_t idx = path_map->beacon_ref[i].index;
//printf("Seeding %d with %.1f\n", idx, (double) beacon_weights_[idx]);
      path_map->beacon_ref[i].path_weight = beacon_weights_[idx];
   }
   // print weights
   for (uint32_t i=0; i<path_map->num_beacons; i++) {
      map_beacon_reference_type *ref = &path_map->beacon_ref[i];
printf("  %6d   wt=%.1f  at %d,%d\n", ref->index, (double) ref->path_weight, ref->pos_in_map.x, ref->pos_in_map.y);
   }
}

