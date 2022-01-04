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
static map_level1_type *level1_map_ = NULL;
static map_level2_type **level2_maps_ = NULL;
static map_level3_type **level3_maps_ = NULL;
static uint32_t num_level3_ = 0;

// keep track of when higher-level map values are modified
static uint32_t level1_modified_ = 0;
static uint32_t *level2_modified_ = NULL;
static uint32_t no_submap_[180*360] = { 0 };

static char map_dir_[BUF_LEN] = { 0 };

#define MAX_OPEN_MAPS   4000

////////////////////////////////////////////////////////////////////////
// saving maps (new level 3 and modified level 1,2)

static void check_resources(
      /* in     */ const int32_t force    // !=0 to force a save
      )
{
printf("Check resources: %d maps\n", num_level3_);
   // if over X number of level3 maps are open (eg, 500) then save all
   //    maps and reset
   if ((num_level3_ > MAX_OPEN_MAPS) || (force != 0)) {
      //////////////
      // save levels 2 and 3
      for (uint32_t y=0; y<180; y++) {
         for (uint32_t x=0; x<360; x++) {
            uint32_t world_idx = (uint32_t) (x + y * 360);
            map_grid_num_type grid_pos;
            grid_pos.akn_x = (uint16_t) x;
            grid_pos.akn_y = (uint16_t) y;
            // save all level 3 and free map memory
            map_level3_type *map3 = level3_maps_[world_idx];
            if (map3 != NULL) {
               write_map_level3(map_dir_, grid_pos, map3);
               free(map3);
            }
            // save modified level 2 and free map memory
            map_level2_type *map2 = level2_maps_[world_idx];
            if (level2_modified_[world_idx] != 0) {
               assert(map2 != NULL);
               write_map_level2(map_dir_, grid_pos, map2);
            }
            if (map2 != NULL) {
               free(map2);
            }
         }
      }
      memset(level2_maps_, 0, 180*360*sizeof *level2_maps_);
      memset(level2_modified_, 0, 180*360*sizeof *level2_modified_);
      memset(level3_maps_, 0, 180*360*sizeof *level3_maps_);
      num_level3_ = 0;
      //////////////
      // save level 1 if it's been modified
      if (level1_modified_ != 0) {
         write_map_level1(map_dir_, level1_map_);
         level1_modified_ = 0;
      }
   }
}

// saving maps (new level 3 and modified level 1,2)
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
// loading maps

static map_level1_square_type * get_square1(
      /* in     */ const map_grid_num_type grid_pos
      )
{
   uint32_t world_idx = (uint32_t) (grid_pos.akn_x + grid_pos.akn_y * 360);
   return &level1_map_->grid[world_idx];
}


static map_level2_square_type * get_square2(
      /* in     */ const map_grid_num_type grid_pos,
      /* in     */ const map_subgrid_pos_type sub_pos
      )
{
   uint32_t world_idx = (uint32_t) (grid_pos.akn_x + grid_pos.akn_y * 360);
   map_level2_type *map2 = level2_maps_[world_idx];
   map_level2_square_type *square2 = NULL;
   if (map2 == NULL) {
      // map not loaded -- load if it exists
      map_level1_square_type *square1 = &level1_map_->grid[world_idx];
      if (square1->flags & MAP_FLAG_LEVEL_2) {
         // load existing map. function exit()s if map doesn't exist, so
         //    don't need error checking here
         map2 = calloc(1, sizeof *map2);
//         map2 = load_map_level2(map_dir_, grid_pos, NULL);
         assert(load_map_level2(map_dir_, grid_pos, map2) != NULL);
         level2_maps_[world_idx] = map2;
      }
   }
   //
   if (map2 != NULL) {
      uint32_t grid_idx = (uint32_t) (240.0 * sub_pos.sub_x) +
            (uint32_t) (240.0 * sub_pos.sub_y) * 240u;
      square2 = &map2->grid[grid_idx];
   }
   return square2;
}

// loads level 3 map. if map doesn't exist, a new one is created
static map_level3_type * load_or_create_level3(
      /* in     */ const map_grid_num_type grid_pos
      )
{
   uint32_t world_idx = (uint32_t) (grid_pos.akn_x + grid_pos.akn_y * 360);
   map_level3_type *map3 = level3_maps_[world_idx];
   if (map3 == NULL) {
      // create empty storage
      map3 = malloc(sizeof *map3);
      // map not in memory -- create or load
      // see if we can load it from disk or if we have to make a new one
      map_level1_square_type *square1 = &level1_map_->grid[world_idx];
      if (square1->flags & MAP_FLAG_LEVEL_3) {
         // load existing map
         if (load_map_level3(map_dir_, grid_pos, map3) == NULL) {
            fprintf(stderr, "Error loading level 3 map\n");
            assert(1 == 0);
         }
      } else {
         // create empty one
         memset(map3, -1, sizeof *map3);
         square1->flags |= MAP_FLAG_LEVEL_3;
      }
      level3_maps_[world_idx] = map3;
      num_level3_++;
   }
   return map3;
}


// returns level3_square for specified position
// if map is in memory, that's used. if not, it's loaded from disk. if
//    there's no file, an empty map is created
static map_level3_square_type * get_square3(
      /* in     */ const map_grid_num_type grid_pos,
      /* in     */ const map_subgrid_pos_type sub_pos
      )
{
   map_level3_type *map3 = load_or_create_level3(grid_pos);
   assert(map3 != NULL);
   uint32_t grid_idx = (uint32_t) (720.0 * sub_pos.sub_x) +
         (uint32_t) (720.0 * sub_pos.sub_y) * 720u;
   map_level3_square_type *square3 = &map3->grid[grid_idx];
   return square3;
}

// loading maps
////////////////////////////////////////////////////////////////////////
