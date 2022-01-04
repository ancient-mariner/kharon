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
#if !defined(SHARE_C)
#define SHARE_C


static void check_world_coordinate(
      /* in     */ const world_coordinate_type coord,
      /* in     */ const char *label
      )
{
   if (c_assert(coord.x_deg >= 0.0) || c_assert(coord.x_deg < 360.0) ||
         c_assert(coord.y_deg >= -90.0) || c_assert(coord.y_deg <= 90.0)) {
      fprintf(stderr, "Coordinate from '%s' has illegal value: "
            " %.5f,%.5f\n", label, coord.x_deg, coord.y_deg);
      hard_exit(__FILE__, __LINE__);
   }
}


// calculates meter offset between two world coordinates
// positive dx is rightward
// positive dy is upward
// for dx, latitude correction is performed (based on dest's latitude)
void calc_meter_offset(
      /* in     */ const world_coordinate_type src,
      /* in     */ const world_coordinate_type dest,
      /*    out */       meter_type *dx,
      /*    out */       meter_type *dy,
      /* in     */ const char *label
      )
{
   check_world_coordinate(src, label);
   check_world_coordinate(dest, label);
   double dx_deg = dest.x_deg - src.x_deg;
   if (dx_deg <= -360.0) {
      dx_deg += 360.0;
   } else if (dx_deg >= 360.0) {
      dx_deg -= 360.0;
   }
   dx->meters = dx_deg * DEG_LAT_TO_METER *
         cos(D2R * 0.5 * (src.y_deg + dest.y_deg));
   dy->meters = (dest.y_deg - src.y_deg) * DEG_LAT_TO_METER;
}


// calculates meter offset between two world coordinates
// positive dx is rightward
// positive dy is upward
// for dx, latitude correction is performed (based on dest's latitude)
static meter_type calc_distance(
      /* in     */ const world_coordinate_type src,
      /* in     */ const world_coordinate_type dest,
      /* in     */ const char *label
      )
{
   meter_type dx, dy;
   calc_meter_offset(src, dest, &dx, &dy, label);
   meter_type dist = { .meters =
         sqrt(dx.meters * dx.meters + dy.meters * dy.meters) };
   return dist;
}


// returns coordiantes that are offset by 'range' at 'heading' from 'source'
world_coordinate_type calc_offset_position(
      /* in     */ const world_coordinate_type source,
      /* in     */ const degree_type heading,
      /* in     */ const meter_type range
      )
{
   double s, c;
   sincos(D2R * heading.degrees, &s, &c);
   double dx_met = range.meters * s;
   double dy_met = range.meters * c;
   double lat_corr = cos(D2R * source.lat);
   double dx_deg = dx_met * METER_TO_DEG_LAT / lat_corr;
   double dy_deg = dy_met * METER_TO_DEG_LAT;
   world_coordinate_type dest = {
         .lon = source.lon + dx_deg, .lat = source.lat + dy_deg };
   if (dest.lon < 0.0) {
      dest.lon += 360.0;
   } else if (dest.lon >= 360.0) {
      dest.lon -= 360.0;
   }
   return dest;
}


// returns pixel location in map for specified world coordinate
// if coordinate is outside of map area then pixel's x and/or y will
//    be >= MAP_LEVEL3_SIZE
image_coordinate_type get_pix_position_in_map(
      /* in     */ const path_map_type *path_map,
      /* in     */ const world_coordinate_type pos
      )
{
   meter_type dx, dy;
   calc_meter_offset(path_map->center, pos, &dx, &dy, __func__);
   // there's an easy off-by-one error as map is digitized and the
   //    center is not at a pixel but rather the intersection of
   //    4 pixels. evaluate possibility of error (off by one pixel
   //    is off by 150m at present map resolution)
   // map is built based on floating-point offsets from map center's lat-lon
   // content that is 1 meter left of center falls in x=359 while 1 meter
   //    right is in x=360
   // convert meters to pixels. floor() will make -1.0m a -1 and +1.0m a +1
   // map-size / 2 is 360. a simple combination of these should be correct
   int32_t x_offset_pix =
         (int32_t) floor(dx.meters / path_map->node_width.meters);
   // positive dy is up, whereas here it should be down as array origin is
   //    in top-left
   int32_t y_offset_pix =
         (int32_t) floor(-dy.meters / path_map->node_height.meters);
   uint32_t x_pix = (uint32_t) ((int32_t) MAP_LEVEL3_SIZE/2 + x_offset_pix);
   uint32_t y_pix = (uint32_t) ((int32_t) MAP_LEVEL3_SIZE/2 + y_offset_pix);
//printf("POSN %d   delta %.1f,%.1f   offset %d,%d    pix %d,%d\n", beacon_num, dx.meters, dy.meters, x_offset_pix, y_offset_pix, x_pix, y_pix);
   /////////////////////////////////////////////
   image_coordinate_type map_pos;
   if ((x_pix >= MAP_LEVEL3_SIZE) || (y_pix >= MAP_LEVEL3_SIZE)) {
      // out of bounds and so invisible. set ridiculous x,y values to
      //    indicate this
      map_pos.x = 65535;
      map_pos.y = 65535;
   } else {
      map_pos.x = (uint16_t) x_pix;
      map_pos.y = (uint16_t) y_pix;
   }
   return map_pos;
}

////////////////////////////////////////////////////////////////////////
// path util(s)

// move path_map stack contents down, resetting read position to index 0
// this purges already-read elements
static void shrink_path_stack(
      /* in out */       path_map_type *path_map
      )
{
   uint32_t stack_len =
         (uint32_t) (path_map->write_idx - path_map->read_idx);
//printf("Shrinking path-map stack. Len=%d. Purging %d elements\n", stack_len, path_map->read_idx);
   log_info(log_, "Shrinking path-map stack. Len=%d. Purging %d elements",
         stack_len, path_map->read_idx);
   memmove(path_map->stack, &path_map->stack[path_map->read_idx],
         stack_len * sizeof *path_map->stack);
   path_map->read_idx = 0;
   path_map->write_idx = stack_len;
}


// given two adjacent pixels, generate bitfield indicating which direction
//    b is from a. one bit is set for either of the 8 possibile offsets
static pixel_offset_bitfield_type get_offset_mask(
      /* in     */ const image_coordinate_type a,
      /* in     */ const image_coordinate_type b
      )
{
   pixel_offset_bitfield_type offset;
   int32_t dx = a.x - b.x + 1;
   int32_t dy = a.y - b.y + 1;
   assert(dx <= 2);
   assert(dy <= 2);
   uint32_t idx = (uint32_t) (dx + dy * 3);
   switch (idx) {
      case 1:     // N
         offset.mask = 0b10000000;
         //offset.mask = 0x80;
         break;
      case 2:     // NE
         offset.mask = 0b01000000;
         //offset.mask = 0x40;
         break;
      case 5:     // E
         offset.mask = 0b00100000;
         //offset.mask = 0x20;
         break;
      case 8:     // SE
         offset.mask = 0b00010000;
         //offset.mask = 0x10;
         break;
      case 7:     // S
         offset.mask = 0b00001000;
         //offset.mask = 0x08;
         break;
      case 6:     // SW
         offset.mask = 0b00000100;
         //offset.mask = 0x04;
         break;
      case 3:     // W
         offset.mask = 0b00000010;
         //offset.mask = 0x02;
         break;
      case 0:     // NW
         offset.mask = 0b00000001;
         //offset.mask = 0x01;
         break;
      case 4:     // no offset -- fall through
      default:    // not reachable (no legal default)
         offset.mask = 0;
         break;
   }
   return offset;
}


// given two adjacent pixels, generate bitfield indicating which direction
//    b is from a. three bits are set for either of the 8 possibile offsets,
//    with the center of that field matching the above get_offset_mask()
static pixel_offset_bitfield_type get_offset_mask_wide(
      /* in     */ const image_coordinate_type a,
      /* in     */ const image_coordinate_type b
      )
{
   pixel_offset_bitfield_type offset;
   int32_t dx = a.x - b.x + 1;
   int32_t dy = a.y - b.y + 1;
   assert(dx <= 2);
   assert(dy <= 2);
   uint32_t idx = (uint32_t) (dx + dy * 3);
   switch (idx) {
      case 1:     // N
         offset.mask = 0b11000001;
         //offset.mask = BIN(1100, 0001);
         break;
      case 2:     // NE
         offset.mask = 0b11100000;
         //offset.mask = BIN(1110, 0000);
         break;
      case 5:     // E
         offset.mask = 0b01110000;
         //offset.mask = BIN(0111, 0000);
         break;
      case 8:     // SE
         offset.mask = 0b00111000;
         //offset.mask = BIN(0011, 1000);
         break;
      case 7:     // S
         offset.mask = 0b00011100;
         //offset.mask = BIN(0001, 1100);
         break;
      case 6:     // SW
         offset.mask = 0b00001110;
         //offset.mask = BIN(0000, 1110);
         break;
      case 3:     // W
         offset.mask = 0b00000111;
         //offset.mask = BIN(0000, 0111);
         break;
      case 0:     // NW
         offset.mask = 0b10000011;
         //offset.mask = BIN(1000, 0011);
         break;
      case 4:     // no offset -- fall through
      default:    // not reachable (no legal default)
         offset.mask = 0;
         break;
   }
   return offset;
}


// path util(s)
////////////////////////////////////////////////////////////////////////


#endif   // SHARE_C
