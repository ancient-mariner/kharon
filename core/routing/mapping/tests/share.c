#include "pin_types.h"
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include "routing/mapping.h"
//
//static meter_per_second_type get_default_cruise_speed_mps(void)
//{
//   meter_per_second_type s = { .mps = 5.0f };
//   return s;
//}


#include "../world_map.c"

//#include "zcommon.c"


////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

static uint32_t test_calc_meter_offset(void)
{
   uint32_t errs = 0;
   printf("Testing calc_meter_offset\n");
   /////////////////////////////////////////////////////////////////////
   world_coordinate_type a, b;
   meter_type dx, dy;
   double expected_x_met, expected_y_met;
   a.lat = 0.0;
   a.lon = 0.0;
   b.lat = 1.0;
   b.lon = 0.0;
   expected_x_met = 0.0;
   expected_y_met = METERS_PER_DEG_LAT;
   calc_meter_offset(a, b, &dx, &dy, __func__);
   if ((fabs(dx.meters - expected_x_met) > 0.1) || 
         (fabs(dy.meters - expected_y_met) > 0.1)) {
      fprintf(stderr, "A %.3f,%.3f -> %.3f,%.3f should have delta %.3f,%.3f, "
            "got %.3f,%.3f\n", a.lon, a.lat, b.lon, b.lat, 
            expected_x_met, expected_y_met, dx.meters, dy.meters);
      errs++;
   }
   expected_y_met = -METERS_PER_DEG_LAT;
   calc_meter_offset(b, a, &dx, &dy, __func__);
   if ((fabs(dx.meters - expected_x_met) > 0.1) || 
         (fabs(dy.meters - expected_y_met) > 0.1)) {
      fprintf(stderr, "B %.3f,%.3f -> %.3f,%.3f should have delta %.3f,%.3f, "
            "got %.3f,%.3f\n", b.lon, b.lat, a.lon, a.lat, 
            expected_x_met, expected_y_met, dx.meters, dy.meters);
      errs++;
   }
   /////////////////////////////
   a.lat = 0.0;
   a.lon = 0.0;
   b.lat = 0.0;
   b.lon = 1.0;
   expected_x_met = METERS_PER_DEG_LAT;
   expected_y_met = 0.0;
   calc_meter_offset(a, b, &dx, &dy, __func__);
   if ((fabs(dx.meters - expected_x_met) > 0.1) || 
         (fabs(dy.meters - expected_y_met) > 0.1)) {
      fprintf(stderr, "C %.3f,%.3f -> %.3f,%.3f should have delta %.3f,%.3f, "
            "got %.3f,%.3f\n", a.lon, a.lat, b.lon, b.lat, 
            expected_x_met, expected_y_met, dx.meters, dy.meters);
      errs++;
   }
   expected_x_met = -METERS_PER_DEG_LAT;
   calc_meter_offset(b, a, &dx, &dy, __func__);
   if ((fabs(dx.meters - expected_x_met) > 0.1) || 
         (fabs(dy.meters - expected_y_met) > 0.1)) {
      fprintf(stderr, "D %.3f,%.3f -> %.3f,%.3f should have delta %.3f,%.3f, "
            "got %.3f,%.3f\n", b.lon, b.lat, a.lon, a.lat, 
            expected_x_met, expected_y_met, dx.meters, dy.meters);
      errs++;
   }
   /////////////////////////////
   a.lat = 59.9;
   a.lon = 180.0;
   b.lat = 60.1;
   b.lon = 181.0;
   expected_x_met = 0.5 * METERS_PER_DEG_LAT;
   expected_y_met = 0.2 * METERS_PER_DEG_LAT;
   calc_meter_offset(a, b, &dx, &dy, __func__);
   if ((fabs(dx.meters - expected_x_met) > 0.1) || 
         (fabs(dy.meters - expected_y_met) > 0.1)) {
      fprintf(stderr, "E %.3f,%.3f -> %.3f,%.3f should have delta %.3f,%.3f, "
            "got %.3f,%.3f\n", a.lon, a.lat, b.lon, b.lat, 
            expected_x_met, expected_y_met, dx.meters, dy.meters);
      errs++;
   }
   /////////////////////////////////////////////////////////////////////
   if (errs == 0) {
      printf("    passed\n");
   } else {
      printf("    %d errors\n", errs);
   }
   return errs;
}

static uint32_t test_calc_offset_position(void)
{
   uint32_t errs = 0;
   printf("Testing calc_meter_offset\n");
   /////////////////////////////////////////////////////////////////////
   world_coordinate_type source = { .lat = 60.0, .lon = 270.0 };
   degree_type heading;
   meter_type range = { .meters = 60.0 * NM_TO_METERS };
   world_coordinate_type dest, expected;
   //
   heading.degrees = 0.0;
   expected.lat = 61.0;
   expected.lon = 270.0;
   dest = calc_offset_position(source, heading, range);
   if ((fabs(expected.lat - dest.lat) > 0.01) ||
         (fabs(expected.lon - dest.lon) > 0.01)) {
      fprintf(stderr, "A %.3f,%.3f moving %.1fm at %.1fdeg. Expected "
            "%.3f,%.3f, offset at %.3f,%.3f\n", source.lon, source.lat,
            range.meters, heading.degrees, expected.lon, expected.lat,
            dest.lon, dest.lat);
      errs++;
   }
   //
   heading.degrees = 90.0;
   expected.lat = 60.0;
   expected.lon = 272.0;
   dest = calc_offset_position(source, heading, range);
   if ((fabs(expected.lat - dest.lat) > 0.01) ||
         (fabs(expected.lon - dest.lon) > 0.01)) {
      fprintf(stderr, "B %.3f,%.3f moving %.1fm at %.1fdeg. Expected "
            "%.3f,%.3f, offset at %.3f,%.3f\n", source.lon, source.lat,
            range.meters, heading.degrees, expected.lon, expected.lat,
            dest.lon, dest.lat);
      errs++;
   }
   //
   heading.degrees = 225.0;
   expected.lat = 59.29;
   expected.lon = 268.586;
   dest = calc_offset_position(source, heading, range);
   if ((fabs(expected.lat - dest.lat) > 0.01) ||
         (fabs(expected.lon - dest.lon) > 0.01)) {
      fprintf(stderr, "C %.3f,%.3f moving %.1fm at %.1fdeg. Expected "
            "%.3f,%.3f, offset at %.3f,%.3f\n", source.lon, source.lat,
            range.meters, heading.degrees, expected.lon, expected.lat,
            dest.lon, dest.lat);
      errs++;
   }
   /////////////////////////////////////////////////////////////////////
   if (errs == 0) {
      printf("    passed\n");
   } else {
      printf("    %d errors\n", errs);
   }
   return errs;
}


static uint32_t check_stack_element(
      /* in     */ const path_map_type *path_map,
      /* in     */ const uint32_t idx,
      /* in     */ const uint32_t expected
      )
{
   uint32_t errs = 0;
   if (path_map->stack[idx].idx != expected) {
      fprintf(stderr, "Stack element %d should have index %d. Instead "
            "has %d\n", idx, expected, path_map->stack[idx].idx);
      errs++;
   }
   return errs;
}


static uint32_t test_shrink_path_stack(void)
{
   uint32_t errs = 0;
   printf("Testing shrink_path_stack\n");
   /////////////////////////////////////////////////////////////////////
   image_size_type size = { .x=720, .y=720 };
   uint32_t n_nodes = (uint32_t) (size.x * size.y);
   path_map_type *path_map = create_path_map(size);
   for (uint32_t i=0; i<n_nodes; i++) {
      path_map->stack[i].idx = i;
   }
   /////////////////////////////
   path_map->read_idx = 0;
   path_map->write_idx = 10;
   shrink_path_stack(path_map);
   errs += check_stack_element(path_map, 0, 0);
   /////////////////////////////
   path_map->read_idx = 5;
   path_map->write_idx = 10;
   shrink_path_stack(path_map);
   errs += check_stack_element(path_map, 0, 5);
   errs += check_stack_element(path_map, 4, 9);
   errs += check_stack_element(path_map, 5, 5);
   errs += check_stack_element(path_map, 15, 15);
   /////////////////////////////////////////////////////////////////////
   if (errs == 0) {
      printf("    passed\n");
   } else {
      printf("    %d errors\n", errs);
   }
   return errs;
}


static uint32_t check_offset_mask(
      /* in     */ const pixel_offset_type base_dir,
      /* in     */ const pixel_offset_type left,   // should overlap w/ mask
      /* in     */ const pixel_offset_type right,  // should overlap w/ mask
      /* in     */ const pixel_offset_type far_left,  // no overlap w/ mask
      /* in     */ const pixel_offset_type far_right  // no overlap w/ mask
      )
{
   uint32_t errs = 0;
   image_coordinate_type a = { .x = 10, .y = 10 };
   image_coordinate_type b;
   b.x = (uint16_t) (a.x + base_dir.dx);
   b.y = (uint16_t) (a.y + base_dir.dy);
   pixel_offset_bitfield_type base = get_offset_mask(a, b);
   //
   {
      b.x = (uint16_t) (a.x + left.dx);
      b.y = (uint16_t) (a.y + left.dy);
      pixel_offset_bitfield_type field = get_offset_mask_wide(a, b);
      if ((base.mask & field.mask) == 0) {
         fprintf(stderr, "Left bitmask for offset %d,%d (%d,%d) "
               "doesn't overlap base: 0x%02x vs 0x%02x\n", 
               base_dir.dx, base_dir.dy, left.dx, left.dy,
               base.mask, field.mask);
         errs++;
      }
   }
   {
      b.x = (uint16_t) (a.x + right.dx);
      b.y = (uint16_t) (a.y + right.dy);
      pixel_offset_bitfield_type field = get_offset_mask_wide(a, b);
      if ((base.mask & field.mask) == 0) {
         fprintf(stderr, "Right bitmask for offset %d,%d (%d,%d) "
               "doesn't overlap base: 0x%02x vs 0x%02x\n", 
               base_dir.dx, base_dir.dy, right.dx, right.dy,
               base.mask, field.mask);
         errs++;
      }
   }
   {
      b.x = (uint16_t) (a.x + far_left.dx);
      b.y = (uint16_t) (a.y + far_left.dy);
      pixel_offset_bitfield_type field = get_offset_mask_wide(a, b);
      if ((base.mask & field.mask) != 0) {
         fprintf(stderr, "Far-left bitmask for offset %d,%d (%d,%d) "
               "overlaps base: 0x%02x vs 0x%02x\n", 
               base_dir.dx, base_dir.dy, far_left.dx, far_left.dy,
               base.mask, field.mask);
         errs++;
      }
   }
   {
      b.x = (uint16_t) (a.x + far_right.dx);
      b.y = (uint16_t) (a.y + far_right.dy);
      pixel_offset_bitfield_type field = get_offset_mask_wide(a, b);
      if ((base.mask & field.mask) != 0) {
         fprintf(stderr, "Far-right bitmask for offset %d,%d (%d,%d) "
               "overlaps base: 0x%02x vs 0x%02x\n", 
               base_dir.dx, base_dir.dy, far_right.dx, far_right.dy,
               base.mask, field.mask);
printf("far right %d,%d -> %d,%d   0x%02x\n", a.x, a.y, b.x, b.y, field.mask);
printf("0x%02x  0x%02x  0x%02x\n", base.mask, field.mask, base.mask & field.mask);
         errs++;
      }
   }
   return errs;
}


static uint32_t test_get_offset_mask(void)
{
   uint32_t errs = 0;
   printf("Testing get_offset_mask\n");
   /////////////////////////////////////////////////////////////////////
   errs += check_offset_mask( OFF_N, OFF_NW, OFF_NE,  OFF_W,  OFF_E);
   errs += check_offset_mask(OFF_NE,  OFF_N,  OFF_E, OFF_NW, OFF_SE);
   errs += check_offset_mask( OFF_E, OFF_NE, OFF_SE,  OFF_N,  OFF_S);
   errs += check_offset_mask(OFF_SE,  OFF_E,  OFF_S, OFF_NE, OFF_SW);
   errs += check_offset_mask( OFF_S, OFF_SE, OFF_SW,  OFF_E,  OFF_W);
   errs += check_offset_mask(OFF_SW,  OFF_S,  OFF_W, OFF_SE, OFF_NW);
   errs += check_offset_mask( OFF_W, OFF_SW, OFF_NW,  OFF_S,  OFF_N);
   errs += check_offset_mask(OFF_NW,  OFF_W,  OFF_N, OFF_SW, OFF_NE);
   /////////////////////////////////////////////////////////////////////
   if (errs == 0) {
      printf("    passed\n");
   } else {
      printf("    %d errors\n", errs);
   }
   return errs;
}


////////////////////////////////////////////////////////////////////////


int main(int argc, char** argv) {
   (void) argc;
   (void) argv;
   set_log_dir_string("/tmp/");
   //
   uint32_t errs = 0;
   errs += test_calc_meter_offset();
   errs += test_calc_offset_position();
   errs += test_shrink_path_stack();
   errs += test_get_offset_mask();
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
