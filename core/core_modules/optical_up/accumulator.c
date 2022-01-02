#include "pin_types.h"
#include "pixel_types.h"
#include "accumulator.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "lin_alg.h"
#include "logger.h"

// dist map stores precomputed weight distributions for how a pixel
//    is spread out among neighboring accumulator elements. to ease
//    computation, each pixel is right-shifted 1/2 pixel
// pixel positions are computed in float space. these floats are
//    split along 1/8 pixel boundaries. weight distribution between
//    left/right (top/bottom) pixels is based on which bin the
//    pixel position falls in:
//       slice 0: 8 left, 0 right
//       slice 1: 7 left, 1 right
//       ...
//       slice 7: 1 left, 7 right
#include "dist_map.h"

// for vector_to_latlon, project_pixel_to_sphere (amalgamation approach)
#include "common_optical.c"

// accumulator stores pixel values as they're pushed in, as part of
//    rotation or remapping operation
// the auxiliary channel Z is used to store image border information,
//    which is used in edge and salience detection. pixels with
//    non-zero border values are discounted because of border 
//    artifacts. to prevent non-image areas from registering
//    edges at the image boundary, accumulator squares with zero
//    weight will be flagged as border areas, and so will be
//    discounted from edge/salience detection
//

static image_size_type get_upright_size(
      /* in     */ const degree_type fov_horiz,
      /* in     */ const degree_type fov_vert,
      /* in     */ const double ppd
      )
{
   image_size_type sz;
   sz.x = (uint16_t) (ppd *
         (0.866 * fov_horiz.degrees + 0.5 * fov_vert.degrees));
   sz.y = (uint16_t) (ppd *
         (0.866 * fov_vert.degrees + 0.5 * fov_horiz.degrees));
   return sz;
}

static inline void update_world_pix(
      /* in      */ const vy_accumulator_pixel_type pix,
      /* in      */ const uint32_t border,
      /* in      */ const uint32_t wt,
      /* in  out */       vy_accumulator_element_type * tgt
      )
{
//printf("     vy  %d,%d   b %d  wt %d\n", tgt->v, tgt->y, border, wt);
   tgt->y = tgt->y + pix.y * wt;
   tgt->v = tgt->v + pix.v * wt;
   tgt->z = tgt->z | border;  // input is 0 or 255 (z only matters if != 0)
//tgt->z = 0;
   tgt->w = tgt->w + wt;
}

static void push_pixel_to_accumulator(
      /* in     */ const vy_accumulator_pixel_type pix,
      /* in     */ const uint32_t border,
      /* in     */ const vector_type *sphere_pos,
      /* in     */ const degree_type img_center_lon,
      /* in     */ const degree_type img_center_lat,
      /* in     */ const double pix_per_degree_x8,
      /* in out */       vy_accumulator_type * restrict accum
      )
{
   // sphere image top left is max lat (+lat in upper hemisphere) and 
   //    least lon (lon increasing rightward)
   // image buffer top left is least x, least y. need to invert y at some
   //    point so image isn't upside down
   // accumulator uses relative coords, so center of image is
   //    at center of accumulator, need to subtract out center and add
   //    in half width|height
   degree_type pix_lon, pix_lat;
   vector_to_latlon(sphere_pos, &pix_lon, &pix_lat);
//print_vec(sphere_pos, "sphere pos");
//printf("init pix at lat=%f lon=%f\n", (double) pix_latlon.latitude, (double) pix_latlon.longitude);
   pix_lat.degrees = pix_lat.degrees - img_center_lat.degrees;
   pix_lon.degrees = pix_lon.degrees - img_center_lon.degrees;
   // push longitude to interval (-180,180]
   if (pix_lon.degrees <= -180.0) {
      pix_lon.degrees += 360.0;
   } else if (pix_lon.degrees > 180.0) {
      pix_lon.degrees -= 360.0;
   }
//printf("  gaze pix at lat=%f lon=%f\n", (double) pix_latlon.latitude, (double) pix_latlon.longitude);
   // offset so 0,0 is in center of accumulator
   pix_lat.degrees = accum->half_height.degrees - pix_lat.degrees;
   pix_lon.degrees += accum->half_width.degrees;
//printf("  offset to lat=%f lon=%f\n", (double) pix_latlon.latitude, (double) pix_latlon.longitude);
   // find accumulator bin this pixel maps to
   // ppd is multiplied by 8 to more easily find which part of 
   //    bin pixel is centered at
   uint32_t x_pos_x8 = (uint32_t) (pix_lon.degrees * pix_per_degree_x8);
   uint32_t y_pos_x8 = (uint32_t) (pix_lat.degrees * pix_per_degree_x8);
   uint32_t x_bin = x_pos_x8 >> 3;
   uint32_t y_bin = y_pos_x8 >> 3;
   // invert bin so up is down (see comment above)
   //uint32_t y_bin = accum->size.y - (y_pos_x8 >> 3) - 1;
//printf("  x,y = %d,%d   8: %d,%d\n", x_bin, y_bin, x_pos_x8, y_pos_x8);
   if ((x_bin < accum->size.x) && (y_bin < accum->size.y)) {
      // look up distribution of weights between neighboring pixels
      // invert map index for y to go along with inverting y bin
      const uint32_t map_idx = (x_pos_x8 & 0x07) + (((y_pos_x8 & 0x07)) << 3);
//      const uint32_t map_idx = (x_pos_x8 & 0x07) + ((7 - (y_pos_x8 & 0x07)) << 3);
//printf("Map idx: %d  (%d, %d)\n", map_idx, x_pos_x8 & 7, y_pos_x8 & 7);
      const struct distribution_map weight_map = map_[map_idx];
      // top row
      vy_accumulator_element_type *element;
      const uint32_t accum_idx = x_bin + y_bin * accum->size.cols;
      element = &accum->accum[accum_idx];
//printf("  idx: %d\n", accum_idx);
      update_world_pix(pix, border, weight_map.nw, element);
      update_world_pix(pix, border, weight_map.ne, element+1);
      // bottom row
      element += accum->size.cols;
//printf("  idx: %d\n", accum_idx + accum->size.cols);
      update_world_pix(pix, border, weight_map.sw, element);
      update_world_pix(pix, border, weight_map.se, element+1);
   }
}


// initialize values in accumulator to zero
static void clear_vy_accumulator(
      /* in out  */       vy_accumulator_type *accum
      )
{
   const uint32_t n_pix = (uint32_t) (accum->size.x * accum->size.y);
   vy_accumulator_element_type * elements = accum->accum;
   memset(elements, 0, n_pix * sizeof *elements);
}


// allocate a new accumulator on the heap
static vy_accumulator_type * create_vy_accumulator(
      /* in      */ const image_size_type img_sz,
      /* in      */ const degree_type size_horiz,
      /* in      */ const degree_type size_vert
      )
{
   ///////////////
   // construct accumulator
   vy_accumulator_type *accum = malloc(sizeof *accum);
   accum->size = img_sz;
   accum->half_width.degrees = size_horiz.degrees / 2.0;
   accum->half_height.degrees = size_vert.degrees / 2.0;
   //
   const uint32_t n_pix = (uint32_t) (accum->size.x * accum->size.y);
   accum->accum = malloc(n_pix * sizeof *accum->accum);
   clear_vy_accumulator(accum);
   //
   return accum;
}


////////////////////////////////////////////////////////////////////////
//--------------------------------------------------------------------//
////////////////////////////////////////////////////////////////////////

#if defined(TEST_ACCUMULATOR)

static void set_vector(vector_type *v, double x, double y, double z)
{
   v->v[0] = x;
   v->v[1] = y;
   v->v[2] = z;
}

static int32_t check_acc_pix(
      /* in     */ const vy_accumulator_type *acc,
      /* in     */ const int32_t pos_x,
      /* in     */ const int32_t pos_y,
      /* in     */ const uint32_t expected_y,
      /* in     */ const uint32_t expected_v,
      /* in     */ const uint32_t expected_z,
      /* in     */ const uint32_t expected_w
      )
{
//printf("Check pix %d,%d\n", pos_x, pos_y);
   int32_t errs = 0;
   uint32_t idx = (uint32_t) (pos_x + pos_y * acc->size.cols);
   vy_accumulator_element_type *pix = &acc->accum[idx];
   if (expected_y != pix->y)
      errs++;
   if (expected_v != pix->v)
      errs++;
   if (expected_z != pix->z)
      errs++;
   if (expected_w != pix->w)
      errs++;
   if (errs > 0) {
      printf("  %d,%d\t expected %d,%d,%d,%d, got %d,%d,%d,%d\n", pos_x, pos_y,
            expected_y, expected_v, expected_z, expected_w,
            pix->y, pix->v, pix->z, pix->w);
   }
   return errs;
}


static int32_t test_push_pixel_to_accumulator(void)
{
   // 3 test cases, one extreme
   int32_t errs = 0;
   printf("Testing push_pixel_to_accumulator\n");
   degree_type horiz = { .degrees = 60.0 };
   degree_type vert = { .degrees = 45.0 };
   double ppd = 10.0;
   double dpp = 0.1;
   image_size_type img_sz = get_upright_size(horiz, vert, ppd);
   uint16_t expected_x = 744;
   uint16_t expected_y = 689;
   if ((img_sz.x != expected_x) || (img_sz.y != expected_y)) {
      printf("  Unexpected upright size. Expected %d,%d but got %d,%d\n",
            expected_x, expected_y, img_sz.x, img_sz.y);
      errs++;
   }
   degree_type accum_width = { .degrees = dpp * (double) img_sz.x };
   degree_type accum_height = { .degrees = dpp * (double) img_sz.y };
   vy_accumulator_type *acc = create_vy_accumulator(img_sz, 
         accum_width, accum_height);
   degree_type image_center_lat = { .degrees = 0.0 };
   degree_type image_center_lon = { .degrees = 0.0 };
   vector_type vec;
   vy_accumulator_pixel_type pix = { .y=100, .v=0 };
   // should all be in desired pixel
   errs += check_acc_pix(acc, 1, 1, 0, 0, 0, 0);
   // 
   set_vector(&vec, 0.0, 0.0, 1.0);
   push_pixel_to_accumulator(pix, 0, &vec, image_center_lon, 
         image_center_lat, 80.0, acc);
   // pixel is pushed right,up half of the accumulator because it's at image
   //    center
   // center pixel content projects to same accum column, but is split
   //    between rows. this split is counter-intuitive as would have
   //    guessed it would split by column, due even number of them,
   //    but that's not the case (perhaps due half-pixel shift). close
   //    enough so not digging further
   errs += check_acc_pix(acc, img_sz.x/2, img_sz.y/2, 3200, 0, 0, 32);
   errs += check_acc_pix(acc, img_sz.x/2, img_sz.y/2+1, 3200, 0, 0, 32);
   //////////////////////////
   // shift y up by half a pixel so all content goes to upper accum bin
   // one pixel is approx 0.001745 displacement on x or y
   // NOTE: validating test requires tweaking due to small round-off 
   //    errors can put pixel in bin adjacent the one that's expected
   set_vector(&vec, 0.0f, 0.0008725f, 0.9983f);
   push_pixel_to_accumulator(pix, 0, &vec, image_center_lon, 
         image_center_lat, 80.0f, acc);
   // accumulated input is 3200 + 6400
   errs += check_acc_pix(acc, img_sz.x/2, img_sz.y/2, 9600, 0, 0, 96);
   //////////////////////////
   // move right 2 pixels to fresh accum territory
   // NOTE that sphere has standard xyz axes, w/ z forward, y up, and x left.
   //    therefore, moving right is decreasing x
   // shift y by up quarter pixel so 1/4 goes to top and 3/4 goes to bottom
   //    (with no shift, split is half and half)
   set_vector(&vec, -0.00349, 0.0004363, 0.99648);
   push_pixel_to_accumulator(pix, 0, &vec, image_center_lon, 
         image_center_lat, 80.0, acc);
   errs += check_acc_pix(acc, img_sz.x/2+2, img_sz.y/2, 4800, 0, 0, 48);
   errs += check_acc_pix(acc, img_sz.x/2+2, img_sz.y/2+1, 1600, 0, 0, 16);
   //////////////////////////
   // move right 4.125 pixels to fresh accum territory: 7/8 left and 1/8 right
   // shift y by 3/8 pixel so 1/8 goes to top and 7/8 goes to bottom
   set_vector(&vec, -0.00720, 0.000654, 0.99648);
   push_pixel_to_accumulator(pix, 0, &vec, image_center_lon, 
         image_center_lat, 80.0, acc);
   errs += check_acc_pix(acc, img_sz.x/2+4, img_sz.y/2, 4900, 0, 0, 49);
   errs += check_acc_pix(acc, img_sz.x/2+5, img_sz.y/2, 700, 0, 0, 7);
   errs += check_acc_pix(acc, img_sz.x/2+4, img_sz.y/2+1, 700, 0, 0, 7);
   errs += check_acc_pix(acc, img_sz.x/2+5, img_sz.y/2+1, 100, 0, 0, 1);
   //
   //////////////
   if (errs > 0)
      printf("    FAILED\n");
   return errs;
}


// make sure the left/top weight distributions are decreasing moving
//    right/down, and vice versa
static int32_t test_dist_map(void)
{
   int32_t errs = 0;
   printf("Testing dist map\n");
   // make sure left (west) vals are decreasing going right and
   //    right (east) are increasing
   for (uint32_t y=0; y<8; y++) {
      uint32_t idx_base = y * 8;
      const struct distribution_map start = map_[idx_base];
      uint8_t nw = start.nw;
      uint8_t ne = start.ne;
      uint8_t sw = start.sw;
      uint8_t se = start.se;
      for (uint32_t x=1; x<8; x++) {
         const struct distribution_map w2 = map_[idx_base + x];
         if ((nw < w2.nw) || (ne > w2.ne) || (sw < w2.sw) || (se > w2.se)) {
            printf("  horiz: %d,%d\n", x, y);
            printf("   %d>%d, %d<%d, %d>%d, %d<%d\n", nw, w2.nw,
                  ne, w2.ne, sw, w2.sw, se, w2.se);
            errs++;
         }
      }
   }
   // make sure top (north) vals are decreasing going down and
   //    bottom (south) are increasing
   for (uint32_t x=1; x<8; x++) {
      uint32_t idx_base = x;
      const struct distribution_map start = map_[idx_base];
      uint8_t nw = start.nw;
      uint8_t ne = start.ne;
      uint8_t sw = start.sw;
      uint8_t se = start.se;
      for (uint32_t y=0; y<8; y++) {
         const struct distribution_map w2 = map_[idx_base + y*8];
         if ((nw < w2.nw) || (ne < w2.ne) || (sw > w2.sw) || (se > w2.se)) {
            printf("  vert: %d,%d\n", x, y);
            printf("   %d<%d, %d>%d, %d<%d, %d>%d\n", nw, w2.nw,
                  ne, w2.ne, sw, w2.sw, se, w2.se);
            errs++;
         }
      }
   }
   //////////////
   if (errs > 0)
      printf("    FAILED\n");
   return errs;
}


int main(int argc, char** argv)
{
   (void) argc;
   (void) argv;
   int32_t errs = 0;
   //printf("\n---- World space ----\n");
   errs += test_dist_map();
   errs += test_push_pixel_to_accumulator();
   //
   if (errs == 0) {
      printf("--------------------\n");
      printf("--  Tests passed  --\n");
      printf("--------------------\n");
      return 0;
   } else {
      printf("---------------------------------\n");
      printf("***  One or more tests failed ***\n");
      return 1;
   }
}

#endif   // TEST_ACCUMULATOR

