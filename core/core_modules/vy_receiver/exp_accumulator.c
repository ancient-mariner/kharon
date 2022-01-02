#include "exp_accumulator.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#if !defined(D2R)
#define D2R ((float) (M_PI / 180.0))
#endif // D2R
#if !defined(R2D)
#define R2D ((float) (180.0 / M_PI))
#endif // R2D

/* 
Defines accumulators for remapping 'undistorted' camera images to
perspective views. One accumulator is used to produce an output 
image for each channel (intensity + color). Source images from each
channel have different resultions, resulting in much duplicated code.

The limiting factor for output resolution (pixels per degree) is the
lower resolution V channel. Using 12 pix/deg is about the max resolution
that can be achieved. Even at 12 p/d there may be minor upsampling in
the image center.

Input images are assumed to be fed in as YUV. Y and V channels are used.

Algorithm derivation is at end of accumulator.h
*/

////////////////////////////////////////////////////////////////////////
// accumulator logic

#define VY_COLS      820
#define VY_ROWS      616

// accumulator
// each receiver requires an accumulator, so keep this in thread storage
// TODO why not put the accumulator in vy class then?
static __thread accumulator_element_type * accum_ = NULL;

// map of image pixels to accumulator elements
// this is const once initialized, so can be shared between threads
static accumulator_coord_type * accum_map_ = NULL;

// local image buffer, storing blurred version of image that's pushed
//    to accumulator
static __thread uint8_t * blurred_img_ = NULL;

// update accumulator element with color value
static void update_element(
      /* in     */ const uint8_t v,
      /* in     */ const uint8_t w,
      /* in out */       accumulator_element_type *element
      )
{
   element->val = element->val + (uint32_t) (w * v);
   element->w = (uint16_t) (element->w + w);
}

// push top or bottom row of image to accumulator, setting border flag 
//    for nw,ne,sw,se elements
static void push_border_row(
      /* in     */ const uint8_t *buf,
      /* in     */ const uint32_t row_num
      )
{
   // get pointer to this data row
   uint32_t idx = row_num * VY_COLS;
   accumulator_coord_type *map_row =  &accum_map_[idx];
   const uint8_t *row = &buf[idx];
   // push pixels of this row to the accumulator
   for (uint32_t i=0; i<VY_COLS; i++) {
      // get pixel value and its accumulator mapping
      const uint8_t val = row[i];
      const accumulator_coord_type *coord = &map_row[i];
      // get accumulator elements
      const uint32_t top_idx = (uint32_t) (coord->x + coord->y * ACCUM_WIDTH);
      accumulator_element_type *top = &accum_[top_idx];
      accumulator_element_type *bot = &accum_[top_idx + ACCUM_WIDTH];
      // set border and push values, left edge
      top->border = 255;
      update_element(val, coord->nw, top);
      bot->border = 255;
      update_element(val, coord->sw, bot);
      // set border and push values, right edge
      top++;
      bot++;
      top->border = 255;
      update_element(val, coord->ne, top);
      bot->border = 255;
      update_element(val, coord->se, bot);
   }
}

// push non-border row of image to accumulator, setting border flag 
//    for nw,ne,sw,se elements
static void push_row(
      /* in     */ const uint8_t *buf,
      /* in     */ const uint32_t row_num
      )
{
   // get pointer to this data row
   uint32_t idx = row_num * VY_COLS;
   accumulator_coord_type *coord =  &accum_map_[idx];
   const uint8_t *row = &buf[idx];
   // push pixel value for first col, setting border flag
   {
      // get pixel value and its accumulator mapping
      const uint8_t val = *row;
      const uint32_t top_idx = (uint32_t) (coord->x + coord->y * ACCUM_WIDTH);
      accumulator_element_type *top = &accum_[top_idx];
      accumulator_element_type *bot = &accum_[top_idx + ACCUM_WIDTH];
      // set border and push values, left edge
      top->border = 255;
      update_element(val, coord->nw, top);
      bot->border = 255;
      update_element(val, coord->sw, bot);
      // set border and push values, right edge
      top++;
      bot++;
      top->border = 255;
      update_element(val, coord->ne, top);
      bot->border = 255;
      update_element(val, coord->se, bot);
      // advance to next pixel
      coord++;
      row++;
   }
   // push pixels of this row to the accumulator
   for (uint32_t i=1; i<VY_COLS-1; i++) {
      // get pixel value and its accumulator mapping
      const uint8_t val = *row;
      const uint32_t top_idx = (uint32_t) (coord->x + coord->y * ACCUM_WIDTH);
      accumulator_element_type *top = &accum_[top_idx];
      accumulator_element_type *bot = &accum_[top_idx + ACCUM_WIDTH];
      // push values, left edge
      update_element(val, coord->nw, top);
      update_element(val, coord->sw, bot);
      // push values, right edge
      update_element(val, coord->ne, &top[1]);
      update_element(val, coord->se, &bot[1]);
      // advance to next pixel
      coord++;
      row++;
   }
   // push pixel value for first col, setting border flag
   {
      // get pixel value and its accumulator mapping
      const uint8_t val = *row;
      const uint32_t top_idx = (uint32_t) (coord->x + coord->y * ACCUM_WIDTH);
      accumulator_element_type *top = &accum_[top_idx];
      accumulator_element_type *bot = &accum_[top_idx + ACCUM_WIDTH];
      // set border and push values, left edge
      top->border = 255;
      update_element(val, coord->nw, top);
      bot->border = 255;
      update_element(val, coord->sw, bot);
      // set border and push values, right edge
      top++;
      bot++;
      top->border = 255;
      update_element(val, coord->ne, top);
      bot->border = 255;
      update_element(val, coord->se, bot);
   }
}

static void blur_image(
      /* in     */ const uint8_t *buf
      )
{
   // perform 3x3 blurring kernel to image before pushing to accumulator
   // TODO WHY blur here? blurring should be done after push to accum to
   //    ensure uniform blur
   const uint32_t kern[3][3] = 
         {{ 1, 2, 1},
          { 2, 4, 2},
          { 1, 2, 1}};
   // border pixels ignored, so don't need to blur those
   for (uint32_t y=1; y<VY_ROWS-1; y++) {
      const uint32_t idx_0 = (uint32_t) ((y-1) * VY_COLS) - 1;
      for (uint32_t x=1; x<VY_COLS-1; x++) {
         uint32_t sum = 0;
         uint32_t idx = idx_0 + x;
         sum = (uint32_t) (sum + buf[idx] * kern[0][0]);
         sum = (uint32_t) (sum + buf[idx+1] * kern[0][1]);
         sum = (uint32_t) (sum + buf[idx+2] * kern[0][2]);
         //
         idx = (uint32_t) (idx + VY_COLS);
         sum = (uint32_t) (sum + buf[idx] * kern[1][0]);
         sum = (uint32_t) (sum + buf[idx+1] * kern[1][1]);
         sum = (uint32_t) (sum + buf[idx+2] * kern[1][2]);
         //
         idx = (uint32_t) (idx + VY_COLS);
         sum = (uint32_t) (sum + buf[idx] * kern[2][0]);
         sum = (uint32_t) (sum + buf[idx+1] * kern[2][1]);
         sum = (uint32_t) (sum + buf[idx+2] * kern[2][2]);
         // 
         blurred_img_[idx_0 + x + 1 + VY_COLS] = (uint8_t) (sum / 16);
      }
   }
}

// push color image to accumulator
static void push_image_to_accumulator(
      /* in     */ const uint8_t *buf
      )
{
   blur_image(buf);
   push_border_row(blurred_img_, 0);
   for (uint32_t row=1; row<VY_ROWS-1; row++) {
      push_row(blurred_img_, row);
   }
   push_border_row(blurred_img_, VY_ROWS-1);
}

// accumulator logic
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
//

static void reset_accumulator(void)
{
   uint32_t sz = ACCUM_WIDTH * ACCUM_HEIGHT * sizeof(*accum_);
   memset(accum_, 0, sz);
}

// write accumulator to supplied buffer
// NOTE: it is important that buffer is of sufficient size. it should be
//    be able to store a full perspective image
// where there is no data, or a pixel is a border, output value is 0
// otherwise, output value is on [1,255]
static void collapse_and_reset_accumulator(
      /*    out */       uint8_t *buf
      )
{
   // output image has a dynamic range of 255. one value is reserved
   //    to indicate the pixel has no content or is a border pixel
   // pixel values on [1,255]
   // border flag 0
   for (uint32_t r=0; r<PERSP_HEIGHT_SIZE; r++) {
      uint8_t *out = &buf[r * PERSP_WIDTH_SIZE];
      const uint32_t idx_acc = r * ACCUM_WIDTH;
      //const accumulator_element_type *end = &acc[idx_acc+PERSP_WIDTH_SIZE];
      for (uint32_t c=0; c<PERSP_WIDTH_SIZE; c++) {
         accumulator_element_type *el = &accum_[idx_acc + c];
         if ((el->w == 0) || (el->border)) {
            // invalid pixel. flag as 0 so downstream process can ignore
            *out++ = 0;
         } else {
            uint32_t val = el->val / el->w;
            if (val > 255) {
               printf("Internal error: normalized accum value is %d\n", val);
            }
            if (val >= 255)
               val = 254;
            *out = (uint8_t) (1 + val);
            out++;
         }
      }
   }
   // clear the accumulator when done
   reset_accumulator();
}

static void print_coord(
      /* in     */ const uint32_t x,
      /* in     */ const uint32_t y,
      /* in     */ const accumulator_coord_type *coord
      )
{
   printf("%4d,%4d      %3d,%3d     %2d %2d %2d %2d\n", x, y,
         coord->x, coord->y, coord->nw, coord->ne, coord->sw, coord->se);
}

//
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
// constructor routines

static void get_perspective_pix(
      /* in     */ const uint32_t x,
      /* in     */ const uint32_t y,
      /* in     */ const uint32_t cam_width,
      /* in     */ const uint32_t cam_height,
      /*    out */       accumulator_coord_type *pos
      )
{
//printf("%d,%d  of  %d,%d\n", x, y, cam_width, cam_height);
   // to avoid having to cast ints to floats below, making equations
   //    more difficult to read, pre-cast values
   const float cam_width_full = (float) cam_width;
   const float cam_height_full = (float) cam_height;
   const float cam_width_half = cam_width_full / 2.0f;
   const float cam_height_half = cam_height_full / 2.0f;
   //
   const float fov_height_full = (float) FOV_HEIGHT_DEGREES;
   const float fov_width_full = (float) FOV_WIDTH_DEGREES;
   const float fov_height_half = fov_height_full / 2.0f;
   const float fov_width_half = fov_width_full / 2.0f;
   // move origin to image center and get x,y in the new coordinate system
   const float centered_x = (float) x - cam_width_half;
   const float centered_y = (float) y - cam_height_half;
//printf("centered position: %f,%f\n", (double) centered_x, (double) centered_y);
   // there's a slight difference between K as calculated on X and Y
   //    axes. use the larger value (this corresponds to the FOV on
   //    one axis being slightly overstated, but going the other
   //    way results in pixels being pushed outside of the FOV on
   //    the understated axis)
   const float kx = cam_width_half / tanf(D2R * fov_width_half);
   const float ky = cam_height_half / tanf(D2R * fov_height_half);
   const float k = kx > ky ? kx : ky;
//printf("k: %f (%f, %f)\n", (double) k, (double) kx, (double) ky);
   // distance from camera to pixel in question (in units of pixels)
   const float j = sqrtf(k*k + centered_x*centered_x + centered_y*centered_y);
//printf("j: %f\n", (double) j);
   // vertical adjustment
   const float y_pos = PERSP_HEIGHT / 2.0f + 
         R2D * asinf(centered_y / j) * PERSP_HEIGHT / fov_height_full;
   const float x_pos = PERSP_WIDTH / 2.0f + 
         R2D * asinf(centered_x / j) * PERSP_WIDTH / fov_width_full;
//printf("xpos: %f\n", (double) x_pos);
//printf("ypos: %f\n", (double) y_pos);
   // values are based on left/top edge of pixel, and ne/nw/se/sw 
   //    interpolation is based on this, so rounding to nearest
   //    integer value is not appropriate
   pos->x = (uint16_t) x_pos;
   pos->y = (uint16_t) y_pos;
   uint32_t x_bin = (uint32_t) ((x_pos - (float) pos->x) * 8);
   uint32_t y_bin = (uint32_t) ((y_pos - (float) pos->y) * 8);
//printf("Bins: %d,%d\n", x_bin, y_bin);
   pos->nw = (uint8_t) ((8 - x_bin) * (8 - y_bin));
   pos->ne = (uint8_t) (x_bin * (8 - y_bin));
   pos->sw = (uint8_t) ((8 - x_bin) * y_bin);
   pos->se = (uint8_t) (x_bin * y_bin);
//exit(0);
}

static void create_accumulator(void)
{
   // create accumulator
   if (accum_ == NULL) {
      uint32_t sz_acc = ACCUM_WIDTH * ACCUM_HEIGHT * sizeof(*accum_);
      accum_ = malloc(sz_acc);
      memset(accum_, 0, sz_acc);
   }
   if (blurred_img_ == NULL) {
      uint32_t sz_img = VY_COLS * VY_ROWS * sizeof(*blurred_img_);
      blurred_img_ = malloc(sz_img);
   }
   //////////////////////////////////////
   // create map for image to accumulator
   if (accum_map_ == NULL) {
      uint32_t sz_map = VY_COLS * VY_ROWS * sizeof(*accum_map_);
      accum_map_ = malloc(sz_map);
      memset(accum_map_, 0, sz_map);
      for (uint32_t r=0; r<VY_ROWS; r++) {
         uint32_t idx = r * VY_COLS;
         for (uint32_t c=0; c<VY_COLS; c++) {
            accumulator_coord_type *coord = &accum_map_[idx++];
            get_perspective_pix(c, r, VY_COLS, VY_ROWS, coord);
//            if (r == VY_ROWS / 2) {
//               print_coord(c, r, coord);
//            }
         }
      }
   }
}


