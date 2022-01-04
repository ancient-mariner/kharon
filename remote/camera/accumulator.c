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
#include "accumulator.h"
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

// TODO: periodically reverse the order of these imports to help
//    reduce the possibility of copy-paste errors, and V logic being
//    included with Y and vice versa
#include "accumulator_v.c"
#include "accumulator_y.c"

////////////////////////////////////////////////////////////////////////
//

static void reset_accumulator(
      /* in out */       accumulator_element_type *acc
      )
{
   uint32_t sz = ACCUM_WIDTH * ACCUM_HEIGHT * sizeof(*acc);
   memset(acc, 0, sz);
}

// write accumulator to supplied buffer
// NOTE: it is important that buffer is of sufficient size. it should be
//    be able to store a full perspective image
// where there is no data, or a pixel is a border, output value is 0
// otherwise, output value is on [1,255]
static void collapse_and_reset_accumulator(
      /* in out */       accumulator_element_type *acc,
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
         accumulator_element_type *el = &acc[idx_acc + c];
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
   reset_accumulator(acc);
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
   const float centered_y =(float) y - cam_height_half;
//printf("Centered pos: %f,%f\n", (double) centered_x, (double) centered_y);
   float y_pos, x_pos;
   // vertical adjustment
   {
//printf("Vertical adjustment\n");
      // get longitudinal position
      const float theta = fov_width_half * centered_x / cam_width_half;
//printf("  x theta (deg): %f\n", (double) theta);
      // get max distance from horizon at this longitude for this location
      const float bound = cam_height_half * cosf(D2R * theta);
            //(centered_y < 0 ? -1 : 1);
//printf("  bound (pix): %f  (%f)\n", (double) bound, (double) cosf(D2R * theta));
      // 'distance' from camera to image plane
      const float k = cam_height_half / tanf(D2R * fov_height_half);
//printf("  k (pix): %f\n", (double) k);
      // degrees of elevation from horizon to point of view
      const float degs = R2D * atanf(centered_y / k);
//printf("  y degs (degs): %f\n", (double) degs);
//      float elevation = degs * cam_height_half / fov_height_half;
//printf("  elevation (pix): %f\n", (double) elevation);
      float vert_pos = degs / fov_height_half;
//printf("  elevation (relative): %f\n", (double) vert_pos);
      // vertical location of projected pixel
      y_pos = bound * vert_pos * PERSP_HEIGHT / cam_height_full;
//printf("  y persp pos (+/-): %f\n", (double) y_pos);
      // adjust back to positive territory
      y_pos += PERSP_HEIGHT / 2.0f;
//printf("  y persp pos (abs): %f\n", (double) y_pos);
   }
   // horizontal adjustment
   {
//printf("Horizontal adjustment\n");
      // get latidinal position
      const float theta = fov_height_half * centered_y / cam_height_half;
//printf("  y theta (deg): %f  (%f)\n", (double) theta, (double) cosf(D2R * theta));
      // get max height at this latitude
      const float bound = cam_width_half * cosf(D2R * theta);
//printf("  bound (pix): %f\n", (double) bound);
      // 'distance' from camera to image plane
      const float k = cam_width_half / tanf(D2R * fov_width_half);
//printf("  k (pix): %f\n", (double) k);
      // degrees of elevation from horizon to point of view
      const float degs = R2D * atanf(centered_x / k);
//printf("  x degs (degs): %f\n", (double) degs);
//      float azimuth = degs * cam_width_half / fov_width_half;
//printf("  azimuth (pix): %f\n", (double) azimuth);
      float horiz_pos = degs / fov_width_half;
//printf("  azimuth (relative): %f\n", (double) vert_pos);
      // horizontal location of projected pixel
      x_pos = bound * horiz_pos * PERSP_WIDTH / cam_width_full;
//printf("  x persp pos (+/-): %f\n", (double) x_pos);
      // adjust back to positive territory
      x_pos += PERSP_WIDTH / 2.0f;
//printf("  x persp pos (abs): %f\n", (double) x_pos);
   }
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

static void create_accumulator_maps(void)
{
   // intensity color channel
   uint32_t sz_y = CAM_WIDTH_PIX_Y_YUV * CAM_HEIGHT_PIX_Y *
         sizeof(*accum_map_y_);
   if (accum_map_y_ == NULL) {
      accum_map_y_ = malloc(sz_y);
   }
   memset(accum_map_y_, 0, sz_y);
   for (uint32_t r=0; r<CAM_HEIGHT_PIX_Y; r++) {
      uint32_t idx = r * CAM_WIDTH_PIX_Y_YUV;
      for (uint32_t c=0; c<CAM_WIDTH_PIX_Y; c++) {
         accumulator_coord_type *coord = &accum_map_y_[idx++];
         get_perspective_pix(c, r, CAM_WIDTH_PIX_Y, CAM_HEIGHT_PIX_Y, coord);
      }
   }
   // color channel
   uint32_t sz_v = CAM_WIDTH_PIX_V_YUV * CAM_HEIGHT_PIX_V *
         sizeof(*accum_map_v_);
   if (accum_map_v_ == NULL) {
      accum_map_v_ = malloc(sz_v);
   }
   memset(accum_map_v_, 0, sz_v);
   for (uint32_t r=0; r<CAM_HEIGHT_PIX_V; r++) {
      uint32_t idx = r * CAM_WIDTH_PIX_V_YUV;
      for (uint32_t c=0; c<CAM_WIDTH_PIX_V; c++) {
         accumulator_coord_type *coord = &accum_map_v_[idx++];
         get_perspective_pix(c, r, CAM_WIDTH_PIX_V, CAM_HEIGHT_PIX_V, coord);
//         if (r == CAM_HEIGHT_PIX_V / 2) {
//            print_coord(c, r, coord);
//         }
      }
   }
}

static void create_accumulators(void)
{
   // intensity channel
   uint32_t sz_y = ACCUM_WIDTH * ACCUM_HEIGHT * sizeof(*accum_y_);
   if (accum_y_ == NULL) {
      accum_y_ = malloc(sz_y);
   }
   memset(accum_y_, 0, sz_y);
   // color channel
   uint32_t sz_v = ACCUM_WIDTH * ACCUM_HEIGHT * sizeof(*accum_v_);
   if (accum_v_ == NULL) {
      accum_v_ = malloc(sz_v);
   }
   memset(accum_v_, 0, sz_v);
   // create maps for images to these accumulators
   create_accumulator_maps();
}


