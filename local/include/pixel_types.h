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
#if !defined(PIXEL_TYPES_H)
#define  PIXEL_TYPES_H
#include "pin_types.h"

// color channels -- presently Y and V
//#define NUM_IMAGE_CHANNELS      2

// indices for color channels
enum { V_CHAN_IDX = 0, Y_CHAN_IDX, NUM_IMAGE_CHANNELS };

// container for 2-channel (Y,V) pixels
//// NOTE: V should be stored as signed in, w/ 128 already subtracted
////    from acquired/reported value (ie, move range to [-128,128) from
////    [0,256)
// NOTE: V zero value is 128
union vy_pixel {
   struct {
      uint8_t v, y;
   };
//   struct {
//      int8_t signed_v, signed_y;
//   };
   uint8_t channel[NUM_IMAGE_CHANNELS];
//   int8_t signed_channel[NUM_IMAGE_CHANNELS];
   uint16_t all;
};
typedef union vy_pixel vy_pixel_type;

//// like vy_pixel, but with 16 bit storage
//union vy_pixel16 {
//   struct {
//      uint16_t v, y;
//   };
//   uint16_t channel[NUM_IMAGE_CHANNELS];
//   uint32_t all;
//};
//typedef union vy_pixel16 vy_pixel16_type;

// generic pixel container, as some day we may have more pixel channels
typedef vy_pixel_type pixel_color_type;

// pixel types that store y,v channels plus auxiliary info
// at the first stage of processing this stores a flag indicating
//    that the pixel is at the border and its value should be
//    discarded before edge detection
// radius is the relative distance of the pixel from the center of the
//    image. this is not necessarily a linear distance. the only
//    requirement is that pixels further from the center have
//    equal or higher values as those closer to the center
// if a pixel has a valid it should have a camera number (typically 0-7).
//    if the pixel is invalid the cam num should be 255
struct pixel_cam_info {
   pixel_color_type color;
   //pixel_color_type val;
   uint16_t radius;  // dist from image center. 0xffff if unset
   uint8_t border;   // non-zero if a border pixel
   uint8_t cam_num;  // camera number
};
typedef struct pixel_cam_info pixel_cam_info_type;
//_Static_assert(sizeof(vy_pix_f_type) == 8, "Struct should be 8 bytes");


#define PIX_FLAG_EDGE         0x01  // pixel is on edge (max local gradient)

// pixel has been aligned to by a KP
#define PIX_FLAG_ALIGNED_KP        0x02

// local salience peak
#define PIX_FLAG_LOCAL_PEAK        0x04

// salience peak that's below local salience threshold and so was suppressed
// note: this isn't necessarily used
#define PIX_FLAG_SUPPRESSED_PEAK   0x08

// approximate motion data is in available
// this should be applied to pixels (during gaze) that are local peaks
//    and that are immediately adjacent to peaks. if that assumption
//    chages, evaluate tracker code and update as necessary
#define PIX_FLAG_MOTION_DATA       0x10

#define PIX_FLAG_HAS_TARGET         0x20

// TODO V channel is squeezed due NIR data greyifying colors. magnify
//    V difference from 128

// both sides of boundary represented with one edgepoint. in most cases
//   background pixel will remain constant during viewing period.
//   if it changes, match can still be accepted if foreground (ie, other)
//      pixel stays the same
struct boundary_pixel {
   // features for KP are primarily the slope and orientation of an
   //    edge, plus a ring of colors surrounding the pixel. the following
   //    ring pattern is used (also can use R3 pattern w/ 16 values)
   //    00 01 02
   // 11 .. .. .. 03
   // 10 .. xx .. 04
   // 09 .. .. .. 05
   //    08 07 06
   // pixel values are color. for only 2 color channels, blurring makes
   //    it redundant to store values for each color at each pixel, so
   //    pixels are interleaved
   //  Y V Y V Y
   //  V       V
   //  Y   x   Y
   //  V       V
   //  Y V Y V Y
   // indices for pixels in ring_v array are idx/2 for odd idx
   // -- 00 -- 01 --
   // 07          02
   // --    xx    --
   // 06          03
   // -- 05 -- 04 --
   // indices for pixels in ring_y array are idx/2 for even idx
   // 00 -- 01 -- 02
   // --          --
   // 07    xx    03
   // --          --
   // 06 -- 05 -- 04
   // to get approximate info on the 'up' versus 'down' hill side of
   //    a slope, split analysis of ring into two components based
   //    on theta. for 16-point ring:
   //       'down' is 8 pixels starting at (((ori+64) >> 4) & 15)
   //       'up' is 8 pixels starting at (((ori+192) >> 4) & 15)
   // NOTE: it was considered using 'Bresenham circle of radius 3' (eg,
   //    see wikipedia description of FAST keypoint) but for small
   //    features a radius 2 circle looks to be more appropriate. the
   //    12-point R2 circle was expanded to 16 bits for mathematical
   //    convenience
   union {
      struct {
         uint8_t ring[16];
      };
      struct {
         uint64_t all_ring_0;
         uint64_t all_ring_1;
      };
   };
   // pixel data
   union {
      struct {
         pixel_color_type color;
         uint8_t steepness;   // slope from edge detector
         uint8_t theta;   // orientation of slope (0 north, 64 east, etc)
         uint8_t flags;
         uint8_t salience;
         uint8_t v_range;
         uint8_t unused;
// TODO delete this. it was to support panorama color layout, but that
//    was too computationally expensive for little noise-reduction benefit
//         // uniqueness of color for either side of this edge relative to
//         //    colors observed in other nearby points in scene
//         // value is based on size of distribution bin for most unique of
//         //    high/low colors. if distribution bin for most unique color
//         //    has ~10+% of all visible colors in the distribution, then
//         //    score is 255. score is reduced linearly w/ %age. eg, 5% is
//         //    128, 2.5% is 64, etc.
//         // actual value: score increased by 1 for every 0.04 %age, w/
//         //    anything less than 0.04% being 1 (there is no valid zero
//         //    score), <0.08%=2, <0.12%=3, etc
//         // 0 means unset/unknown
//         uint8_t color_score;
      };
      uint64_t all_desc;
   };
};
typedef struct boundary_pixel boundary_pixel_type;

// see comments above for color_score
#define COLOR_SCORE_PCT_STEP     0.04f

// pixel features, including salience, gradient direction,
//    gradient intensity and edge state
struct pixel_features {
   // pixel features and info
   boundary_pixel_type  edge_desc;
   /////////////////////////////////////////////////////////////
   // edgepoint for pixel
   // 0xffffffff is unassigned
   union {
      uint32_t edgepoint_idx;
      int32_t edgepoint_idx_signed;
   };
   // tracing edgepoints is done in same image frame as edgepoint
   //    center resides in, even if ring extends into background
   //    (ie, not in foreground). this is to avoid artifacts
   //    when trace spans data from different cameras
   // IMPORTANT: for this to work there must be sufficient overlap
   //    between cams on all pyramid levels
   /////////////////////////////////////////////////////////////
   // location of pixel in image frame
   image_coordinate_type pos;
};
typedef struct pixel_features pixel_features_type;


// source of foreground and background pixels for a point in space
// value stored is camera number
// fg_radius is the distance from foreground pixel to its camera's center
// fg.radius is 0xffff if unset or is border (which is equiv to unset)
// camera num < 0 means no data on that channel
struct overlap_pixel {
   pixel_cam_info_type fg;
   pixel_cam_info_type bg;
};
typedef struct overlap_pixel overlap_pixel_type;


#endif   // PIXEL_TYPES_H

