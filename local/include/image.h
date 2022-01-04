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
#if !defined(IMAGE_H)
#define IMAGE_H
#include "pin_types.h"
//#include <stdint.h>


struct rgb_pix_type {
   uint8_t r, g, b;
};
typedef struct rgb_pix_type rgb_pix_type;
_Static_assert(sizeof(rgb_pix_type) <= 4, "foo");

// overloaded structure that stores different color representations:
//    r,g,b,a
//    y,u,v  [,z]
//    h,s,v  [,z]
union pix_type {
   struct { uint8_t r, g, b, a; };   // RGBA
   struct {
      union {
         struct { uint8_t y, u; };
         struct { uint8_t h, s; };
      };
      uint8_t v, z;
   };
   uint32_t rgba;
   uint32_t yuvz;
   uint32_t hsvz;
};
typedef union pix_type pix_type;

struct rgb_image {
   image_size_type   size;
   uint8_t * gray;
   pix_type *rgb;
};
typedef struct rgb_image image_type;

// creates image from specified pnm or yuv file. returns NULL on failure
image_type * create_image(const char *filename);
image_type * create_image_pgm(const char *filename);
image_type * create_image_pnm(const char *filename);


///////////////////////////////////////////////////////////////////////////////

// TODO test this before first using it
// convert between image_coordinate_type representing actual resolution
//    and the dimensions of image stored as YUV
// for example of algorithm in action, see camera_stream.c
#define TO_YUV_DIM(z)  do { z.w=(z.w+31)&~31; z.h=(z.h+15)&~15; } while (0);

void convert_yuv_to_rgb(image_type * restrict img);

// release memory from created image
void free_image(image_type *img);

int write_pnm_file(
      /* in     */ const char *filename,
      /* in     */ const image_size_type size,
      /* in     */ const pix_type *restrict rgb);

int write_pnm_file_by_channel(const char *filename, image_size_type size, uint8_t * restrict r, uint8_t * restrict g, uint8_t * restrict b);

// allocates storage for image
image_type* raw_create_image(image_size_type size);

uint8_t clamp255(double x);

#endif   // IMAGE_H

