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
#define _GNU_SOURCE
#include "pin_types.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include "lin_alg.h"


////////////////////////////////////////////////////////////////////////
// downsample
//
// TODO test presently is only on border pixels. make sure this doesn't
//    result in artifacts at border of image if camera points outside FOV,
//    or edge artifacts more generally
// TODO store c0,c1,... in union and use SSE to bulk-check values
//


// blur first/last line of image
static void blur_top_row(
      /* in     */ const uint8_t *restrict r0,
      /* in     */ const uint8_t *restrict r1,
      /* in     */ const uint16_t img_w,
      /* in     */ const uint16_t buf_w,
      /*    out */ uint8_t *restrict y2)
{
   // first col
   {
      const uint32_t sum = (uint32_t) (
            4*r0[0] + 2*r0[1] +
            2*r1[0] + 1*r1[1]
            );
      *y2++ = (uint8_t) ((sum + 4) / 9);   // round to nearest
   }
   // remaining cols
   for (uint32_t x=2; x<img_w; x+=2) {
      const uint32_t c0 = x-1;
      const uint32_t c1 = x;
      const uint32_t c2 = x+1;
      const uint32_t sum = (uint32_t) (
            2*r0[c0] + 4*r0[c1] + 2*r0[c2] +
            1*r1[c0] + 2*r1[c1] + 1*r1[c2]
            );
      *y2++ = (uint8_t) ((sum + 6) / 12);   // round to nearest
   }
}

// blur center row in image
static void blur_body(
      /* in     */ const uint8_t *restrict r0,
      /* in     */ const uint8_t *restrict r1,
      /* in     */ const uint8_t *restrict r2,
      /* in     */ const uint16_t img_w,
      /* in     */ const uint16_t buf_w,
      /*    out */ uint8_t *restrict y2)
{
   // first col
   {
      const uint32_t sum = (uint32_t) (
            2*r0[0] + 1*r0[1] +
            4*r1[0] + 2*r1[1] +
            2*r2[0] + 1*r2[1]
            );
      *y2++ = (uint8_t) ((sum+ 6) / 12);
   }
   // center cols
   for (uint32_t x=2; x<img_w; x+=2) {
      const uint32_t c0 = (uint32_t) (x-1);
      const uint32_t c1 = (uint32_t) (x);
      const uint32_t c2 = (uint32_t) (x+1);
      const uint32_t sum = (uint32_t) (
            1*r0[c0] + 2*r0[c1] + 1*r0[c2] +
            2*r1[c0] + 4*r1[c1] + 2*r1[c2] +
            1*r2[c0] + 2*r2[c1] + 1*r2[c2]
            );
      *y2++ = (uint8_t) ((sum + 8) / 16);
   }
}


// blur y, downsample it, and store results in y2
void downsample(
      /* in     */ const uint8_t *restrict img,
      /* in     */ const image_size_type img_size,
      /* in     */ const image_size_type down_size,
      /*    out */ uint8_t *restrict down)
{
   downsample(img, img_size, down_size, down);
}

// blur y, downsample it, and store results in y2
void downsample_33(
      /* in     */ const uint8_t *restrict img,
      /* in     */ const image_size_type img_size,
      /* in     */ const image_size_type down_size,
      /*    out */ uint8_t *restrict down)
{
   const uint8_t *r0, *r1, *r2;
   const uint16_t down_w = down_size.cols;
   const uint16_t img_w = img_size.cols;
   const uint16_t img_h = img_size.rows;
   uint8_t *buf = down;
   // computing first output row -- use top 3 input rows
   r0 = img;
   r1 = img + img_w;
   blur_top_row(r0, r1, img_w, down_w, buf);
   buf += down_w;
   // computing center output rows -- use input rows -1, 0, 1, 2
   for (uint32_t r=2; r<img_h; r+=2) {
      r0 = &img[(r-1)*img_w];
      r1 = &img[(r  )*img_w];
      r2 = &img[(r+1)*img_w];
      blur_body(r0, r1, r2, img_w, down_w, buf);
      buf += down_w;
   }
   // safety check
   if (buf != (&down[down_size.cols * down_size.rows])) {
      fprintf(stderr, "Internal error: downsample output buffer\n");
      fprintf(stderr, "\tNumber of bytes in buffer: %d\n",
            down_size.x * down_size.y);
      fprintf(stderr, "\tNumber of bytes written: %d\n",
            (uint32_t) (buf - down));
      exit(1);
   }
   // TODO explain why blur_bottom_row() not necessary
}


//////////////////////////////////////////////////////////////////////////
////
//
//// downsample 2D array 'orig' to 'down'
//// each downsmple pixel is weighed average of pixels in original
////    array equivalent to if original array was convolved with
////    Gaussian-like kernel.
//// if original array were convolved with the 5x5 Gaussian kernel:
////
////   1   4   7   4   1
////   4  16  26  16   4
////   7  26  41  26   7
////   4  16  26  16   4
////   1   4   7   4   1
////
////   divided by 273
////
//// then the downsampled values would be the values in the original
////    array weighted by the following:
////
////    1   5  11  11   5   1
////    5  25  53  53  25   5
////   11  53 109 109  53  11
////   11  53 109 109  53  11
////    5  25  53  53  25   5
////    1   5  11  11   5   1
////
////    divided by 1092
////
//// with the 109 values corresponding to the pixels in the original space
////    that reduce down to a single pixel in the downsampled space
////
//// this kernel can be simplified to:
////
////    1 2 2 1
////    2 4 4 2
////    2 4 4 2
////    1 2 2 1
////
////    divided by 36
////
//// which can be further simplifed to:
////
////      1 1
////    1 2 2 1
////    1 2 2 1
////      1 1
////
////    divided by 16
////
//// image test shows this approach to induce too much blurring in downsampled
////    images. switching to a higher center-weighted kernel improves this
//// (comparison made between gimp-based downsample and iterative calls to downsample)
////
////      1 1
////    1 4 4 1
////    1 4 4 1
////      1 1
////
////    divided by 24
//
//////////////////////////////////////////////////////////////////////////
//// downsample
////
//// if PIXEL_CARRIES_NULL is set then if any of the pixels that contribute
////    to a mask are zero (null) then the blurred pixel is also zero.
////
//// TODO test presently is only on border pixels. make sure this doesn't
////    result in artifacts at border of image if camera points outside FOV,
////    or edge artifacts more generally
//// TODO store c0,c1,... in union and use SSE to bulk-check values
////
//
//#if !defined(OVERRIDE_PIXEL_CARRY_NULL)
//#define PIXEL_CARRIES_NULL    1
//#else
//#if !defined(LIN_ALG_TEST)
//// if not in test mode, print a compile warning that carry null behavior is disabled
//#warning "Overriding pixel carry null"
//#endif   // LIN_ALG_TEST
//#endif // OVERRIDE_PIXEL_CARRY_NULL
//
//// blur first/last line of image
//static void blur_line3(
//      /* in     */ const uint8_t *restrict r0,
//      /* in     */ const uint8_t *restrict r1,
//      /* in     */ const uint8_t *restrict r2,
//      /* in     */ const uint16_t img_w,
//      /* in     */ const uint16_t buf_w,
//      /*    out */ uint8_t *restrict y2)
//{
//   uint32_t sum;
//   uint32_t c0, c1, c2, c3;
//   // first col
//   {
//#if defined(PIXEL_CARRIES_NULL)
//      if (!r0[2] || !r1[2] || !r2[0] || !r2[1]) {
//         *y2++ = 0;
//      } else {
//#endif   // PIXEL_CARRIES_NULL
//         sum = (uint32_t) (
//               4*r0[0] + 4*r0[1] + r0[2] +
//               4*r1[0] + 4*r1[1] + r1[2] +
//                 r2[0] +   r2[1]);
//         *y2++ = (uint8_t) ((sum + 10) / 20);   // round to nearest
//#if defined(PIXEL_CARRIES_NULL)
//      }
//#endif   // PIXEL_CARRIES_NULL
//   }
//   // center cols
//   {
//      for (uint32_t x=2; x<img_w-2; x+=2) {
//         c0 = x-1;
//         c1 = x;
//         c2 = x+1;
//         c3 = x+2;
//         uint8_t r0c0 = r0[c0];
//         uint8_t r1c0 = r1[c0];
//         uint8_t r0c3 = r0[c3];
//         uint8_t r1c3 = r1[c3];
//         uint8_t r2c1 = r2[c1];
//         uint8_t r2c2 = r2[c2];
//#if defined(PIXEL_CARRIES_NULL)
//         if (!r0c0 || !r1c0 || !r0c3 || !r1c3 || !r2c1 || !r2c2) {
//            *y2++ = 0;
//         } else {
//#endif   // PIXEL_CARRIES_NULL
//            sum = (uint32_t) (
//                  r0c0   + 4*r0[c1] + 4*r0[c2] + r0c3   +
//                  r1c0   + 4*r1[c1] + 4*r1[c2] + r1c3   +
//                             r2c1   +   r2c2);
//            *y2++ = (uint8_t) ((sum + 11) / 22);   // round to nearest
//#if defined(PIXEL_CARRIES_NULL)
//         }
//#endif   // PIXEL_CARRIES_NULL
//      }
//   }
//   // last col
//   {
//      c0 = (uint32_t) (img_w-3);
//      c1 = (uint32_t) (img_w-2);
//      c2 = (uint32_t) (img_w-1);
//      uint8_t r0c0 = r0[c0];
//      uint8_t r1c0 = r1[c0];
//      uint8_t r2c1 = r2[c1];
//      uint8_t r2c2 = r2[c2];
//#if defined(PIXEL_CARRIES_NULL)
//      if (!r0c0 || !r1c0 || !r2c1 || !r2c2) {
//         *y2++ = 0;
//      } else {
//#endif   // PIXEL_CARRIES_NULL
//         sum = (uint32_t) (
//               r0c0   + 4*r0[c1] + 4*r0[c2] +
//               r1c0   + 4*r1[c1] + 4*r1[c2] +
//                          r2c1   +   r2c2);
//         *y2++ = (uint8_t) ((sum + 10) / 20);   // round to nearest
//#if defined(PIXEL_CARRIES_NULL)
//      }
//#endif   // PIXEL_CARRIES_NULL
//   }
//}
//
//// blur center row in image
//static void blur_line4(
//      /* in     */ const uint8_t *restrict r0,
//      /* in     */ const uint8_t *restrict r1,
//      /* in     */ const uint8_t *restrict r2,
//      /* in     */ const uint8_t *restrict r3,
//      /* in     */ const uint16_t img_w,
//      /* in     */ const uint16_t buf_w,
//      /*    out */ uint8_t *restrict y2)
//{
//   uint32_t sum;
//   uint32_t c0, c1, c2, c3;
//   // first col
//   {
//#if defined(PIXEL_CARRIES_NULL)
//      if (!r0[0] || !r0[1] || !r1[2] || !r2[2] || !r2[0] || !r2[1]) {
//         *y2++ = 0;
//      } else {
//#endif   // PIXEL_CARRIES_NULL
//         sum = (uint32_t) (
//                 r0[0] +   r0[1] +
//               4*r1[0] + 4*r1[1] + r1[2] +
//               4*r2[0] + 4*r2[1] + r2[2] +
//                 r3[0] +   r3[1]);
//         *y2++ = (uint8_t) ((sum+ 11) / 22);
//#if defined(PIXEL_CARRIES_NULL)
//      }
//#endif   // PIXEL_CARRIES_NULL
//   }
//   // center cols
//   {
//      for (uint32_t x=2; x<img_w-2; x+=2) {
//         c0 = (uint32_t) (x-1);
//         c1 = (uint32_t) (x);
//         c2 = (uint32_t) (x+1);
//         c3 = (uint32_t) (x+2);
//         uint8_t r0c1 = r0[c1];
//         uint8_t r0c2 = r0[c2];
//         uint8_t r1c0 = r1[c0];
//         uint8_t r2c0 = r2[c0];
//         uint8_t r1c3 = r1[c3];
//         uint8_t r2c3 = r2[c3];
//         uint8_t r3c1 = r3[c1];
//         uint8_t r3c2 = r3[c2];
//#if defined(PIXEL_CARRIES_NULL)
//         if (!r0c1 || !r0c2 || !r1c0 || !r2c0 || !r1c3 || !r2c3 || !r3c1 || !r3c2) {
//            *y2++ = 0;
//         } else {
//#endif   // PIXEL_CARRIES_NULL
//               sum = (uint32_t) (
//                              r0c1   +   r0c2 +
//                     r1c0   + 4*r1[c1] + 4*r1[c2] + r1c3   +
//                     r2c0   + 4*r2[c1] + 4*r2[c2] + r2c3   +
//                                r3c1   +   r3c2);
//               *y2++ = (uint8_t) ((sum + 12) / 24);
//         }
//#if defined(PIXEL_CARRIES_NULL)
//      }
//#endif   // PIXEL_CARRIES_NULL
//   }
//   // last col
//   {
//      c0 = (uint32_t) (img_w-3);
//      c1 = (uint32_t) (img_w-2);
//      c2 = (uint32_t) (img_w-1);
//      uint8_t r0c1 = r0[c1];
//      uint8_t r0c2 = r0[c2];
//      uint8_t r1c0 = r1[c0];
//      uint8_t r2c0 = r2[c0];
//      uint8_t r3c1 = r3[c1];
//      uint8_t r3c2 = r3[c2];
//#if defined(PIXEL_CARRIES_NULL)
//      if (!r0c1 || !r0c2 || !r1c0 || !r2c0 || !r3c1 || !r3c2) {
//         *y2++ = 0;
//      } else {
//#endif   // PIXEL_CARRIES_NULL
//         sum = (uint32_t) (
//                          r0c1   +   r0c2 +
//               r1c0   + 4*r1[c1] + 4*r1[c2] +
//               r2c0   + 4*r2[c1] + 4*r2[c2] +
//                          r3c1   +   r3c2);
//         *y2++ = (uint8_t) ((sum + 11) / 22);
//#if defined(PIXEL_CARRIES_NULL)
//      }
//#endif   // PIXEL_CARRIES_NULL
//   }
//}
//
//// TODO switch to using 3x3 mask
//
//// blur y, downsample it, and store results in y2
//void downsample(
//      /* in     */ const uint8_t *restrict img,
//      /* in     */ const image_size_type img_size,
//      /* in     */ const image_size_type down_size,
//      /*    out */ uint8_t *restrict down)
//{
//   const uint8_t *r0, *r1, *r2, *r3;
//   const uint16_t down_w = down_size.cols;
//   const uint16_t img_w = img_size.cols;
//   const uint16_t img_h = img_size.rows;
//   uint8_t *buf = down;
//   // computing first output row -- use top 3 input rows
//   r0 = img;
//   r1 = img + img_w;
//   r2 = img + 2*img_w;
//   blur_line3(r0, r1, r2, img_w, down_w, buf);
//   buf += down_w;
//   // computing center output rows -- use input rows -1, 0, 1, 2
//   for (uint32_t r=2; r<img_h-3; r+=2) {
//      r0 = &img[(r-1)*img_w];
//      r1 = &img[(r  )*img_w];
//      r2 = &img[(r+1)*img_w];
//      r3 = &img[(r+2)*img_w];
//      blur_line4(r0, r1, r2, r3, img_w, down_w, buf);
//      buf += down_w;
//   }
//   //
//   // compute last output row -- use bottom 3 rows
//   // put mask upside down so can reuse calculation code for first row
//   r0 = img + (img_h-3)*img_w;
//   r1 = img + (img_h-2)*img_w;
//   r2 = img + (img_h-1)*img_w;
//   blur_line3(r2, r1, r0, img_w, down_w, buf);
//   // safety check
//#if !defined(NDEBUG)
//   buf += down_w;
//   if (buf != (&down[down_size.cols * down_size.rows])) {
//      fprintf(stderr, "Internal error: downsample output buffer\n");
//      fprintf(stderr, "\tNumber of bytes in buffer: %d\n", down_size.x * down_size.y);
//      fprintf(stderr, "\tNumber of bytes written: %d\n", (uint32_t) (buf - down));
//      exit(1);
//   }
//#endif   //NDEBUG
//}

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
#if defined TEST_DOWNSAMPLE
static uint32_t downsample_content(void);
static uint32_t downsample_background(void);

////////////////////////////////////////////////////////////////
// 4x4 downsample
// this has been superceded by 3x3 downsample, but keep the code
//    around in case it's needed again
//
//static uint32_t downsample_content(void);
//static uint32_t downsample_background(void);
//const uint8_t img[] = {
//   1,    1,    1,    1,    1,  128,    1,    1,
//   1,  128,  128,    1,   64,   64,    1,    1,
//   1,    1,    1,    1,   64,   64,    1,    1,
//   1,    1,    1,    1,    1,  128,  128,  128,
//   1,    1,  128,  128,    1,  128,  128,    1,
//   1,    1,  128,  128,    1,   64,    1,    1,
//   1,    1,    1,    1,    1,   64,    1,    1,
//   0,    1,    1,    0,    1,   64,    1,    1
//};
//
//const uint8_t img_bg[] = {
//   0,  0,  0,  0,  0,  0,  0,  0,
//   0,  0,  0,  0,  0,  0,  0,  0,
//   0,  0,  1,  1,  1,  0,  0,  0,
//   0,  1,  1,  1,  1,  1,  1,  1,
//   1,  1,  1,  1,  1,  1,  0,  0,
//   0,  0,  1,  1,  1,  1,  0,  0,
//   0,  0,  0,  1,  1,  1,  1,  1,
//   0,  0,  0,  1,  1,  1,  1,  1
//};
//
//#if defined(PIXEL_CARRIES_NULL)
//
//const uint8_t expected[] = {
//  33,   33,   53,   11,
//   7,   20,   59,   62,
//  13,   86,   56,   44,
//   1,   12,    0,    7
//};
//
//const uint8_t expected_bg[] = {
//  0,   0,   0,   0,
//  0,   0,   0,   0,
//  0,   0,   0,   0,
//  0,   0,   1,   0
//};
//
//#else
//
//// NOTE: suspected bug still in computation because there is a slight increase
////    in total response after downsample, whereas this should remain constant
////    (e.g., with center multiple of 4, original has 1984->496, while output
////    has 505. with center multiple of 2, output has total of 519)
//const uint8_t expected[] = {
//  33,   33,   53,   11,
//   7,   20,   59,   62,
//  13,   86,   56,   44,
//   1,   12,   27,    7
//};
//
//const uint8_t expected_bg[] = {
//  0,   0,   0,   0,
//  0,   1,   1,   0,
//  1,   1,   1,   0,
//  0,   1,   1,   1
//};
//
//#endif // PIXEL_CARRIES_NULL
//
//
//
//static uint32_t downsample_content()
//{
//   if (1 == 0)
//      file_test();
//   ////////
//   printf("Checking downsample (content)\n");
//   uint32_t errs = 0;
//   image_size_type img_size = { .x=8, .y=8 };
//   image_size_type down_size = { .x=4, .y=4 };
//   uint8_t buf[16];
//   downsample(img, img_size, down_size, buf);
//   for (uint32_t r=0; r<down_size.rows; r++) {
//      for (uint32_t c=0; c<down_size.cols; c++) {
//         uint32_t idx = c + r * down_size.cols;
//         if (buf[idx] != expected[idx]) {
//            fprintf(stderr, "Error: %d,%d has unexpected value\n", c, r);
//            errs++;
//         }
//      }
//   }
//   if (errs > 0) {
//      printf("--------------------------------------------------\n");
//      // print source
//      printf("Original:\n");
//      uint32_t sum = 0;
//      for (uint32_t r=0; r<img_size.rows; r++) {
//         for (uint32_t c=0; c<img_size.cols; c++) {
//            uint32_t idx = c + r * img_size.cols;
//            printf("%4d", img[idx]);
//            sum += img[idx];
//         }
//         printf("\n");
//      }
//      printf("total: %d\n", sum);
//      // print output
//      sum = 0;
//      printf("Donwsample:\n");
//      for (uint32_t r=0; r<down_size.rows; r++) {
//         for (uint32_t c=0; c<down_size.cols; c++) {
//            uint32_t idx = c + r * down_size.cols;
//            printf("%4d", buf[idx]);
//            sum += buf[idx];
//         }
//         printf("\n");
//      }
//      printf("total: %d\n", sum);
//      // print expected
//      sum = 0;
//      printf("Expected:\n");
//      for (uint32_t r=0; r<down_size.rows; r++) {
//         for (uint32_t c=0; c<down_size.cols; c++) {
//            uint32_t idx = c + r * down_size.cols;
//            printf("%4d", expected[idx]);
//            sum += expected[idx];
//         }
//         printf("\n");
//      }
//      printf("total: %d\n", sum);
//   }
//   return errs;
//}
//
//
//static uint32_t downsample_background()
//{
//   if (1 == 0)
//      file_test();
//   ////////
//   printf("Checking downsample (background)\n");
//   uint32_t errs = 0;
//   image_size_type img_size = { .x=8, .y=8 };
//   image_size_type down_size = { .x=4, .y=4 };
//   uint8_t buf[16];
//   downsample(img_bg, img_size, down_size, buf);
//   for (uint32_t r=0; r<down_size.rows; r++) {
//      for (uint32_t c=0; c<down_size.cols; c++) {
//         uint32_t idx = c + r * down_size.cols;
//         if (buf[idx] != expected_bg[idx]) {
//            fprintf(stderr, "Error: %d,%d has unexpected value\n", c, r);
//            errs++;
//         }
//      }
//   }
//   if (errs > 0) {
//      printf("--------------------------------------------------\n");
//      // print source
//      printf("Original:\n");
//      uint32_t sum = 0;
//      for (uint32_t r=0; r<img_size.rows; r++) {
//         for (uint32_t c=0; c<img_size.cols; c++) {
//            uint32_t idx = c + r * img_size.cols;
//            printf("%4d", img_bg[idx]);
//            sum += img_bg[idx];
//         }
//         printf("\n");
//      }
//      printf("total: %d\n", sum);
//      // print output
//      sum = 0;
//      printf("Donwsample:\n");
//      for (uint32_t r=0; r<down_size.rows; r++) {
//         for (uint32_t c=0; c<down_size.cols; c++) {
//            uint32_t idx = c + r * down_size.cols;
//            printf("%4d", buf[idx]);
//            sum += buf[idx];
//         }
//         printf("\n");
//      }
//      printf("total: %d\n", sum);
//      // print expected
//      sum = 0;
//      printf("Expected:\n");
//      for (uint32_t r=0; r<down_size.rows; r++) {
//         for (uint32_t c=0; c<down_size.cols; c++) {
//            uint32_t idx = c + r * down_size.cols;
//            printf("%4d", expected_bg[idx]);
//            sum += expected_bg[idx];
//         }
//         printf("\n");
//      }
//      printf("total: %d\n", sum);
//   }
//   return errs;
//}



const uint8_t img[] = {
   1,    1,    1,    1,    1,  128,    1,    1,
   1,  128,  128,    1,   64,   64,    1,    1,
   1,    1,    1,    1,   64,   64,    1,    1,
   1,    1,    1,    1,    1,  128,  128,  128,
   1,    1,  128,  128,    1,  128,  128,    1,
   1,    1,  128,  128,    1,   64,    1,    1,
   1,    1,    1,    1,    1,   64,    1,    1,
   0,    1,    1,    0,    1,   64,    1,    1
};

const uint8_t img_bg[] = {
   0,  0,  0,  0,  0,  0,  0,  0,
   0,  0,  0,  0,  0,  0,  0,  0,
   0,  0,  1,  1,  1,  0,  0,  0,
   0,  1,  1,  1,  1,  1,  1,  1,
   1,  1,  1,  1,  1,  1,  0,  0,
   0,  0,  1,  1,  1,  1,  0,  0,
   0,  0,  0,  1,  1,  1,  1,  1,
   0,  0,  0,  1,  1,  1,  1,  1
};


// NOTE: suspected bug still in computation because there is a slight increase
//    in total response after downsample, whereas this should remain constant
//    (e.g., with center multiple of 4, original has 1984->496, while output
//    has 505. with center multiple of 2, output has total of 519)
const uint8_t expected[] = {
  15,   33,   38,   27,
  12,   25,   44,   45,
   1,   72,   53,   84,
   1,   25,   25,   17
};

const uint8_t expected_bg[] = {
  0,   0,   0,   0,
  0,   1,   1,   0,
  1,   1,   1,   0,
  0,   0,   1,   1
};



//static uint32_t check_brightness(void)
//{
//   ////////
//   printf("Checking brightness\n");
//   uint32_t errs = 0;
//   image_size_type img_size = { .x=8, .y=8 };
//   image_size_type down_size = { .x=4, .y=4 };
//   uint32_t full = 0;
//   uint32_t mini = 0;
//   for (uint32_t i=0; i<img_size.x*img_size.y; i++)
//      full += img[i];
//   for (uint32_t i=0; i<down_size.x*down_size.y; i++)
//      mini += expected[i];
//
//   printf("Full: %d\n", full);
//   printf("Mini: %d\n", mini);
//   return errs;
//}

static uint32_t downsample_content()
{
   ////////
   printf("Checking downsample (content)\n");
   uint32_t errs = 0;
   image_size_type img_size = { .x=8, .y=8 };
   image_size_type down_size = { .x=4, .y=4 };
   uint8_t buf[16];
   downsample_33(img, img_size, down_size, buf);
   for (uint32_t r=0; r<down_size.rows; r++) {
      for (uint32_t c=0; c<down_size.cols; c++) {
         uint32_t idx = c + r * down_size.cols;
         if (buf[idx] != expected[idx]) {
            fprintf(stderr, "Error: %d,%d has unexpected value\n", c, r);
            errs++;
         }
      }
   }
   if (errs > 0) {
      printf("--------------------------------------------------\n");
      // print source
      printf("Original:\n");
      uint32_t sum = 0;
      for (uint32_t r=0; r<img_size.rows; r++) {
         for (uint32_t c=0; c<img_size.cols; c++) {
            uint32_t idx = c + r * img_size.cols;
            printf("%4d", img[idx]);
            sum += img[idx];
         }
         printf("\n");
      }
      printf("total: %d\n", sum);
      // print output
      sum = 0;
      printf("Donwsample:\n");
      for (uint32_t r=0; r<down_size.rows; r++) {
         for (uint32_t c=0; c<down_size.cols; c++) {
            uint32_t idx = c + r * down_size.cols;
            printf("%4d", buf[idx]);
            sum += buf[idx];
         }
         printf("\n");
      }
      printf("total: %d\n", sum);
      // print expected
      sum = 0;
      printf("Expected:\n");
      for (uint32_t r=0; r<down_size.rows; r++) {
         for (uint32_t c=0; c<down_size.cols; c++) {
            uint32_t idx = c + r * down_size.cols;
            printf("%4d", expected[idx]);
            sum += expected[idx];
         }
         printf("\n");
      }
      printf("total: %d\n", sum);
   }
   return errs;
}


static uint32_t downsample_background()
{
   ////////
   printf("Checking downsample (background)\n");
   uint32_t errs = 0;
   image_size_type img_size = { .x=8, .y=8 };
   image_size_type down_size = { .x=4, .y=4 };
   uint8_t buf[16];
   downsample_33(img_bg, img_size, down_size, buf);
   for (uint32_t r=0; r<down_size.rows; r++) {
      for (uint32_t c=0; c<down_size.cols; c++) {
         uint32_t idx = c + r * down_size.cols;
         if (buf[idx] != expected_bg[idx]) {
            fprintf(stderr, "Error: %d,%d has unexpected value\n", c, r);
            errs++;
         }
      }
   }
   if (errs > 0) {
      printf("--------------------------------------------------\n");
      // print source
      printf("Original:\n");
      uint32_t sum = 0;
      for (uint32_t r=0; r<img_size.rows; r++) {
         for (uint32_t c=0; c<img_size.cols; c++) {
            uint32_t idx = c + r * img_size.cols;
            printf("%4d", img_bg[idx]);
            sum += img_bg[idx];
         }
         printf("\n");
      }
      printf("total: %d\n", sum);
      // print output
      sum = 0;
      printf("Donwsample:\n");
      for (uint32_t r=0; r<down_size.rows; r++) {
         for (uint32_t c=0; c<down_size.cols; c++) {
            uint32_t idx = c + r * down_size.cols;
            printf("%4d", buf[idx]);
            sum += buf[idx];
         }
         printf("\n");
      }
      printf("total: %d\n", sum);
      // print expected
      sum = 0;
      printf("Expected:\n");
      for (uint32_t r=0; r<down_size.rows; r++) {
         for (uint32_t c=0; c<down_size.cols; c++) {
            uint32_t idx = c + r * down_size.cols;
            printf("%4d", expected_bg[idx]);
            sum += expected_bg[idx];
         }
         printf("\n");
      }
      printf("total: %d\n", sum);
   }
   return errs;
}



// check downsample
int main(int argc, char **argv)
{
   ////////
   uint32_t errs = 0;
   //
   errs += downsample_content();
   errs += downsample_background();
   //errs += check_brightness();
   if (errs == 0) {
      printf("--------------------\n");
      printf("--  Tests passed  --\n");
      printf("--------------------\n");
   } else {
      printf("---------------------------------\n");
      printf("***  One or more tests failed ***\n");
   }
   return (int) errs;
}

#endif // TEST_DOWNSAMPLE

