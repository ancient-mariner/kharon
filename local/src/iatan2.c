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
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <assert.h>
#include <stdint.h>
#include "iatan2.h"

//// TODO find appropriate home for these functions


// rotational distance necessary to align two vectors
// a and b represent 360-degree orientation mapped to [0,255]
// return value on [0,128]
uint8_t angle_between_vectors(
      /* in     */ const uint8_t a,
      /* in     */ const uint8_t b
      )
{
   uint8_t c = (uint8_t) (a - b);
   // if value > 128 then rotation is shorter in the opposite direction
   c = c>128 ? (uint8_t) (256-c) : c;
   assert(c <= 128);
   return c;
}

// rotational distance between two unoriented (bidirectional) vectors
// a and b represent 360-degree orientation mapped to [0,255]
// return value on [0,64)
uint8_t angle_between_lines(
      /* in     */ const uint8_t a,
      /* in     */ const uint8_t b)
{
   // double angle, mirror about Y axis (this makes orthogonal values
   //    stay near negative Y axis, and nearly parallel lines point
   //    near positive Y), then halve angle to reverse doubling. ortho
   //    values are near +X (64) while parallel are near +Y (ie, 0)
   // negative values (b > a) are handled similarly, with mirror on Y
   //    converting to positive
   const uint8_t a2 = (uint8_t) (2 * a);
   const uint8_t b2 = (uint8_t) (2 * b);
   uint8_t c = (uint8_t) (a2 - b2);
   c = c>127 ? (uint8_t) ((256-c)/2) : (uint8_t) (c/2);
   assert(c <= 64);
   return c;
}


// to compile test program, either uncomment following line and
//    recompile or specify -DIATAN2_TEST when compiling
//#define   IATAN2_TEST

// degrees mapped to 256 values so they can be stored in bytes
// one byte-radian is approx 1.5 degrees (360/255=1.412)

////////////////////////////////////////////////////////////////////////

// tables aligned to 64-bit cache line boundary
// use size <=64 to fit table into one cache line
// 48 appears to allow covering all 256 'degrees'
// 64 better for radius estimation
//
// radius is estimated by multiplying longer of x,y by icos and
//    dividing by 128
//
#define AT2_BINS 64
static uint8_t AT2_LUT[AT2_BINS] __attribute__ ((__aligned__(64))) = {
     0,     1,     1,     2,     3,     3,     4,     5,
     5,     6,     6,     7,     8,     8,     9,    10,
    10,    11,    11,    12,    13,    13,    14,    14,
    15,    15,    16,    16,    17,    18,    18,    19,
    19,    20,    20,    21,    21,    22,    22,    23,
    23,    24,    24,    24,    25,    25,    26,    26,
    27,    27,    27,    28,    28,    28,    29,    29,
    30,    30,    30,    31,    31,    31,    32,    32
};

#define ICOS_BINS (AT2_BINS)
static uint8_t ICOS_LUT[ICOS_BINS] __attribute__ ((__aligned__(64))) = {
   128,   128,   128,   128,   128,   128,   129,   129,
   129,   129,   130,   130,   130,   131,   131,   132,
   132,   133,   133,   134,   134,   135,   136,   136,
   137,   138,   138,   139,   140,   141,   142,   143,
   144,   144,   145,   146,   147,   148,   149,   151,
   152,   153,   154,   155,   156,   157,   158,   160,
   161,   162,   163,   165,   166,   167,   169,   170,
   171,   173,   174,   175,   177,   178,   180,   181
};


#define AT2_STOPS 256
#define AT2_POSY  (0)
#define AT2_POSX  (    AT2_STOPS / 4)
#define AT2_NEGY  (2 * AT2_STOPS / 4)
#define AT2_NEGX  (3 * AT2_STOPS / 4)

// approximate integral atan2 and len(x,y)
// mimic atan2 logic but return integer value on [0-256) that represents
//    degrees on [0,360) degrees
// 0 is north, angle increases moving clockwise
// angle points in direction of increasing gradient (ie, points up hill)
void rtheta(int32_t x, int32_t y, uint32_t *r, uint8_t *theta)
{
   // initialize to null valuem and return if x,y == 0
   *theta = 0;
   *r = 0;
   if ((x | y) == 0)
      return;
   //
   const int32_t SIGN_X = (x >> 31) & 1;   // 0 if positive; 1 if negative
   const int32_t SIGN_Y = (y >> 31) & 1;   // 0 if positive; 1 if negative
   int32_t idx;
   switch (SIGN_X) {
      case 0:  // positive x
         switch (SIGN_Y) {
            case 0:  // positive y -- first quadrant
               if (y > x) {   // 0-45 degrees
//printf("Q1, y>x\n");
                  //printf("idx = %d\n", x * (AT2_BINS-1) / y);
                  idx = x * (AT2_BINS-1) / y;
                  *theta = (uint8_t) (AT2_LUT[idx]);
                  *r = (uint16_t) (1 + y * ICOS_LUT[idx] / 128);
               } else { // 45-90 degrees
//printf("Q1, y<=x\n");
                  idx = y * (AT2_BINS-1) / x;
                  *theta = (uint8_t) (AT2_POSX - AT2_LUT[idx]);
                  *r = (uint16_t) (1 + x * ICOS_LUT[idx] / 128);
               }
               break;
            case 1:  // negative y -- fourth quadrant
               if (x > -y) {
//printf("Q4, x > -y\n");
                  idx = -y * (AT2_BINS-1) / x;
                  *theta = (uint8_t) (AT2_POSX + AT2_LUT[idx]);
                  *r = (uint16_t) (1 + x * ICOS_LUT[idx] / 128);
               } else {
//printf("Q4, x <= -y\n");
                  idx = -x * (AT2_BINS-1) / y;
                  *theta = (uint8_t) (AT2_NEGY - AT2_LUT[idx]);
                  *r = (uint16_t) (1 + -y * ICOS_LUT[idx] / 128);
               }
               break;
         };
         break;
      case 1:  // negative x
         switch (SIGN_Y) {   // is y positive?
            case 0:  // positive y -- second quadrant
               if (-x > y) {
//printf("Q2, -x > y\n");
                  idx = -y * (AT2_BINS-1) / x;
                  *theta = (uint8_t) (AT2_NEGX + AT2_LUT[idx]);
                  *r = (uint16_t) (1 + -x * ICOS_LUT[idx] / 128);
               } else {
//printf("Q2, -x <= y\n");
                  idx = -x * (AT2_BINS-1) / y;
                  *theta = (uint8_t) (AT2_STOPS - AT2_LUT[idx]);
                  *r = (uint16_t) (1 + y * ICOS_LUT[idx] / 128);
               }
               break;
            case 1:  // negative y -- third quadrant
//printf("Q3, x < y\n");
               if (x < y) {
                  idx = y * (AT2_BINS-1) / x;
                  *theta = (uint8_t) (AT2_NEGX - AT2_LUT[idx]);
                  *r = (uint16_t) (1 + -x * ICOS_LUT[idx] / 128);
               } else {
//printf("Q3, x >= y\n");
                  idx = x * (AT2_BINS-1) / y;
                  *theta = (uint8_t) (AT2_NEGY + AT2_LUT[idx]);
                  *r = (uint16_t) (1 + -y * ICOS_LUT[idx] / 128);
               }
               break;
         };
         break;
   };
}

// mimic atan2 logic but return integer value on [0-256) that represents
//    degrees on [0,360) degrees
// function takes integral inputs and uses look-up table for calculation
uint8_t iatan2(int32_t y, int32_t x)
{
   if ((x | y) == 0)
      return 0;
   const int SIGN_X = (x >> 31) & 1;   // 0 if positive; 1 if negative
   const int SIGN_Y = (y >> 31) & 1;   // 0 if positive; 1 if negative
   //printf("Start: %d,%d   (%d,%d)\n", x, y, SIGN_X, SIGN_Y);
   switch (SIGN_X) {
      case 0:  // positive x
         switch (SIGN_Y) {
            case 0:  // positive y -- first quadrant
               if (y > x) {
                  //printf("idx = %d\n", x * (AT2_BINS-1) / y);
                  return (uint8_t) (AT2_LUT[x * (AT2_BINS-1) / y]);
               } else {
                  return (uint8_t) (AT2_POSX - AT2_LUT[y * (AT2_BINS-1) / x]);
               }
               break;
            case 1:  // negative y -- fourth quadrant
               if (x > -y)
                  return (uint8_t) (AT2_POSX + AT2_LUT[-y * (AT2_BINS-1) / x]);
               else
                  return (uint8_t) (AT2_NEGY - AT2_LUT[-x * (AT2_BINS-1) / y]);
               break;
         };
         break;
      case 1:  // negative x
         switch (SIGN_Y) {   // is y positive?
            case 0:  // positive y -- second quadrant
               if (-x > y)
                  return (uint8_t) (AT2_NEGX + AT2_LUT[-y * (AT2_BINS-1) / x]);
               else
                  return (uint8_t) (AT2_STOPS - AT2_LUT[-x * (AT2_BINS-1) / y]);
               break;
            case 1:  // negative y -- third quadrant
               if (x < y)
                  return (uint8_t) (AT2_NEGX - AT2_LUT[y * (AT2_BINS-1) / x]);
               else
                  return (uint8_t) (AT2_NEGY + AT2_LUT[x * (AT2_BINS-1) / y]);
               break;
         };
         break;
   };
   return 0;
}

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
#if defined(IATAN2_TEST)
#include <stdlib.h>

static void build_luts(void);
static uint32_t test_rtheta(void);
static uint32_t test_iatan2(void);
static uint32_t test_angle_between(void);

static uint32_t test_angle_between()
{
   uint32_t errs = 0;
   printf("testing angle_between_vectors, angle_between_lines\n");
   const uint8_t a[] = {   1,   1,   1,  60,  60, 120, 120, 130,  1 };
   const uint8_t b[] = {  60, 120, 254, 120, 254, 254, 130, 150, 10 };
   // angles between vectors
   const uint8_t c[] = {  59, 119,   3,  60,  62, 122,  10,  20,  9 };
   const uint32_t ITER = (uint32_t) sizeof c;
   for (uint32_t i=0; i<ITER; i++) {
      uint8_t x = angle_between_vectors(a[i], b[i]);
      if (x != c[i]) {
         fprintf(stderr, "angle_between_vectors(%d,%d) = %d, not %d\n",
               a[i], b[i], c[i], x);
      }
      x = angle_between_vectors(b[i], a[i]);
      if (x != c[i]) {
         fprintf(stderr, "angle_between_vectors(%d,%d) = %d, not %d\n",
               b[i], a[i], c[i], x);
      }
   }
   // angles between lines
   const uint8_t d[] = {  59,   9,   3,  60,  62,   6,  10,  20,  9 };
   for (uint32_t i=0; i<ITER; i++) {
      uint8_t x = angle_between_lines(a[i], b[i]);
      if (x != d[i]) {
         fprintf(stderr, "angle_between_lines(%d,%d) = %d, not %d\n",
               a[i], b[i], d[i], x);
      }
      x = angle_between_lines(b[i], a[i]);
      if (x != d[i]) {
         fprintf(stderr, "angle_between_lines(%d,%d) = %d, not %d\n",
               b[i], a[i], d[i], x);
      }
   }
   //
   if (errs == 0) {
      printf("    passed\n");
   } else {
      printf("    %d errors\n", errs);
   }
   return errs;
}


////////////////////////////////////
// code to build lookup tables
//#include <stdio.h>
//#include <math.h>
static void build_luts()
{
   int i, ith, ics;
   const int n = 64;
   printf("#define AT2_BINS %d\n", n);
   printf("static uint8_t AT2_LUT[AT2_BINS] __attribute__ "
      "((__aligned__(64))) = {\n");
   for (i=0; i<n; i++) {
      ith = (int) (0.5 + 128.0 * atan2(i, (n-1)) / M_PI);
      if (i < (n-1))
         printf("%6d,", ith);
      else
         printf("%6d", ith);
      if ((i%8) == 7)
         printf("\n");
   }
   printf("};\n\n");
   //
   printf("#define ICOS_BINS (AT2_BINS)\n");
   printf("static uint8_t ICOS_LUT[ICOS_BINS] __attribute__ "
      "((__aligned__(64))) = {\n");
   for (i=0; i<n; i++) {
      double thetar = atan2(i, (n-1));
      double icos = 1.0 / cos(thetar);
      ics = (int) (128.0 * icos + 0.5);
      if (i < (n-1))
         printf("%6d,", ics);
      else
         printf("%6d", ics);
      if ((i%8) == 7)
         printf("\n");
   }
   printf("};\n");
}
////////////////////////////////////


static uint32_t test_rtheta(void)
{
   printf("    rtheta()\n");
   uint8_t prev = 0;
   uint8_t curr = 0;
   uint32_t r;
   const uint32_t STEP = 2;
   const int32_t MAX_DIF = 2;
   const float MAX_DELTA = 0.015f;
   for (uint32_t th=STEP; th<360; th+=STEP) {
      int16_t x = (int16_t) (100 * sin(M_PI * th / 180.0));
      int16_t y = (int16_t) (100 * cos(M_PI * th / 180.0));
      rtheta(x, y, &r, &curr);
      if ((curr - prev) > MAX_DIF) {
         fprintf(stderr, "atan2 test failed. Gap between adjacent theta "
               "too large\n");
         fprintf(stderr, "theta steps = %d degrees\n", STEP);
         fprintf(stderr, "Max gap = %d\n", MAX_DIF);
         fprintf(stderr, "theta = %d\n", th);
         fprintf(stderr, "%d - %d = %d\n", curr, prev, curr-prev);
         goto err;
      }
      float dist = sqrtf((float) (x*x + y*y));
      float delta = fabsf((float) r - dist);
      if (delta > (dist * MAX_DELTA)) {
         fprintf(stderr, "Radius test failed. At %d,%d, dist=%f, r=%d\n",
               x, y, (double) dist, r);
         goto err;
      }
      prev = curr;
   }
   return 0;
err:
   fprintf(stderr, "iatan2 output:\n");
   for (uint32_t th=5; th<360; th+=5) {
      int16_t x = (int16_t) (100 * sin(M_PI * th / 180.0));
      int16_t y = (int16_t) (100 * cos(M_PI * th / 180.0));
      rtheta(x, y, &r, &curr);
      printf(" theta@%3d = %3d\n", th, curr);
   }
   return 1;
}

static uint32_t test_iatan2()
{
   printf("    iatan2()\n");
   const uint32_t ITER = 200;
   for (uint32_t i=0; i<ITER; i++)
   {
      int16_t x = (int16_t) (-500 + random() % 1000);
      int16_t y = (int16_t) (-500 + random() % 1000);
      uint32_t rad;
      uint8_t t1, t2;
      t1 = iatan2(y, x);
      rtheta(x, y, &rad, &t2);
      if (t1 != t2) {
         fprintf(stderr, "rtheta / iatan2 mismatch\n");
         fprintf(stderr, "%d,%d => %d (iatan2)   %d (rtheta)\n", x, y, t1, t2);
         return 1;
      }
   }
   return 0;
}


int main(int argc, char **argv)
{
   (void) argc;
   (void) argv;
   uint32_t errs = 0;
   printf("Testing:\n");
   errs += test_rtheta();
//   errs += test_extract();
//   errs += test_extract_with_inset();
   errs += test_iatan2();
   errs += test_angle_between();
   printf("\n");
   if (errs == 0) {
      printf("----------\n");
      printf("Tests pass\n");
   } else {
      printf("**********************************\n");
      printf("**** ONE OR MORE TESTS FAILED ****\n");
      printf("**********************************\n");
   }
   return (int) errs;
   // include this call here to keep compiler from complaining about
   //    not-used static function
   build_luts();
}

#endif   // IATAN2_TEST

