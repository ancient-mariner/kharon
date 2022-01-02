#include "pixel_types.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static unsigned int check_vy_pixel_type(void)
{
   unsigned int errs = 0;
   printf("Checking vy_pixel_type\n");
   // check size and structure of pixel_edge_type
   // size of individual not important. size of array is
   vy_pixel_type color;
   memset(&color, 0, sizeof color);
   color.all = 0x0101;
   if ((color.v != 1) || (color.y != 1)) {
      fprintf(stderr, "pixel_color_type.all = X does not work as expected\n");
      errs++;
   }
   color.channel[V_CHAN_IDX] = 2;
   color.channel[Y_CHAN_IDX] = 3;
   if ((color.v != 2) || (color.y != 3)) {
      // if indices not aligned, may need to wrap enum definition in
      //    an ifdef for this platform
      fprintf(stderr, "pixel_color_type.channel indices not aligned\n");
      errs++;
   }
   vy_pixel_type colors[16];
   if (sizeof(colors) != 32) {
      fprintf(stderr, "pixel_color_type size incorrect. 16 of require "
            "%ld bytes\n", sizeof(colors));
      errs++;
   }
   return errs;
}

static unsigned int check_pixel_color_type(void)
{
   unsigned int errs = 0;
   printf("Checking pixel_color_type\n");
   // check size and structure of pixel_edge_type
   // size of individual not important. size of array is
   pixel_color_type color;
   memset(&color, 0, sizeof color);
   color.all = 0x0101;
   if ((color.v != 1) || (color.y != 1)) {
      fprintf(stderr, "pixel_color_type.all = X does not work as expected\n");
      errs++;
   }
   color.channel[V_CHAN_IDX] = 2;
   color.channel[Y_CHAN_IDX] = 3;
   if ((color.v != 2) || (color.y != 3)) {
      // if indices not aligned, may need to wrap enum definition in
      //    an ifdef for this platform
      fprintf(stderr, "pixel_color_type.channel indices not aligned\n");
      errs++;
   }
   pixel_color_type colors[16];
   if (sizeof(colors) != 32) {
      fprintf(stderr, "pixel_color_type size incorrect. 16 of require "
            "%ld bytes\n", sizeof(colors));
      errs++;
   }
   return errs;
}

static unsigned int check_boundary_pixel_type(void)
{
   unsigned int errs = 0;
   printf("Checking boundary_pixel_type\n");
   // check size and structure of pixel_edge_type
   // size of individual not important. size of array is
   boundary_pixel_type pix;
   size_t sz = sizeof pix;
   if (sz != 24) {
      fprintf(stderr, "boundary pixel should be 24 bytes. it's %ld\n", sz);
      errs++;
   }
   return errs;
}

static unsigned int check_bitshift(void)
{
   unsigned int errs = 0;
   printf("Checking bitshift\n");
   // unsigned rightshift should lose sign bit
   uint16_t a = 0xffff;
   a >>= 1;
   if (a != 0x7fff) {
      fprintf(stderr, "unsigned 0xffff >> 1 is 0x%04x, not 0x7fff\n", a);
      errs++;
   }
   a >>= 14;
   if (a != 0x0001) {
      fprintf(stderr, "unsigned 0xffff >> 15 is 0x%04x, not 0x0001\n", a);
      errs++;
   }
   // unsigned rightshift should lose sign bit
   uint32_t b = 0xffffffff;
   b >>= 1;
   if (b != 0x7fffffff) {
      fprintf(stderr, "unsigned 0xffffffff >> 1 is 0x%08x\n", b);
      errs++;
   }
   // signed rightshift should maintain sign bit
   int32_t c = (int32_t) 0xffffffff;
   c >>= 1;
   if (c != (int32_t) 0xffffffff) {
      fprintf(stderr, "signed 0xffffffff >> 1 is 0x%08x\n", c);
      errs++;
   }
   return errs;
}

static unsigned int check_pix_per_degree(void)
{
   unsigned int errs = 0;
   printf("Checking pixels per degree\n");
   // pixels per degree should be integral, at least on lowest pyramid
   //    level (e.g., layout grids need full degree and full pixel 
   //    boundaries; maybe image stitching does on all levels). 
   // check all levels. if it's necessary to trim back test that can
   //    be done... after code is evaluated for safety
   // ugh. using PIXELS_PER_DEGREE requires linking to pinet.a
   // use constants instead. throw an error if number of pyramid levels
   //    changes so test can be updated
   if (NUM_PYRAMID_LEVELS != 2) {
      fprintf(stderr, "Test not complete due unexpected number of pyramid "
            "levels\n");
      errs++;
   }
   // 0
   {
      uint32_t ppd = (uint32_t) PIX_PER_DEG[0];
      double fppd = (double) ppd;
      if (fppd != PIX_PER_DEG[0]) {
         fprintf(stderr, "Non-integral pixels per degree on level 0\n");
         errs++;
      }
   }
   // 0
   {
      uint32_t ppd = (uint32_t) PIX_PER_DEG[1];
      double fppd = (double) ppd;
      if (fppd != PIX_PER_DEG[1]) {
         fprintf(stderr, "Non-integral pixels per degree on level 1\n");
         errs++;
      }
   }
   return errs;
}

static unsigned int check_negative_float_conversion(void)
{
   unsigned int errs = 0;
   printf("Checking negative float-int conversion\n");
   // make sure -0.5f converts to integer 0 on cast
   // if this test breaks, code that assumes slightly negative floats
   //    will round to zero will break (at least at one point, layout
   //    module made this assumption)
   // TODO switch to using ceil and floor instead of cast
   float f = -0.5f;
   int i = (int) f;
   if (i != 0) {
      fprintf(stderr, "(int) %f is cast to %d, not 0\n", (double) f, i);
      errs++;
   }
   ////////
   return errs;
}


// verify conversion between BAM32 and double work as expected
// uses argc (which is assumed to be 1) to 'modify' float values to
//    prevent compiler from optimizing constants away, skewing results
//    compared to normal runtime operation
static unsigned int check_bam32_conversions(int argc)
{
   unsigned int errs = 0;
   printf("Checking bam32 conversions\n");
   // we can't hardcode a constant to check as the optimizer can 
   //    get too smart and alter behavior (that's how this problem was
   //    found in the first place). use argc to build the constant.
   //    for this test to work, argc should always be 1 (ie, no params
   //    to test)
   {
      // positive degrees <360 
      double deg = 1.234 * (double) argc;
      bam32_type bam;
      CVT_DEG_TO_BAM32(deg, bam);
      uint32_t expected = 14722193;
      if (bam.angle32 != expected) {
         fprintf(stderr, "convert positive degrees to bam32 is off. got %d, "
               "expected %d\n", bam.angle32, expected);
         errs++;
      }
   }
   {
      // positive degrees 180
      double deg = 180.0 * (double) argc;
      bam32_type bam;
      CVT_DEG_TO_BAM32(deg, bam);
      uint32_t expected = 2147483648;
      if (bam.angle32 != expected) {
         fprintf(stderr, "convert positive degrees =180 to bam32 is off. "
               "got %d, expected %d\n", bam.angle32, expected);
         errs++;
      }
   }
   {
      // positive degrees >360
      double deg = 721.234 * (double) argc;
      bam32_type bam;
      CVT_DEG_TO_BAM32(deg, bam);
      uint32_t expected = 14722193;
      if (bam.angle32 != expected) {
         fprintf(stderr, "convert positive degrees >360 to bam32 is off. "
               "got %d, expected %d\n", bam.angle32, expected);
         errs++;
      }
   }
   {
      // negative degrees >-180
      double deg = -1.234 * (double) argc;
      bam32_type bam;
      CVT_DEG_TO_BAM32(deg, bam);
      int32_t expected = -14722193;
      if (bam.sangle32 != expected) {
         fprintf(stderr, "convert negative degrees >-180 to bam32 is off. "
               "got %d, expected %d\n", bam.sangle32, expected);
         errs++;
      }
   }
   {
      // const negative degrees >-180
      double deg = -1.234;
      bam32_type bam;
      CVT_DEG_TO_BAM32(deg, bam);
      int32_t expected = -14722193;
      if (bam.sangle32 != expected) {
         fprintf(stderr, "convert const negative degrees >-180 to bam32 "
               "is off. got %d, expected %d\n", bam.sangle32, expected);
         errs++;
      }
   }
   {
      // negative degrees >-360
      double deg = -181.234 * (double) argc;
      bam32_type bam;
      CVT_DEG_TO_BAM32(deg, bam);
      uint32_t expected = 2132761455;
      if (bam.angle32 != expected) {
         fprintf(stderr, "convert negative degrees >-360 to bam32 is off. "
               "got %d, expected %d\n", bam.angle32, expected);
         errs++;
      }
   }
   {
      // negative degrees <-360
      double deg = -721.234 * (double) argc;
      bam32_type bam;
      CVT_DEG_TO_BAM32(deg, bam);
      int32_t expected = -14722193;
      if (bam.sangle32 != expected) {
         fprintf(stderr, "convert negative degrees <-360 to bam32 is off. "
               "got %d, expected %d\n", bam.sangle32, expected);
         errs++;
      }
   }
   {
      // negative constant degrees <-360
      double deg = -721.234;
      bam32_type bam;
      CVT_DEG_TO_BAM32(deg, bam);
      int32_t expected = -14722193;
      if (bam.sangle32 != expected) {
         fprintf(stderr, "convert negative const degrees <-360 to bam32 "
               "is off. got %d, expected %d\n", bam.sangle32, expected);
         errs++;
      }
   }
   ////////
   return errs;
}


//#warning "Have this test run on every startup"
// ...at least until there's a test system in place that runs the test
//    as a part of the build
int main(int argc, char** argv) 
{
   (void) argc;
   (void) argv;
   unsigned int errs = 0;
   errs += check_vy_pixel_type();
   errs += check_pixel_color_type();
   errs += check_boundary_pixel_type();
   errs += check_bitshift();
   errs += check_pix_per_degree();
   errs += check_negative_float_conversion();
   errs += check_bam32_conversions(argc);
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
