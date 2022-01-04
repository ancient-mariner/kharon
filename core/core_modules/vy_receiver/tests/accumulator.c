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
#include <sys/time.h>
#include "../exp_accumulator.c"

// write a pgm file for manual evaluation
static void write_pnm(
      /* in     */ const char *name,
      /* in     */ const uint32_t w,
      /* in     */ const uint32_t h,
      /* in     */ const uint8_t *buf
      )
{
   FILE *fp = fopen(name, "w");
   if (!fp) {
      printf("Failed to create output file '%s'\n", name);
      goto end;
   }
   fprintf(fp, "P5\n%d %d\n255\n", w, h);
   fwrite(buf, w, h, fp);
end:
   if (fp)
      fclose(fp);
}

static uint32_t check_accumulator_pos(
      /* in     */ const uint32_t orig_x,
      /* in     */ const uint32_t orig_y,
      /* in     */ const accumulator_coord_type *pos,
      /* in     */ const uint16_t x,
      /* in     */ const uint16_t y,
      /* in     */ const uint16_t nw,
      /* in     */ const uint16_t ne,
      /* in     */ const uint16_t sw,
      /* in     */ const uint16_t se
      )
{
   uint32_t errs = 0;
   printf("  Coordinate %d,%d\n", orig_x, orig_y);
   if (pos->x != x) {
      printf("    Coordinate x (%d) does not match expected value (%d)\n",
            pos->x, x);
      errs++;
   }
   if (pos->y != y) {
      printf("    Coordinate y (%d) does not match expected value (%d)\n",
            pos->y, y);
      errs++;
   }
   if (pos->nw != nw) {
      printf("    Coordinate nw (%d) does not match expected value (%d)\n",
            pos->nw, nw);
      errs++;
   }
   if (pos->ne != ne) {
      printf("    Coordinate ne (%d) does not match expected value (%d)\n",
            pos->ne, ne);
      errs++;
   }
   if (pos->sw != sw) {
      printf("    Coordinate sw (%d) does not match expected value (%d)\n",
            pos->sw, sw);
      errs++;
   }
   if (pos->se != se) {
      printf("    Coordinate se (%d) does not match expected value (%d)\n",
            pos->se, se);
      errs++;
   }
   if (errs > 0) {
//      printf("  Coordinate %d,%d\n", orig_x, orig_y);
      print_coord(orig_x, orig_y, pos);
   }
   return errs;
}

static uint32_t test_perspective_mapping(void)
{
   printf("Testing perspective mapping\n");
   uint32_t errs = 0;
   ////////////
   uint32_t x, y;
   accumulator_coord_type pos;
   // top left
   x = y = 0;
   get_perspective_pix(x, y, VY_COLS, VY_ROWS, &pos);
   errs += check_accumulator_pos(x, y, &pos, 27, 38, 12, 36, 4, 12);
   // mid left
   x = 0;
   y = VY_ROWS / 2;
   get_perspective_pix(x, y, VY_COLS, VY_ROWS, &pos);
   errs += check_accumulator_pos(x, y, &pos, 0, 292, 16, 0, 48, 0);
   // mid top
   x = VY_COLS / 2;
   y = 0;
   get_perspective_pix(x, y, VY_COLS, VY_ROWS, &pos);
   errs += check_accumulator_pos(x, y, &pos, 373, 0, 42, 6, 14, 2);
   // top right
   x = VY_COLS;
   y = 0;
   get_perspective_pix(x, y, VY_COLS, VY_ROWS, &pos);
   errs += check_accumulator_pos(x, y, &pos, 718, 38, 18, 30, 6, 10);
   // center
   x = VY_COLS / 2;
   y = VY_ROWS / 2;
   get_perspective_pix(x, y, VY_COLS, VY_ROWS, &pos);
   errs += check_accumulator_pos(x, y, &pos, 373, 292, 14, 2, 42, 6);
   // mid upper/left
   x = VY_COLS / 4;
   y = VY_ROWS / 4;
   get_perspective_pix(x, y, VY_COLS, VY_ROWS, &pos);
   errs += check_accumulator_pos(x, y, &pos, 176, 145, 5, 3, 35, 21);
   // mid lower/right
   x = 3 * VY_COLS / 4;
   y = 3 * VY_ROWS / 4;
   get_perspective_pix(x, y, VY_COLS, VY_ROWS, &pos);
   errs += check_accumulator_pos(x, y, &pos, 569, 439, 3, 21, 5, 35);
   // mid lower, far right
   x = VY_COLS;
   y = 3 * VY_ROWS / 4;
   get_perspective_pix(x, y, VY_COLS, VY_ROWS, &pos);
   errs += check_accumulator_pos(x, y, &pos, 738, 424, 8, 24, 8, 24);
   /////////////////////////////////////////////
   if (errs == 0) {
      printf("    passed\n");
   } else {
      printf("    %d errors\n", errs);
   }
   return errs;
}

static uint8_t *raw_img_ = NULL;
static uint8_t *out_img_ = NULL;

static void run_perspective(void)
{
//   printf("Testing perspective v\n");
   ////////////
   // allocate image buffer
   if (raw_img_ == NULL) {
      raw_img_ = calloc(VY_COLS, VY_ROWS);
      if (!raw_img_) {
         printf("Failed to create input image buffer\n");
         exit(1);
      }
      // fill buffer with content, row by row. fill central area with white
      //    and YUV border (padding) with gray
      for (uint32_t i=0; i<VY_ROWS; i++) {
         uint32_t idx = i * VY_COLS;
         memset(&raw_img_[idx], 255, VY_COLS);
         memset(&raw_img_[idx+VY_COLS], 128, VY_COLS-VY_COLS);
         // draw diagonal lines across image
         uint32_t a = i * VY_COLS / VY_ROWS;
         raw_img_[i * VY_COLS + a] = 92;
         uint32_t b = VY_COLS - a - 1;
         raw_img_[i * VY_COLS + b] = 160;
      }
      // draw boxes on image
      for (uint32_t i=0; i<VY_ROWS/2; i+=50) {
         uint32_t top = VY_ROWS/2 - i;
         uint32_t bottom = VY_ROWS/2 + i;
         uint32_t x_delta = i * VY_COLS / VY_ROWS;
         uint32_t left = VY_COLS/2 - x_delta;
         uint32_t right = VY_COLS/2 + x_delta;
//printf("top=%d, bot=%d, left=%d, right=%d\n", top, bottom, left, right);
         // top line
         for (uint32_t c=left; c<=right; c++) {
            raw_img_[top * VY_COLS + c] = 32;
            raw_img_[bottom * VY_COLS + c] = 32;
         }
         for (uint32_t r=top; r<bottom+1; r++) {
            raw_img_[r * VY_COLS + left] = 32;
            raw_img_[r * VY_COLS + right] = 32;
         }
      }
      // draw black region through image
      for (uint32_t i=VY_ROWS/2+5; i<VY_ROWS/2+15; i++) {
         uint32_t idx = i * VY_COLS;
         memset(&raw_img_[idx], 0, VY_COLS);
      }
      reset_accumulator();
   }
   if (out_img_ == NULL) {
      out_img_ = calloc(PERSP_WIDTH_SIZE, PERSP_HEIGHT_SIZE);
      if (!out_img_) {
         printf("Failed to create output image buffer\n");
         exit(1);
      }
   }
   push_image_to_accumulator(raw_img_);
   collapse_and_reset_accumulator(out_img_);
   write_pnm("out-y.pnm", PERSP_WIDTH_SIZE, PERSP_HEIGHT_SIZE, out_img_);
   write_pnm("raw-y.pnm", VY_COLS, VY_ROWS, raw_img_);
}

int main(int argc, char** argv)
{
   uint32_t errs = 0;
   errs += test_perspective_mapping();
   create_accumulator();

//   struct timeval t0, t1;
//   gettimeofday(&t0, NULL);
//   run_perspective();
//   gettimeofday(&t1, NULL);
//   double d0 = (double) t0.tv_sec + (double) t0.tv_usec * 0.000001;
//   double d1 = (double) t1.tv_sec + (double) t1.tv_usec * 0.000001;
//   printf("Average time: %f\n", (d1 - d0));

   struct timeval t0, t1;
   gettimeofday(&t0, NULL);
   for (uint32_t i=0; i<100; i++) {
      run_perspective();  // v
      run_perspective();  // y
   }
   gettimeofday(&t1, NULL);
   double d0 = (double) t0.tv_sec + (double) t0.tv_usec * 0.000001;
   double d1 = (double) t1.tv_sec + (double) t1.tv_usec * 0.000001;
   printf("Average time: %f\n", (d1 - d0) *0.01);


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

