#include <stdio.h>
#include <sys/time.h>
#include "../accumulator.c"

// write a pgm file for manual evaluation
void write_pnm(
      /* in     */ const char *name,
      /* in     */ const uint32_t w,
      /* in     */ const uint32_t h,
      /* in     */ const uint8_t *buf
      );
void write_pnm(
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

uint32_t test_perspective_mapping(void);
uint32_t test_perspective_mapping(void)
{
   printf("Testing perspective mapping\n");
   uint32_t errs = 0;
   ////////////
   uint32_t x, y;
   accumulator_coord_type pos;
   // top left
   x = y = 0;
   get_perspective_pix(x, y, CAM_WIDTH_PIX_Y, CAM_HEIGHT_PIX_Y, &pos);
   errs += check_accumulator_pos(x, y, &pos, 33, 42, 48, 16, 0, 0);
   // mid left
   x = 0;
   y = CAM_HEIGHT_PIX_Y / 2;
   get_perspective_pix(x, y, CAM_WIDTH_PIX_Y, CAM_HEIGHT_PIX_Y, &pos);
   errs += check_accumulator_pos(x, y, &pos, 0, 292, 16, 0, 48, 0);
   // mid top
   x = CAM_WIDTH_PIX_Y / 2;
   y = 0;
   get_perspective_pix(x, y, CAM_WIDTH_PIX_Y, CAM_HEIGHT_PIX_Y, &pos);
   errs += check_accumulator_pos(x, y, &pos, 373, 0, 56, 8, 0, 0);
   // top right
   x = CAM_WIDTH_PIX_Y;
   y = 0;
   get_perspective_pix(x, y, CAM_WIDTH_PIX_Y, CAM_HEIGHT_PIX_Y, &pos);
   errs += check_accumulator_pos(x, y, &pos, 713, 42, 64, 0, 0, 0);
   // center
   x = CAM_WIDTH_PIX_Y / 2;
   y = CAM_HEIGHT_PIX_Y / 2;
   get_perspective_pix(x, y, CAM_WIDTH_PIX_Y, CAM_HEIGHT_PIX_Y, &pos);
   errs += check_accumulator_pos(x, y, &pos, 373, 292, 14, 2, 42, 6);
   // mid upper/left
   x = CAM_WIDTH_PIX_Y / 4;
   y = CAM_HEIGHT_PIX_Y / 4;
   get_perspective_pix(x, y, CAM_WIDTH_PIX_Y, CAM_HEIGHT_PIX_Y, &pos);
   errs += check_accumulator_pos(x, y, &pos, 176, 145, 48, 16, 0, 0);
   // mid lower/right
   x = 3 * CAM_WIDTH_PIX_Y / 4;
   y = 3 * CAM_HEIGHT_PIX_Y / 4;
   get_perspective_pix(x, y, CAM_WIDTH_PIX_Y, CAM_HEIGHT_PIX_Y, &pos);
   errs += check_accumulator_pos(x, y, &pos, 570, 440, 32, 0, 32, 0);
   // mid lower, far right
   x = CAM_WIDTH_PIX_Y;
   y = 3 * CAM_HEIGHT_PIX_Y / 4;
   get_perspective_pix(x, y, CAM_WIDTH_PIX_Y, CAM_HEIGHT_PIX_Y, &pos);
   errs += check_accumulator_pos(x, y, &pos, 737, 424, 8, 56, 0, 0);
   /////////////////////////////////////////////
   if (errs == 0) {
      printf("    passed\n");
   } else {
      printf("    %d errors\n", errs);
   }
   return errs;
}

static uint8_t *raw_img_y_ = NULL;
static uint8_t *raw_img_v_ = NULL;
static uint8_t *out_img_y_ = NULL;
static uint8_t *out_img_v_ = NULL;

void test_perspective_y(void);
void test_perspective_y(void)
{
//   printf("Testing perspective v\n");
   ////////////
   // allocate image buffer
   if (raw_img_y_ == NULL) {
      raw_img_y_ = calloc(CAM_WIDTH_PIX_Y_YUV, CAM_HEIGHT_PIX_Y);
      if (!raw_img_y_) {
         printf("Failed to create input image buffer\n");
         exit(1);
      }
      // fill buffer with content, row by row. fill central area with white
      //    and YUV border (padding) with gray
      for (uint32_t i=0; i<CAM_HEIGHT_PIX_Y; i++) {
         uint32_t idx = i * CAM_WIDTH_PIX_Y_YUV;
         memset(&raw_img_y_[idx], 255, CAM_WIDTH_PIX_Y);
         memset(&raw_img_y_[idx+CAM_WIDTH_PIX_Y], 128, 
               CAM_WIDTH_PIX_Y_YUV-CAM_WIDTH_PIX_Y);
         // draw diagonal lines across image
         uint32_t a = i * CAM_WIDTH_PIX_Y / CAM_HEIGHT_PIX_Y;
         raw_img_y_[i * CAM_WIDTH_PIX_Y_YUV + a] = 92;
         uint32_t b = CAM_WIDTH_PIX_Y - a - 1;
         raw_img_y_[i * CAM_WIDTH_PIX_Y_YUV + b] = 160;
      }
      // draw boxes on image
      for (uint32_t i=0; i<CAM_HEIGHT_PIX_Y/2; i+=50) {
         uint32_t top = CAM_HEIGHT_PIX_Y/2 - i;
         uint32_t bottom = CAM_HEIGHT_PIX_Y/2 + i;
         uint32_t x_delta = i * CAM_WIDTH_PIX_Y / CAM_HEIGHT_PIX_Y;
         uint32_t left = CAM_WIDTH_PIX_Y/2 - x_delta;
         uint32_t right = CAM_WIDTH_PIX_Y/2 + x_delta;
//printf("top=%d, bot=%d, left=%d, right=%d\n", top, bottom, left, right);
         // top line
         for (uint32_t c=left; c<=right; c++) {
            raw_img_y_[top * CAM_WIDTH_PIX_Y_YUV + c] = 32;
            raw_img_y_[bottom * CAM_WIDTH_PIX_Y_YUV + c] = 32;
         }
         for (uint32_t r=top; r<bottom+1; r++) {
            raw_img_y_[r * CAM_WIDTH_PIX_Y_YUV + left] = 32;
            raw_img_y_[r * CAM_WIDTH_PIX_Y_YUV + right] = 32;
         }
      }
      reset_accumulator(accum_y_);
   }
   if (out_img_y_ == NULL) {
      out_img_y_ = calloc(PERSP_WIDTH_SIZE, PERSP_HEIGHT_SIZE);
      if (!out_img_y_) {
         printf("Failed to create output image buffer\n");
         exit(1);
      }
   }
   push_image_y_to_accumulator(raw_img_y_);
   collapse_and_reset_accumulator(accum_y_, out_img_y_);
//   write_pnm("out-y.pnm", PERSP_WIDTH_SIZE, PERSP_HEIGHT_SIZE, out_img_y_);
//   write_pnm("raw-y.pnm", CAM_WIDTH_PIX_Y_YUV, CAM_HEIGHT_PIX_Y, raw_img_y_);
}

static void test_perspective_v(void)
{
//   printf("Testing perspective v\n");
   ////////////
   // allocate image buffer
   if (raw_img_v_ == NULL) {
      raw_img_v_ = calloc(CAM_WIDTH_PIX_V_YUV, CAM_HEIGHT_PIX_V);
      if (!raw_img_v_) {
         printf("Failed to create input image buffer\n");
         exit(1);
      }
      // fill buffer with content, row by row. fill central area with white
      //    and YUV border (padding) with gray
      for (uint32_t i=0; i<CAM_HEIGHT_PIX_V; i++) {
         uint32_t idx = i * CAM_WIDTH_PIX_V_YUV;
         memset(&raw_img_v_[idx], 255, CAM_WIDTH_PIX_V);
         memset(&raw_img_v_[idx+CAM_WIDTH_PIX_V], 128, 
               CAM_WIDTH_PIX_V_YUV-CAM_WIDTH_PIX_V);
         // draw diagonal lines across image
         uint32_t a = i * CAM_WIDTH_PIX_V / CAM_HEIGHT_PIX_V;
         raw_img_v_[i * CAM_WIDTH_PIX_V_YUV + a] = 92;
         uint32_t b = CAM_WIDTH_PIX_V - a - 1;
         raw_img_v_[i * CAM_WIDTH_PIX_V_YUV + b] = 160;
      }
      // draw boxes on image
      for (uint32_t i=0; i<CAM_HEIGHT_PIX_V/2; i+=50) {
         uint32_t top = CAM_HEIGHT_PIX_V/2 - i;
         uint32_t bottom = CAM_HEIGHT_PIX_V/2 + i;
         uint32_t x_delta = i * CAM_WIDTH_PIX_V / CAM_HEIGHT_PIX_V;
         uint32_t left = CAM_WIDTH_PIX_V/2 - x_delta;
         uint32_t right = CAM_WIDTH_PIX_V/2 + x_delta;
//printf("top=%d, bot=%d, left=%d, right=%d\n", top, bottom, left, right);
         // top line
         for (uint32_t c=left; c<=right; c++) {
            raw_img_v_[top * CAM_WIDTH_PIX_V_YUV + c] = 32;
            raw_img_v_[bottom * CAM_WIDTH_PIX_V_YUV + c] = 32;
         }
         for (uint32_t r=top; r<bottom+1; r++) {
            raw_img_v_[r * CAM_WIDTH_PIX_V_YUV + left] = 32;
            raw_img_v_[r * CAM_WIDTH_PIX_V_YUV + right] = 32;
         }
      }
      reset_accumulator(accum_v_);
   }
   if (out_img_v_ == NULL) {
      out_img_v_ = calloc(PERSP_WIDTH_SIZE, PERSP_HEIGHT_SIZE);
      if (!out_img_v_) {
         printf("Failed to create output image buffer\n");
         exit(1);
      }
   }
   push_image_v_to_accumulator(raw_img_v_);
   collapse_and_reset_accumulator(accum_v_, out_img_v_);
//   write_pnm("out.pnm", PERSP_WIDTH_SIZE, PERSP_HEIGHT_SIZE, out_img);
}


int main(int argc, char** argv)
{
   (void) argc;
   uint32_t errs = 0;
//   errs += test_perspective_mapping();
   create_accumulators();

   struct timeval t0, t1;
   gettimeofday(&t0, NULL);
   for (uint32_t i=0; i<100; i++) {
      test_perspective_v();
      test_perspective_v();
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

