#include "blur.h"
#include <string.h>
#include <assert.h>
#include <stdio.h>
//
#include <stdlib.h>
#include <stdint.h>

void blur_5x5(
      /* in     */ const uint8_t *orig, 
      /*    out */       uint8_t *blurred,
      /* in     */ const image_size_type size
      )
{
   int32_t c = size.cols;
   int32_t offset[25] = {
      -2*c-2,  -2*c-1,  -2*c  ,  -2*c+1,  -2*c+2,
      -1*c-2,  -1*c-1,  -1*c  ,  -1*c+1,  -1*c+2,
          -2,      -1,       0,      +1,      +2,
       1*c-2,   1*c-1,   1*c  ,   1*c+1,   1*c+2,
       2*c-2,   2*c-1,   2*c  ,   2*c+1,   2*c+2,
      
   };
   uint32_t weight[25] = {
      1,   4,   7,   4,   1,
      4,  16,  26,  16,   4,
      7,  26,  41,  26,   7,
      4,  16,  26,  16,   4,
      1,   4,   7,   4,   1
   };
   ////////////////////////////////
   const uint32_t TOT_WEIGHT = 273;
   // If weight matrix changes, uncomment the following code to verify
   //    TOT_WEIGHT is correct and offset is at least balanced
   //uint32_t tot=0;
   //for (uint32_t i=0; i<25; i++) {
   //   tot = tot + weight[i];
   //   printf("%4d ", weight[i]);
   //   if ((i%5) == 4)
   //      printf("\n");
   //}
   //printf("tot=%d\n", tot);
   //assert(tot == TOT_WEIGHT);
   //int32_t off = 0;
   //for (uint32_t i=0; i<25; i++) {
   //   off += offset[i];
   //   printf("%4d ", offset[i]);
   //   if ((i%5) == 4)
   //      printf("\n");
   //}
   //assert(off == 0);
   ////////////////////////////////
   const uint32_t n_pix = (uint32_t) (size.x * size.y);
   memcpy(blurred, orig, n_pix);
   for (int32_t y=2; y<size.y-2; y++) {
      const uint32_t row_offset = (uint32_t) y * size.cols;
      for (int32_t x=2; x<size.x-2; x++) {
         const uint32_t base_idx = (uint32_t) x + row_offset;
         uint32_t sum = 0;
         for (uint32_t i=0; i<25; i++) {
            const uint32_t idx = (uint32_t) ((int32_t) base_idx + offset[i]);
            sum += weight[i] * orig[idx];
         }
         blurred[base_idx] = (uint8_t) (sum / TOT_WEIGHT);
      }
   }
}


////////////////////////////////////////////////////////////////////////
// next generation of blurring. should be more efficient plus it
//    handles boundaries

void blur_image_r2(
      /* in     */ const uint8_t * restrict src,
      /*    out */       unsigned int * restrict tmp,
      /*    out */       uint8_t * restrict dest,
      /* in     */ const image_size_type size
      )
{
   /////////////////////////////////////////////////////////////////////
   // horizontal blur
   {
      unsigned int left1, left0, center, right0, right1;
      for (unsigned int y=0; y<size.height; y++) {
         unsigned int ctr_idx = y * size.width;
         ////////////////////////////////////////////////
         // far left col
         center = src[ctr_idx];
         right0 = src[ctr_idx+1];
         right1 = src[ctr_idx+2];
         tmp[ctr_idx] = (11*center + 4*right0 + right1) >> 4;
         ////////////////////////////////////////////////
         // 2nd left col
         ctr_idx++;
         left0 = center;
         center = right0;
         right0 = right1;
         right1 = src[ctr_idx+2];
         tmp[ctr_idx] = (5*left0 + 6*center + 4*right0 + right1) >> 4;
         ////////////////////////////////////////////////
         // middle cols
         for (int x=2; x<size.width-2; x++) {
            ctr_idx++;
            left1 = left0;
            left0 = center;
            center = right0;
            right0 = right1;
            right1 = src[ctr_idx+2];
            tmp[ctr_idx] = 
                  ((left1 + 4*left0 + 6*center + 4*right0 + right1) >> 4);
         }
         ////////////////////////////////////////////////
         // 2nd right col
         ctr_idx++;
         left1 = left0;
         left0 = center;
         center = right0;
         right0 = src[ctr_idx+1];
         tmp[ctr_idx] = (left1 + 4*left0 + 6*center + 5*right0) >> 4;
         ////////////////////////////////////////////////
         // right col
         ctr_idx++;
         left1 = left0;
         left0 = center;
         center = src[ctr_idx];
         tmp[ctr_idx] = (left1 + 4*left0 + 11*center) >> 4;
      }
   }
   /////////////////////////////////////////////////////////////////////
   // vertical blur
   {
      unsigned int bot1, bot0, center, top0, top1;
      for (unsigned int x=0; x<size.width; x++) {
         unsigned int ctr_idx = x;
         ////////////////////////////////////////////////
         // top row
         center = tmp[ctr_idx];
         bot0 = tmp[ctr_idx+size.width];
         bot1 = tmp[ctr_idx+2u*size.width];
         dest[ctr_idx] = (uint8_t) ((11*center + 4*bot0 + bot1) >> 4);
         ////////////////////////////////////////////////
         // 2nd row
         ctr_idx += size.width;
         top0 = center;
         center = bot0;
         bot0 = bot1;
         bot1 = tmp[ctr_idx+2u*size.width];
         dest[ctr_idx] = (uint8_t) ((5*top0 + 6*center + 4*bot0 + bot1) >> 4);
         ////////////////////////////////////////////////
         // middle rows
         for (int y=2; y<size.height-2; y++) {
            ctr_idx += size.width;
            top1 = top0;
            top0 = center;
            center = bot0;
            bot0 = bot1;
            bot1 = tmp[ctr_idx+2u*size.width];
            dest[ctr_idx] = (uint8_t) 
                  ((top1 + 4*top0 + 6*center + 4*bot0 + bot1) >> 4);
         }
         ////////////////////////////////////////////////
         // 2nd bottom row
         ctr_idx += size.width;
         top1 = top0;
         top0 = center;
         center = bot0;
         bot0 = tmp[ctr_idx+size.width];
         dest[ctr_idx] = (uint8_t) ((center + top0) >> 1);
         dest[ctr_idx] = (uint8_t) ((top1 + 4*top0 + 6*center + 5*bot0) >> 4);
         ////////////////////////////////////////////////
         // bottom row
         ctr_idx += size.width;
         top1 = top0;
         top0 = center;
         center = tmp[ctr_idx];
         dest[ctr_idx] = (uint8_t) ((top1 + 4*top0 + 11*center) >> 4);
      }
   }
}

void blur_image_r1(
      /* in     */ const uint8_t * restrict src,
      /*    out */       unsigned int * restrict tmp,
      /*    out */       uint8_t * restrict dest,
      /* in     */ const image_size_type size
      )
{
   /////////////////////////////////////////////////////////////////////
   // horizontal blur
   {
      unsigned int left0, center, right0;
      for (unsigned int y=0; y<size.height; y++) {
         unsigned int ctr_idx = y * size.width;
         ////////////////////////////////////////////////
         // left col
         center = src[ctr_idx];
         right0 = src[ctr_idx+1];
         tmp[ctr_idx] = (center + right0) >> 1;
         ++ctr_idx;
         ////////////////////////////////////////////////
         // middle cols
         for (int x=1; x<size.width-1; x++) {
            left0 = center;
            center = right0;
            tmp[ctr_idx] = ((left0 + 2*center + right0) >> 2);
            ++ctr_idx;
            right0 = src[ctr_idx];
         }
         ////////////////////////////////////////////////
         // right col
         left0 = center;
         center = right0;
         tmp[ctr_idx] = (left0 + center) >> 1;
      }
   }
   /////////////////////////////////////////////////////////////////////
   // vertical blur
   {
      unsigned int top0, center, bot0;
      for (unsigned int x=0; x<size.width; x++) {
         unsigned int ctr_idx = x;
         ////////////////////////////////////////////////
         // top row
         center = tmp[ctr_idx];
         bot0 = tmp[ctr_idx+size.width];
         dest[ctr_idx] = (uint8_t) ((center + bot0) >> 1);
         ctr_idx += size.width;
         ////////////////////////////////////////////////
         // middle rows
         for (int y=1; y<size.height-1; y++) {
            top0 = center;
            center = bot0;
            dest[ctr_idx] = (uint8_t) ((top0 + 2*center + bot0) >> 2);
            ctr_idx += size.width;
            bot0 = tmp[ctr_idx];
         }
         ////////////////////////////////////////////////
         // bottom row
         top0 = center;
         center = bot0;
         dest[ctr_idx] = (uint8_t) ((center + top0) >> 1);
      }
   }
}


// downsample 2D array 'orig' to 'down'
// each downsmple pixel is weighed average of pixels in original
//    array equivalent to if original array was convolved with 
//    Gaussian-like kernel.
// if original array were convolved with the 5x5 Gaussian kernel:
//
//   1   4   7   4   1
//   4  16  26  16   4
//   7  26  41  26   7
//   4  16  26  16   4
//   1   4   7   4   1
//  
//   divided by 273
//
// then the downsampled values would be the values in the original 
//    array weighted by the following:
//
//    1   5  11  11   5   1
//    5  25  53  53  25   5
//   11  53 109 109  53  11
//   11  53 109 109  53  11
//    5  25  53  53  25   5
//    1   5  11  11   5   1
//
//    divided by 1092
//
// with the 109 values corresponding to the pixels in the original space
//    that reduce down to a single pixel in the downsampled space
//
// this kernel can be simplified to:
//
//    1 2 2 1
//    2 4 4 2
//    2 4 4 2
//    1 2 2 1
//
//    divided by 36
//
// which can be further simplifed to:
//
//      1 1  
//    1 2 2 1
//    1 2 2 1
//      1 1   
//
//    divided by 16

//// TODO evaluate and write test code for this
//void downsample(
//      /* in     */   uint8_t *orig, 
//      /*    out */   uint8_t *down, 
//      /* in     */   const image_size_type orig_size)
//{
//   for (int r=0; r<orig_size.rows; r+=2) {
//      for (int c=0; c<orig_size.cols; c+=2) {
//         int cnt = 0;   // number of pixels sampled
//         float sum = 0.0;
//         int idx = (c-1) + (r-1)*orig_size.cols;
//         // idx is top-left of pixel of blurring mask (X in following):
//         // X 1 1 
//         // 1 2 2 1
//         // 1 2 2 1
//         //   1 1
//         // where '2' area is section corresponding to downsampled pixel
//         int cnt = 8;   // 8 for central region
//         int sum = 0;
//         if (r > 0) {
//            // not top row of image, so can use top row of mask
//            sum += orig[idx+1] + orig[idx+2];
//            cnt += 2;
//         }
//         if (r < (orig_rows-1)) {
//            // not bottom row so can use last row of mask
//            int pos = idx + 3 * orig_rows + 1;
//            sum += orig[pos] + orig[pos + 1];
//            cnt += 2;
//         }
//         if (c > 0) {
//            // not left column so can use left col of mask
//            sum += orig[idx + orig_rows] + orig[idx + 2*orig_rows];
//            cnt += 2;
//         }
//         if (c > (orig_cols-1)) {
//            // not far right column so can use right col of mask
//            int pos = idx + orig_cols + 3;
//            sum += orig[pos] + orig[pos + orig_cols];
//            cnt += 2;
//         }
//         int pos = idx + orig_cols + 1;
//         sum += orig[pos] + orig[pos+1];
//         pos += orig_cols;
//         sum += orig[pos] + orig[pos+1];
//      }
//   }
//}

//// TODO evaluate and write test code for this
//void downsample(
//      /* in     */   uint8_t *orig, 
//      /*    out */   uint8_t *down, 
//      /* in     */   int orig_rows, 
//      /* in     */   int orig_cols)
//{
//   for (int r=0; r<orig_rows; r+=2) {
//      for (int c=0; c<orig_cols; c+=2) {
//         int cnt = 0;   // number of pixels sampled
//         float sum = 0.0;
//         int idx = (c-1) + (r-1)*orig_cols;
//         // idx is top-left of pixel of blurring mask (X in following):
//         // X 1 1 
//         // 1 2 2 1
//         // 1 2 2 1
//         //   1 1
//         // where '2' area is section corresponding to downsampled pixel
//         int cnt = 8;   // 8 for central region
//         int sum = 0;
//         if (r > 0) {
//            // not top row of image, so can use top row of mask
//            sum += orig[idx+1] + orig[idx+2];
//            cnt += 2;
//         }
//         if (r < (orig_rows-1)) {
//            // not bottom row so can use last row of mask
//            int pos = idx + 3 * orig_rows + 1;
//            sum += orig[pos] + orig[pos + 1];
//            cnt += 2;
//         }
//         if (c > 0) {
//            // not left column so can use left col of mask
//            sum += orig[idx + orig_rows] + orig[idx + 2*orig_rows];
//            cnt += 2;
//         }
//         if (c > (orig_cols-1)) {
//            // not far right column so can use right col of mask
//            int pos = idx + orig_cols + 3;
//            sum += orig[pos] + orig[pos + orig_cols];
//            cnt += 2;
//         }
//         int pos = idx + orig_cols + 1;
//         sum += orig[pos] + orig[pos+1];
//         pos += orig_cols;
//         sum += orig[pos] + orig[pos+1];
//      }
//   }
//}

////////////////////////////////////////////////////////////////////////
#if defined(TEST_BLUR)
#include <stdio.h>


const uint8_t ORIG[256] = {
 255, 255, 255, 255, 255,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
 255, 255, 255, 255, 255,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
 255, 255, 255, 255, 255,   0,   0,   0,   0,   0,   0,   0,   0,   0, 137,   0,
 255, 255, 255, 255, 255,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
 255, 255, 255, 255, 255,   0,   0,   0,   0, 137,   0,   0,   0,   0,   0,   0,
 255, 255, 255, 255, 255,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
   0,   0,   0,   0,   0,   0,   0,   0,   0, 255,   0,   0,   0,   0,   0,   0,
 100, 100, 100, 100, 100, 100,   0,   0,   0, 255,   0,   0,   0,   0,   0,   0,
 100, 100, 100, 100, 100, 100,   0,   0,   0, 255, 255, 255, 255, 255, 255, 255,
 100, 100, 100, 100, 100, 100,   0,   0,   0, 255, 255, 255, 255, 255, 255, 255,
 100, 100, 100, 100, 100, 100,   0,   0,   0, 255,   0,   0,   0,   0,   0,   0,
 100, 100, 100, 100, 100, 100,   0,   0,   0, 255,   0,   0,   0,   0,   0,   0
};

const uint8_t BLUR[256] = {
 255, 255, 255, 255, 255, 255,   0,   0,   0, 128,   0,   0,   0,   0,   0,   0,
 255, 255, 255, 255, 255, 255,   0,   0,   0, 128,   0,   0,   0,   0, 100,   0,
 255, 255, 255, 255, 255, 255,   0,   0,   0, 128,   0,   0,   0,   0,   0,   0,
 255, 255, 255, 255, 255, 255,   0,   0,   0, 128,   0,   0,   0,   0,   0,   0,
 255, 255, 255, 255, 255, 255,   0,   0,   0, 128,   0,   0,   0,   0,   0,   0,
 255, 255, 255, 255, 255, 255,   0,   0,   0, 128,   0,   0,   0,   0,   0,   0,
   0,   0,   0,   0,   0,   0,   0,   0,   0, 255,   0,   0,   0,   0,   0,   0,
   0,   0,   0,   0,   0,   0,   0,   0,   0, 255,   0,   0,   0,   0,   0,   0,
   0,   0,   0,   0,   0,   0,   0,   0,   0, 255,   0,   0,   0,   0,   0,   0,
   0,   0,   0,   0,   0,   0,   0,   0,   0, 255,   0,   0,   0,   0,   0,   0,
   0,   0,   0,   0,   0,   0,   0,   0,   0, 255,   0,   0,   0,   0,   0,   0,
 100, 100, 100, 100, 100, 100,   0,   0,   0, 255, 255, 255, 255, 255, 255, 255,
 100, 100, 100, 100, 100, 100,   0,   0,   0, 255, 255, 255, 255, 255, 255, 255,
 100, 100, 100, 100, 100, 100,   0,   0,   0, 255, 255, 255, 255, 255, 255, 255,
 100, 100, 100, 100, 100, 100,   0,   0,   0, 255,   0,   0,   0,   0,   0,   0,
 100, 100, 100, 100, 100, 100,   0,   0,   0, 255,   0,   0,   0,   0,   0,   0
};

static uint32_t test_blur_5x5(void)
{
   uint32_t errs = 0;
   printf("Testing blur_5x5\n");
   //
   uint8_t out[256];
   memset(out, 0, 256);
   const image_size_type sz = { .x=16, .y=16 };
   blur_5x5(ORIG, out, sz);
for (uint32_t i=0; i<256; i++) {
   printf("%3d  ", out[i]);
   if ((i % 16) == 15)
      printf("\n");
}
   //
   if (errs == 0) {
      printf("    passed\n");
   } else {
      printf("    %d errors\n", errs);
   }
   return errs;
}

int main(int argc, char** argv) {
   (void) argc;
   (void) argv;
   uint32_t errs = 0;
   errs += test_blur_5x5();
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

#endif   // TEST_BLUR
