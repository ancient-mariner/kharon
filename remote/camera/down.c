#include <stdio.h>
#include <stdint.h>

static void blur_image(
      /* in     */ const uint8_t * restrict src,
      /*    out */       uint8_t * restrict dest,
      /* in     */ const uint32_t width,
      /* in     */ const uint32_t height
      )
{
   // perform 3x3 blurring kernel to image before pushing to accumulator
   const uint32_t kern[3][3] = 
         {{ 1, 2, 1},
          { 2, 4, 2},
          { 1, 2, 1}};
   // border pixels ignored, so don't need to blur those
   for (uint32_t y=1; y<height-1; y++) {
      const uint32_t idx_0 = (uint32_t) ((y-1) * width) - 1;
      for (uint32_t x=1; x<width-1; x++) {
         uint32_t sum = 0;
         uint32_t idx = idx_0 + x;
         sum = (uint32_t) (sum + src[idx] * kern[0][0]);
         sum = (uint32_t) (sum + src[idx+1] * kern[0][1]);
         sum = (uint32_t) (sum + src[idx+2] * kern[0][2]);
         //
         idx = (uint32_t) (idx + width);
         sum = (uint32_t) (sum + src[idx] * kern[1][0]);
         sum = (uint32_t) (sum + src[idx+1] * kern[1][1]);
         sum = (uint32_t) (sum + src[idx+2] * kern[1][2]);
         //
         idx = (uint32_t) (idx + width);
         sum = (uint32_t) (sum + src[idx] * kern[2][0]);
         sum = (uint32_t) (sum + src[idx+1] * kern[2][1]);
         sum = (uint32_t) (sum + src[idx+2] * kern[2][2]);
         // 
         dest[idx_0 + x + 1 + width] = (uint8_t) (sum / 16);
      }
   }
}


// simple downsample, averaging each 2x2 bin to produce output
// if bluring is necessary then it must be done separately
static void downsample_nonblur(
      /* in     */ const uint8_t * restrict src,
      /*    out */       uint8_t * restrict dest,
      /* in     */ const uint32_t src_w,
      /* in     */ const uint32_t src_h,
      /* in     */ const uint32_t dest_w,
      /* in     */ const uint32_t dest_h
      )
{
   const uint32_t local_dest_w = dest_w;
   const uint32_t local_dest_h = dest_h;
   const uint32_t local_src_w = src_w;
   uint8_t *dest_buf = dest;
   for (uint32_t y=0; y<local_dest_h; y++) {
      uint32_t src_idx_top = 2 * y * local_src_w;
      uint32_t src_idx_bottom = src_idx_top + local_src_w;
      for (uint32_t x=0; x<local_dest_w; x++) {
         uint32_t tot = (uint32_t) (src[src_idx_top] + 
                                    src[src_idx_top+1] +
                                    src[src_idx_bottom] + 
                                    src[src_idx_bottom+1]);
         src_idx_top += 2; 
         src_idx_bottom += 2;
         *dest_buf++ = (uint8_t) ((tot + 2) / 4);
      }
   }
}

// downsamples then applies 3x3 blurring kernel
static void downsample(
      /* in     */ const uint8_t * restrict src,
      /*    out */       uint8_t * restrict tmp_buf,
      /*    out */       uint8_t * restrict dest,
      /* in     */ const uint32_t src_w,
      /* in     */ const uint32_t src_h,
      /* in     */ const uint32_t dest_w,
      /* in     */ const uint32_t dest_h
      )
{
   // TODO explain why image not blurred first???
#warning "Evaluate blurring approach. seems buggy and uses obsolete approach"
   downsample_nonblur(src, tmp_buf, src_w, src_h, dest_w, dest_h);
   blur_image(tmp_buf, dest, dest_w, dest_h);
}

// copy image, stripping unused YUV cols/rows
static void copy_image(
      /* in     */       uint8_t * restrict src,
      /*    out */       uint8_t * restrict dest,
      /* in     */ const uint32_t src_w,
      /* in     */ const uint32_t src_h,
      /* in     */ const uint32_t dest_w,
      /* in     */ const uint32_t dest_h
      )
{
   const uint32_t local_dest_w = dest_w;
   const uint32_t local_dest_h = dest_h;
   const uint32_t local_src_w = src_w;
   for (uint32_t y=0; y<local_dest_h; y++) {
      memcpy(&dest[y * local_dest_w], &src[y * local_src_w], local_dest_w);
   }
}

#if defined(TEST_DOWNSAMPLE)

const uint8_t img[] = {
   1,    1,    1,    1,    1,  128,    1,    1,
   1,  128,  128,    1,   64,   64,    1,    1,
   1,    1,    1,    1,   64,   64,    1,    1,
   1,    1,    1,    1,    1,  128,  128,  128,
   1,    1,  128,  128,    1,  128,  128,    1,
   0,    1,  128,  128,    1,   64,    1,    1,
   0,    0,    0,    1,    1,   64,    1,    1,
   0,    1,    1,    0,    1,   64,    1,    1
};

const uint8_t expected[] = {
  33,   33,   64,    1,
   1,    1,   64,   65,
   1,  128,   49,   33,
   0,    1,   33,    1 
};


static uint32_t test_downsample(void)
{
   printf("Testing downsample non-blur\n");
   uint32_t errs = 0;
   ////////////
   uint8_t down[16];
   downsample_nonblur(img, down, 8, 8);
   for (uint32_t i=0; i<16; i++) {
      if (down[i] != expected[i]) {
         errs++;
      }
   }
   if (errs > 0) {
      printf("Calculated values are different than expected\n");
      printf("original\n");
      for (uint32_t i=0; i<64; i++) {
         printf("%6d", img[i]);
         if ((i % 8) == 7)
            printf("\n");
      }
      printf("Expected\n");
      for (uint32_t i=0; i<16; i++) {
         printf("%6d", expected[i]);
         if ((i % 4) == 3)
            printf("\n");
      }
      printf("Calculated\n");
      for (uint32_t i=0; i<16; i++) {
         printf("%6d", down[i]);
         if ((i % 4) == 3)
            printf("\n");
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


int main(int argc, char** argv)
{
   uint32_t errs = 0;
   errs += test_downsample();
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

#endif   // TEST_DOWNSAMPLE
