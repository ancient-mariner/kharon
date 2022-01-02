#if !defined(BLUR_H)
#define BLUR_H
#include "pin_types.h"

void blur_5x5(
      /* in     */ const uint8_t *orig, 
      /*    out */       uint8_t *blurred,
      /* in     */ const image_size_type orig_size
      );

////////////////////////////////////////////////////////////////////////
// next generation of blurring algorithm. should be more efficient plus
//    it handles image edges. 

// radius 2 gaussian blur. kernel = 1 4 6 4 1
void blur_image_r2(
      /* in     */ const uint8_t * restrict src,
      /*    out */       unsigned int * restrict tmp,
      /*    out */       uint8_t * restrict dest,
      /* in     */ const image_size_type size
      );

// radius 1 gaussian blur. kernel = 1 2 1
void blur_image_r1(
      /* in     */ const uint8_t * restrict src,
      /*    out */       unsigned int * restrict tmp,
      /*    out */       uint8_t * restrict dest,
      /* in     */ const image_size_type size
      );

#endif   // BLUR_H

