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

