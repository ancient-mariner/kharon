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
#if !defined(ACCUMULATOR_H)
#define ACCUMULATOR_H
#include <stdint.h>

// approx field of view of camera. note that the number of pixels on
//    each dimension don't fully align with using only 3 significant
//    digits here (e.g., actual width may be 62.22 or 62.17)
// this is close enough though. the code should commpensate for
//    irregularities
#define FOV_WIDTH_DEGREES    62.2f
#define FOV_HEIGHT_DEGREES   48.8f

////////////////////
// output resolution

// at 2x2 pixel binning on RPi cam 2, central pixels are approximately
//    the size of accumulator elements when PPD is 12. However, the
//    pixels don't necessarily align with the elements so there will
//    be some bluring. blurring down automatically now before accumulation,
//    so this should no longer be a factor
// examination of output images, with one-pixel wide diagonal lines
//    running across input image, shows that PPD 10 and 12 provide good
//    results when 3x3 blurring kernel is used before accumulation.
//    using higher PPD is possible. centeral region of image will
//    be upsampled if using PPD>12, but peripheral regions will maintain
//    more of the detail available there
#define PIX_PER_DEGREE_BASE     12

// manually calculated values from above constants
// values calculated manually so are clearly known and so they
//    can be duplicated in kernel code
// for pix-per-degree == 15, necessary size is 870.8 x 683.2
// for pix-per-degree == 14, necessary size is 870.8 x 683.2
// for pix-per-degree == 13, necessary size is 808.6 x 634.4
// for pix-per-degree == 12, necessary size is 746.4 x 585.6
// for pix-per-degree == 12, necessary size is 746.4 x 585.6
// for pix-per-degree == 11, necessary size is 684.2 x 536.8
// for pix-per-degree == 10, necessary size is 622 x 488
// round up to next nearest multiple of 10 (for convenience)
#if PIX_PER_DEGREE_BASE == 15
#define PERSP_WIDTH       933.0f
#define PERSP_HEIGHT      732.0f
#define PERSP_WIDTH_SIZE   940
#define PERSP_HEIGHT_SIZE  740
// 940 * 740 = 695600

#elif PIX_PER_DEGREE_BASE == 14
#define PERSP_WIDTH       870.8f
#define PERSP_HEIGHT      683.2f
#define PERSP_WIDTH_SIZE   880
#define PERSP_HEIGHT_SIZE  690
// 880 x 690 = 607200

#elif PIX_PER_DEGREE_BASE == 13
#define PERSP_WIDTH       808.6f
#define PERSP_HEIGHT      634.4f
#define PERSP_WIDTH_SIZE   810
#define PERSP_HEIGHT_SIZE  640
// 810 x 640 = 518400

#elif PIX_PER_DEGREE_BASE == 12
#define PERSP_WIDTH       746.4f
#define PERSP_HEIGHT      585.6f
#define PERSP_WIDTH_SIZE   750
#define PERSP_HEIGHT_SIZE  590
// 750 x 590 = 442500

#elif PIX_PER_DEGREE_BASE == 11
#define PERSP_WIDTH       684.2f
#define PERSP_HEIGHT      536.8f
#define PERSP_WIDTH_SIZE   690
#define PERSP_HEIGHT_SIZE  540
// 690 x 540 = 372600

#elif PIX_PER_DEGREE_BASE == 10
#define PERSP_WIDTH       622.0f
#define PERSP_HEIGHT      488.0f
#define PERSP_WIDTH_SIZE   630
#define PERSP_HEIGHT_SIZE  490
// 690 x 540 = 308700

#elif PIX_PER_DEGREE_BASE == 5
#define PERSP_WIDTH       311.0f
#define PERSP_HEIGHT      244.0f
#define PERSP_WIDTH_SIZE   320
#define PERSP_HEIGHT_SIZE  250
#endif   // PIX_PER_DEGREE_BASE

// last row and column of accumulator are ignored
#define ACCUM_WIDTH     (PERSP_WIDTH_SIZE + 1)
#define ACCUM_HEIGHT    (PERSP_HEIGHT_SIZE + 1)


// accumulator is used when remapping image, to bring together weighted
//    values of source pixels projecting to destination pixel locations
// stores all values in the world pixel type, plus weight
union accumulator_element {
   struct {
      uint32_t val;
      uint16_t border; // border flag
      uint16_t w; // total input
   };
};
typedef union accumulator_element accumulator_element_type;

// represents how a pixel maps onto accumulator elements. the x,y
//    is the top-left accumulator element (northwest). the positional
//    values (nw, ne, sw, se) are the weights of how the input pixel
//    is distributed among the 4 neighboring accumulator elements
struct accumulator_coord {
   uint16_t x;  // west
   uint16_t y;  // north
   uint8_t nw;
   uint8_t ne;
   uint8_t sw;
   uint8_t se;
};
typedef struct accumulator_coord accumulator_coord_type;

/* Converting provided undistorted image to perspective view

Camera provides image that has been 'undistorted' in the sense that
parallel lines in the real world are represented as parallel lines
in the camera image. This is most apparent viewing a checkerboard or
graph that is perpendicular to the camera's view. While the squares
closer to the camera have are relatively larger in their field of
view, versus squares further from the camera, all are displayed with
equal size and dimension.

An analogy, used here, is a brick wall. Imagine a brick wall with
a horizontal white line painted on it 4' off the ground, for the
length of the fence. A camera is pointed directly at the wall at
an elevation of 4'. The center of the image is on the white line
at location N, with a line orthogonal to the fence at N passing
through the camera (C)

In an 'undistored' image of the fence, all bricks will appear of
equal size, whether they be closest to the camera (near N) or
in the periphery. Undistored images cannot be readily stitched
because they do not reflect what is actually seen from C.

To stitch images together, the image must be transformed to a
'perspective' view, where each pixel in the image represents a
radial arc of the light received from the camera from w/in that
arc (as opposed to each pixel representing a physical distance, as
in the undistorted image)

To generate a perspective image, each pixel in the undistored image
must be remapped and projected.

Let K be the distance between C and N, in units of the physical
distance represented by each pixel in the undistored image:

   K_x = (ING_WIDTH / 2) / tan(FOV_W / 2)
   K_y = (IMG_HEIGHT / 2) / tan(FOV_H / 2)

K_x and K_y should be equal, with K = K_x = K_y.

To determine the remapped position P' of a single pixel P in the source
image, let x and y be the distance from N to P along X and Y axes.

Let J be the distance from C to P:

   J = sqrt(K^2 + x^2 + y^2)

   P_x' = asin(x / J) * IMG_WIDTH / FOV_W
   P_y' = asin(y / J) * IMG_HEIGHT / FOV_H

so P' = (P_x', P_y')

*/

#endif   // ACCUMULATOR_H
