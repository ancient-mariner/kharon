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

#define FOV_WIDTH_DEGREES    62.2f
#define FOV_HEIGHT_DEGREES   48.8f

// output resolution
// at 2x2 pixel binning on RPi cam 2, central pixels are approximately
//    the size of accumulator elements when PPD is 12. However, the
//    pixels don't necessarily align with the elements so there will
//    be some bluring
// at PPD=14, pixel size is smaller than element size, so there's some
//    blurring due expansion in the center, but the mapping looks like
//    the blurring shouldn't be much worse than PPD=12
// at PPD=15, a single-pixel diagonal line has breaks in the test image
//    (this might be partially fixed by blurring before edge detection)
// at PPD=12, there are small gaps but w/ improved uniformity betw
//    pixels (pixel intensity was highly variable w/ PPD=15)
// at PPD=10, there are a handful of small gaps and better uniformity
// when blurring before edge detection, gaussian radius=2 works well with
//    PPD<=11, and OK w/ 12
//    radius=1 works well w/ PPD=5 (akin to full-frame image and PPD=10)
// testing done with 1/4 resolution V channel
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
in the camera image. A view of the real world has perspective though,
with parallel lines converging in the distance. In order to stitch
images together, images must be converted to perspective view, with
the image representing what's viewed in the real world.

A second distortion in the provided image is that the distance between
parallel lines does not decrease with increasing distance from the
point of focus.

To explain each distortion, imagine an infinitely long fence with a
checkerboard painted on it. Let the line where the fence touches the
ground cross through the image center. That line will stretch left
and right to the edge of the image and will remain straight, both in
the image and also the perspective view of it.

Let the nearest point on the fence be point N, with a line orthogonal
to the fence from N passing through the camera, with the camera
centered at the point where N meets the ground, and the camera being
at ground level. N is at the image center.

Let the height of the fence, H, align with the top the image frame,
such that the height of the fence is half of the vertical field of
view (FOV_H) of the camera, or FOV_H/2.

Assume that the checkerboard painted on the fence is the size such
that a grid square is one degree in horizontal and vertical FOV
as seen from the camera at N.

The 'undistorted' image of the fence will show the section of the
fence between +/- FOV_W/2 from N. The top of the fence in this image
will parallel the top of the camera frame. A perspective view would
have the height of the fence at left and right be of lower relative
height than the center.

Another feature of the image is that each checkerboard in the
'undistorted' image is of equal size, whereas a proper perspective
view would have grid squares at the image edges being narrower than
what's visible in the image center.


Two corrections are seen as necessary to obtain a perspective view
from an 'undistorted' image. The first is parallel lines converging
with distance, and the second is the narrowing of the distance
between parallel lines with increasing distance from the image center.

To correct for perspective, the height of the fence at a given
position in the image will be adjusted to its correct perspective
location. Let H be the height of the fence at N. H' is the perspective
height of the fence at a given lateral position in the field of view.

   H' = H * cos(theta)

for theta on [-FOV_W/2, FOV_W/2]. Because the checkerboard pattern
of the fence is normalized in the undistorted image, with each
checkerboard being of equal size in the image, theta can be measured
directly from the undistorted image:

   theta = (FOV_W / 2) * (X / (IMG_WIDTH / 2))

for pixel X on [-IMG_WIDTH/2, IMG_WIDTH/2].

The visible image height in the perspective view is bounded by H'.
The vertical placement of pixels in the visible image is controlled
by the position of a pixel relative to the image top (or bottom).

To correct for horizontal lines being equidistant, as opposed to
getting closer with vertical distance from image center, the relative
vertical position of each pixel is adjusted.
There is no movement at the image center (the origin) and there's no
relative movement at the image edges, as content there is at the field
of view of the image. Intermediate pixels are shifted outward towards the
edge.

Let V be the vertical position of a pixel in the source image, and
V' the position in the perspective image. Vnorm' is the normalized
position of V', with 1.0 being at the image edge and 0 at the origin.

In the source image, the number of pixels from image center (N) to
V is proportional to the physical distance. This is the undistored
image. Each pixel in the perspective view represents a fixed arc.
To go from undistored to perspective, image pixels must be converted
from distance to angle.

angle

Let
   // K is distance from camera to nearest point on fence
   //    (i.e., N at image center)
   K = (IMG_HEIGHT / 2) / tan(FOV_H / 2)

   // W is the
   // W is degrees in elevation from N to viewed position
   tan(W) = V / K
   W = atan(V / K)

   V' = W * IMG_HEIGHT / FOV_HEIGHT
   Vnorm' = W / (FOV_HEIGHT / 2)

The projected vertical position of the pixel in perspective space,
Y', is
   Y' = Vnorm' * H' * PERSP_PIX_H / PIX_H

We must use Vnorm' as H' is already in pixels, so we only want the
relative height that would display and use that to scale H'.

The same process is used to get the horizontal position of pixels
in perspective space, exchanging width and height values as appropriate.

*/


#endif   // ACCUMULATOR_H
