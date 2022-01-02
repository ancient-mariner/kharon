#include "pin_types.h"
#include "pixel_types.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <assert.h>
#include "datap.h"
#include "logger.h"
#include "lin_alg.h"
#include "image.h"

#include "core_modules/optical_up.h"
#include "accumulator.h"
#include "accumulator.c"

/* Rough derivation of 'undistorted' camera image to perspective view.
Image returned by camera is 'undistorted' in the sense that distances
shown in image correspond to distances measured in the world. A picture
of a rectangular grid will be displayed in the undistorted image with
each grid square being of equal size. This is not how the world is
actually viewed though, as grid squares closer to the viewer will be
larger (i.e., have a larger arc size) than grid squares further away.

What's needed for stitching images is that each image pixel represents
an arc size. Because the input image has each pixel representing physical
distance, a transform is required to convert the input image from physical
to radial distances.

As the input image represents physical distance, it can be assumed to
represent a wall that has the world view displayed on it, with distances
preserved (imagine the picture being that of a grid, or brick wall).
To convert this to spherical perspective, the angle from each pixel (X and
Y) must be calculated based on the pixel's location in the image.
One way to do this is through trigonometry (see vy_receiver's 
exp_accumulator.h for derivation of this approach). Another is to define a
sphere that intersects the "wall" at its nearest point, and take the XYZ of
each pixel (Z is constant for entire wall) and then scale each XYZ vector
to the sphere surface. The XY position on the sphere surface for each
pixel should correspond to the relative radial distance from the
image center.

*/

static const uint32_t BORDER_CHAN_IDX = NUM_IMAGE_CHANNELS;

// ideally blurring would occur in analysis planes, where algorithm there
//    could decide appropriate blurring level. however, analysis planes
//    read from panorama, and at the pan level images are merged and 
//    broken apart into fg and bg sections, making a blurring task much more
//    complex and inefficient. until pan rewritten to pass on image data
//    more directly and efficiently, do blurring here
static void blur_output_r1(
      /* in out */       optical_up_class_type *upright,
      /* in out */       optical_up_output_type *out
      )
{
   vy_blur_pixel_type *buf = upright->blur_buf;
   for (uint32_t lev=0; lev<NUM_PYRAMID_LEVELS; lev++) {
      pixel_cam_info_type *src = out->frame[lev];
      image_size_type size = out->size[lev];
      //////////////////////////////////////////////////////////////////
      // horizontal blur
      {
         pixel_cam_info_type left0, center, right0;
         for (unsigned int y=0; y<size.height; y++) {
            unsigned int ctr_idx = y * size.width;
            ////////////////////////////////////////////////
            // left col
            center = src[ctr_idx];
            right0 = src[ctr_idx+1];
            vy_blur_pixel_type *pix = &buf[ctr_idx];
            for (uint32_t i=0; i<NUM_IMAGE_CHANNELS; i++) {
               pix->channel[i] = (uint8_t) ((
                     center.color.channel[i] + 
                     right0.color.channel[i]) >> 1);
            }
            pix->channel[BORDER_CHAN_IDX] = center.border | right0.border;
            ++ctr_idx;
            ////////////////////////////////////////////////
            // middle cols
            for (int x=1; x<size.width-1; x++) {
               left0 = center;
               center = right0;
               pix = &buf[ctr_idx];
               for (uint32_t i=0; i<NUM_IMAGE_CHANNELS; i++) {
                  pix->channel[i] = (uint8_t) ((
                        left0.color.channel[i] +
                        2*center.color.channel[i] + 
                        right0.color.channel[i]) >> 2);
               }
               // border is all or none, so OR values
               pix->channel[BORDER_CHAN_IDX] = 
                     left0.border | center.border | right0.border;
               ++ctr_idx;
               right0 = src[ctr_idx];
            }
            ////////////////////////////////////////////////
            // right col
            left0 = center;
            center = right0;
            pix = &buf[ctr_idx];
            for (uint32_t i=0; i<NUM_IMAGE_CHANNELS; i++) {
               pix->channel[i] = (uint8_t) ((
                     left0.color.channel[i] + 
                     center.color.channel[i]) >> 1);
            }
            pix->channel[BORDER_CHAN_IDX] = left0.border | center.border;
         }
      }
      /////////////////////////////////////////////////////////////////////
      // vertical blur
      pixel_cam_info_type *dest = out->frame[lev];
      {
         vy_blur_pixel_type top0, center, bot0;
         for (int x=0; x<size.width; x++) {
            uint32_t ctr_idx = (uint32_t) x;
            ////////////////////////////////////////////////
            // top row
            center = buf[ctr_idx];
            bot0 = buf[ctr_idx+size.width];
            pixel_cam_info_type *pix = &dest[ctr_idx];
            for (uint32_t i=0; i<NUM_IMAGE_CHANNELS; i++) {
               pix->color.channel[i] = (uint8_t) ((
                     center.channel[i] + 
                     bot0.channel[i]) >> 1);
            }
            pix->border = center.channel[BORDER_CHAN_IDX] |
                  bot0.channel[BORDER_CHAN_IDX];
            ctr_idx += size.width;
            ////////////////////////////////////////////////
            // middle rows
            for (int y=1; y<size.height-1; y++) {
               top0 = center;
               center = bot0;
               pix = &dest[ctr_idx];
               for (uint32_t i=0; i<NUM_IMAGE_CHANNELS; i++) {
                  dest->color.channel[i] = (uint8_t) ((
                        top0.channel[i] +
                        2*center.channel[i] + 
                        bot0.channel[i]) >> 2);
               }
               pix->border = top0.channel[BORDER_CHAN_IDX] |
                     center.channel[BORDER_CHAN_IDX] |
                     bot0.channel[BORDER_CHAN_IDX];
               ctr_idx += size.width;
               bot0 = buf[ctr_idx];
            }
            ////////////////////////////////////////////////
            // bottom row
            top0 = center;
            center = bot0;
            pix = &dest[ctr_idx];
            for (uint32_t i=0; i<NUM_IMAGE_CHANNELS; i++) {
               pix->color.channel[i] = (uint8_t) 
                     ((top0.channel[i] + center.channel[i]) >> 1);
            }
            pix->border = top0.channel[BORDER_CHAN_IDX] |
                  center.channel[BORDER_CHAN_IDX];
         }
      }
   }
}


// pushes content from (non-normalized) accumulators to pixel arrays 
//    for v and y channels at all pyramid levels
static void flatten_accumulators(
      /* in out */       optical_up_class_type *upright,
      /* in out */       optical_up_output_type *out
      )
{
   const uint8_t camera_num = upright->camera_num;
   const pixel_cam_info_type empty_pix = {
      .color = { .v=128, .y=0 },
      .radius = 0xffff,
      .border = 255,
      .cam_num = camera_num
   };
   // process all pyramid levels
   for (uint32_t level=0; level<NUM_PYRAMID_LEVELS; level++) {
      // input source (data stored in accumulator)
      vy_accumulator_type *accum = upright->accum[level];
      vy_accumulator_element_type *elements = accum->accum;
      const image_size_type img_sz = upright->size[level];
      int32_t half_width = img_sz.x / 2;
      int32_t half_height = img_sz.y / 2;
      // sink
      pixel_cam_info_type *pix = out->frame[level];
      // copy data, setting boundary pixels to empty
      uint32_t idx = 0;
//printf("acc size: %d,%d\n", img_sz.x, img_sz.y);
      for (int32_t y=0; y<img_sz.rows; y++) {
         // flag pixels in first and last rows as empty (border)
         if ((y == 0) || (y == (img_sz.rows-1))) {
            for (uint32_t x=0; x<img_sz.cols; x++) {
               *pix = empty_pix;
               pix++;
               idx++;
            }
         } else {
            // first column is empty
            *pix = empty_pix;
            pix++;
            idx++;
            const int dy = half_height - (int) y;
            const int y2 = dy * dy;
            for (int32_t x=1; x<img_sz.cols-1; x++) {
               const vy_accumulator_element_type *acc = &elements[idx];
               const uint32_t scale = acc->w;
               // if pixel has data (w!=0) then get accum data
               // otherwise flag it as invalid
//if ((x==400) && (y==400)) {
//   printf(" accum %d,%d  scale=%d  border=%d\n", x, y, scale, acc->z);
//}
               if (scale > 0) {
                  pix->color.y = (uint8_t) (acc->y / scale);
                  pix->color.v = (uint8_t) (acc->v / scale);
                  // estimate radius from center
                  int dx = half_width - (int) x;
                  pix->radius = (uint16_t) sqrt((double) (y2 + dx * dx));
                  pix->cam_num = camera_num;
                  // adjust border to be binary on/off (255/0)
                  pix->border = (acc->z != 0) ? 255 : 0;
               } else {
                  *pix = empty_pix;
               }
               pix++;
               idx++;
            }
            // last column is empty
            *pix = empty_pix;
            pix++;
            idx++;
         }
      }
   }
}


// extracted code from raw_image_to_accumulator. pushes image to a
//    single accumulator
// pushing to accumulator takes a huge %age of CPU w/ first iteration
//    of algorithm. this is int-based approach (runs >10% faster)
static void push_image_to_accumulator(
      /* in out */       optical_up_class_type *upright,
      /* in     */ const vy_receiver_output_type *src_img,
      /* in     */ const vy_class_type *vy,
      /* in     */ const matrix_type *cam2world,
      /* in     */ const degree_type world_center_lon,
      /* in     */ const degree_type world_center_lat
      )
{
   // pixel calculated such that center of image is 0,0
   // accumulator top-left is 0,0
   // calculate the degree offset to add to each pixel so that
   //    image and accumulator coordinates align
   // positions are partitioned into nearest 1/8ths by
   //    accumulator -- mult by here once so it doesn't have to
   //    happen once for each pixel
   double ppd_x8[NUM_PYRAMID_LEVELS] = {
         8.0 * upright->pix_per_degree[0],
         8.0 * upright->pix_per_degree[1]
   };
   //
//printf("Center at lat=%f, lon=%f\n", (double) world_center->lat, (double) world_center->lon);
   for (uint32_t lev=0; lev<NUM_PYRAMID_LEVELS; lev++) {
      const vector_type *sphere_map = vy->sphere_map[lev];
      const double pix_per_degree_x8 = ppd_x8[lev];
      const uint32_t offset = src_img->chan_offset[lev];
      const vy_pixel_type *vy_pixels = &src_img->chans[offset];
      vy_accumulator_type * accum = upright->accum[lev];
      //
      uint32_t num_rows = src_img->img_size[lev].rows;
      uint32_t num_cols = src_img->img_size[lev].cols;
      uint32_t idx = 0;
      //
      for (uint32_t y=0; y<num_rows; y++) {
         // to keep track of where borders are, so the artifacts they
         //    produce can be kept out of edge detection algorithm,
         //    an auxiliary color channel (z) is kept, with 255 being
         //    used on border pixels. any pixel with the border channel
         //    non-zero is omitted from edge detection
         int yborder = ((y==0) || (y==(num_rows-1))) ? 255 : 0;
         for (uint32_t x=0; x<num_cols; x++) {
//printf("imt %d,%d\n", x, y);
            int xborder = ((x==0) || (x==(num_cols-1))) ? 255 : 0;
            uint8_t border = (uint8_t) (xborder | yborder);
            // get position of this pixel on sphere and rotate it to
            //    where it belongs in world view
            const vector_type *sphere_pos = &sphere_map[idx];
            vector_type pix_proj;
            mult_matrix_vector(cam2world, sphere_pos, &pix_proj);
            // push pixel to accumulator
            const vy_pixel_type vy_pix = vy_pixels[idx];
            const vy_accumulator_pixel_type pix = 
                  { .v = vy_pix.v, .y = vy_pix.y };
//if ((x > 200) && (y == 300)) {
//   printf("  %d,%d has vy %d,%d, at %f,%f,%f\n", x, y, pix.v, pix.y,
//         (double) pix_proj.v[0],
//         (double) pix_proj.v[1],
//         (double) pix_proj.v[2]);
//}
            idx++;
            push_pixel_to_accumulator(pix, border, &pix_proj, 
                  world_center_lon, world_center_lat,
                  pix_per_degree_x8, accum);
//#warning "make sure pan adds center lat,lon back in (it should already do this)"
         }
      }
   }
}


// projects input image onto sphere and applies rotation to align
//    image to intermediate space. latitude and longitude of image center are
//    calculated and returned, while projected image itself is centered
//    in accumulator
// (-> equirectangular representation)
static void raw_image_to_accumulators(
      /* in out */       optical_up_class_type *upright,
      /* in     */ const vy_receiver_output_type *src_img,
      /* in     */ const attitude_output_type *ship2world,
      /* in     */ const vy_class_type *vy,
      /*    out */       sphere_coordinate32_type *world_center
      )
{
   for (uint32_t level=0; level<NUM_PYRAMID_LEVELS; level++) {
      // initialize accumulators
      clear_vy_accumulator(upright->accum[level]);
   }
   // create matrix to project camera ray traces to location in world
   //    where they originated. this is a convoluted way of saying
   //    map camera pixels to world locations 
   matrix_type cam2world;
   // rotation of A then B is B*A
   mult_matrix(&ship2world->ship2world, &upright->cam2ship, &cam2world);
   // project image center onto world view and get camera center's lat-lon
   const vector_type z_axis = { .v={0.0f, 0.0f, 1.0f } };
   vector_type z_proj;
   mult_matrix_vector(&cam2world, &z_axis, &z_proj);
   vector_to_latlon32(&z_proj, world_center);
//printf("  %f,%f  from %f,%f,%f\n", (double) world_center->longitude, (double) world_center->latitude, (double) z_proj.v[0], (double) z_proj.v[1], (double) z_proj.v[2]);
   ////////
   // cam2world matrix can be used to project each pixel to world
   //    space. because projection is a performance bottleneck, the
   //    number of operations per pixel need to be minimized.
   // pixel projection is pre-mapped onto unit sphere and cam2world
   //    is used to rotate each pixel's vector to where it should be
   //    in the world view. the lat,lon is then taken from that projection
   //    _before_ the projection of the entire image is rotated down
   //    to the equator. as lat,lon is a cylindrical coordinate system,
   //    taking lat,lon at correct spherical location allows preservation
   //    of the spherical projection, while rotating it down to the 
   //    equator allows the image to be pushed to the accumulator
   // when resulting image is applied to panoramic view, it must be
   //    rotated back up so its center is in the correct location
   degree_type world_center_lat = { .degrees = 
         world_center->lat.sangle32 * BAM32_TO_DEG };
   degree_type world_center_lon = { .degrees = 
         world_center->lon.angle32 * BAM32_TO_DEG };
   push_image_to_accumulator(upright, src_img, vy, &cam2world, 
         world_center_lon, world_center_lat);
}


static uint8_t clamp255_(double x)
{  
   if (x <= 0.0)
      return 0;
   else if (x >= 255.0)
      return 255;
   return (uint8_t) x;
}


// saves false-color pyramid image
static int32_t save_pnm_file(
      /* in out */       optical_up_class_type *optical_up,
      /* in     */ const double t,
      /* in     */ const optical_up_output_type *output
      )
{
   // output width is 1.5 image width to fit different pyramid levels
   // main image (pyramid bottom) is to left, full height. successive
   //    pyramid levels are to right
   const uint16_t IMG_ROWS = output->size[0].rows;
   const uint16_t IMG_COLS = (uint16_t) (3 * output->size[0].cols / 2);
   const uint32_t IMG_PIX  = (uint32_t) (IMG_ROWS * IMG_COLS);
   // write pyramid images
   // main image on left, sub-images stacked on right
   // this complicates writing the file, but the right-hand images will
   //    provide visual reference, at least to horizon in main image 
   // offset to top-left of displayed pyramid layer
   const uint32_t x_off[NUM_PYRAMID_LEVELS] = { 
      0, output->size[0].cols
   };
   const uint32_t y_off[NUM_PYRAMID_LEVELS] = { 
      0, 0
   };
//   const uint32_t x_off[NUM_PYRAMID_LEVELS] = { 
//      0, UPRIGHT_N_COLS_BASE, UPRIGHT_N_COLS_BASE
//   };
//   const uint32_t y_off[NUM_PYRAMID_LEVELS] = { 
//      0, 0, UPRIGHT_N_COLS_BASE/2
//   };
   /////////////////////////////////////////////////////////////////////
   // prepare pixel buffer
   if (optical_up->img == NULL) {
      image_size_type sz = { .cols=IMG_COLS, .rows=IMG_ROWS };
      optical_up->img = raw_create_image(sz);
   }
   image_type *img = optical_up->img;
   pix_type empty = { .y=0, .u=128, .v=128, .z=0 };
   for (uint32_t i=0; i<IMG_PIX; i++) {
      img->rgb[i].yuvz = empty.yuvz;
   }
   for (uint32_t lev=0; lev<NUM_PYRAMID_LEVELS; lev++) {
      // pull directly from output buffer -- this will be what subscribers
      //    see so provides a good peak into the system
      // get position at start of this level of the image pyramid
      const pixel_cam_info_type *pix = output->frame[lev];
      const image_size_type sz = output->size[lev];
      const uint32_t x_offset = x_off[lev];
      const uint32_t y_offset = y_off[lev];
      for (uint32_t y=0; y<sz.rows; y++) {
         const uint32_t in_row_idx = y * sz.cols;
         const uint32_t out_row_idx = IMG_COLS * (y_offset + y);
         for (uint32_t x=0; x<sz.cols; x++) {
            const uint32_t in_idx = in_row_idx + x;
            const uint8_t py = pix[in_idx].color.y;
            const double fy = (double) py;
            const double v = (double) (pix[in_idx].color.v - 128);
            // pix.u = 128 -> u = 0
            const uint32_t out_idx = x_offset + x + out_row_idx;
            pix_type *img_pix = &img->rgb[out_idx];
            img_pix->r = clamp255_(fy + 1.402 * v);
            img_pix->g = clamp255_(fy - 0.714 * v);
            img_pix->b = py;
         }
      }
   }
   //convert_yuv_to_rgb(img);
   //
   char path[STR_LEN];
   snprintf(path, STR_LEN, "%s/%.3f.pnm", optical_up->data_folder, t);
   log_info(optical_up->log, "Writing %s", path);
   if (write_pnm_file(path, img->size, img->rgb) != 0) {
      fprintf(stderr, "Error writing pnm file '%s'\n", path);
      return -1;
   }
   return 0;
}

////////////////////////////////////////////////////////////////////////

