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
#if !defined(OPTICAL_UP_H)
#define OPTICAL_UP_H
#include "pinet.h"
#include "logger.h"
#include "pixel_types.h"
#include "image.h"


// 'flattens' and rotates acquired images from single camera so top
//    projects to correct part of visual space. performs edge detection

// FIXME process doesn't appear to do edge detection. update docs

// (actually de-flattens by pulling image back onto sphere)
//
// takes
//    attitude data for image alignment
//    vy image (one image, max resolution)
//
// publishes color-channel image pyramids (y, v & edge) from camera
// downsampling to different pyramid levels done here
//
// with GPU, consider publishing edges for each channel


//-----------------------------

////

//#define VY_N_ROWS_BASE   VY_ROWS_NET_IMG
//#define VY_N_COLS_BASE   VY_COLS_NET_IMG
//
//// diagonal based on 62x49 degree FOV, at 10 pix/deg. we need ~794 pix.
//// safety buffer of a couple of pix (to avoid writing outside of array)
////    pushes this to ~800. the same buffer on higher pyramid levels means
////    we need a bit more than 800. make it a multiple of 8
//#define UPRIGHT_N_ROWS_BASE   808
//#define UPRIGHT_N_COLS_BASE   (UPRIGHT_N_ROWS_BASE)
//// number of pixels in top pyramid layer
//#define UPRIGHT_N_PIX_BASE   (UPRIGHT_N_ROWS_BASE * UPRIGHT_N_ROWS_BASE)

#define OPTICAL_UP_LOG_LEVEL     LOG_LEVEL_DEFAULT

struct optical_up_output {
   // camera direction relative to world coordinate frame
   sphere_coordinate32_type world_center;
   // ship's attitude and heading at time this image was taken
   //    (included here for downstream processing)
   matrix_type ship2world;
   degree_type heading;
   //
   image_size_type size[NUM_PYRAMID_LEVELS];
   // buffer that stores all pyramid levels. frame (below) has pointers
   //    into this buffer
   // size not (ie, no longer) knowable in advance as image sizes must
   //    be flexible
   pixel_cam_info_type   *pyramid_;
   // image data
   pixel_cam_info_type *frame[NUM_PYRAMID_LEVELS];
};
typedef struct optical_up_output optical_up_output_type;


// historical access is done at panorama level so we don't need a
//    large buffer here
// should be longer than frame_sync queue to ensure that frames are
//    around longer than sync process uses them (or sync process will
//    fall too far behind which _should_ induce error)
#define OPTICAL_UP_QUEUE_LEN      48

#define OPTICAL_UP_CLASS_NAME  "optical_up"

////////////////////////////////////////////////////////////////////////
//

struct vy_accumulator;  // this is defined privately in module

// when blurring images, need to blur border channel as well
// create struct that represents v and y as well as border.
struct vy_blur_pixel {
   uint8_t channel[NUM_IMAGE_CHANNELS+1];
};
typedef struct vy_blur_pixel vy_blur_pixel_type;


struct optical_up_class {
   char *data_folder;
   log_info_type *log;
   //
   // this is the only reference to upstream attitude producer.
   //    it's kept off the normal producer list as we don't
   //    care when it publishes data. we only need to contact
   //    it for historical data
   datap_desc_type *attitude_producer;
   // matrix (loaded from config) aligning camera to ship space
   matrix_type cam2ship;
   // this is the size of the output image buffers
   image_size_type size[NUM_PYRAMID_LEVELS];
   double pix_per_degree[NUM_PYRAMID_LEVELS];
   // idx in attitude list corresponding to previous uprighted frame
   // use this location as starting point for finding attitude
   //    corresponding to next frame
   // set to -1 when last position unknown (eg, startup)
   uint64_t last_attitude_idx;
   /////////////////////////////////////////////////////////////
   // sizes are same for sphere and intermediate space
   struct vy_accumulator *accum[NUM_PYRAMID_LEVELS];
   // buffer used to store intermediate blur values
   vy_blur_pixel_type *blur_buf;
   //
   image_type *img;
   // host name of device with camera
   char camera_host[MAX_NAME_LEN];
   uint8_t camera_num;
   //
   image_size_type vy_size[NUM_PYRAMID_LEVELS];
   // degree size of entire window, which is larger than FOV to account
   //    for rotation and elevation
   degree_type size_horiz;
   degree_type size_vert;
};
typedef struct optical_up_class optical_up_class_type;


// thread entry point
void * optical_up_class_init(void *);


// struct to pass config data to thread
struct optical_up_setup {
   uint32_t logging;
};
typedef struct optical_up_setup optical_up_setup_type;


////////////////////////////////////////////////////////////////////////
// returns offset to position in pyramid where desired image resides
uint32_t upright_get_offset_to_pyramid_level(
      /* in     */ const uint32_t level
      );

// use suppoert/static_consts.h
//float get_pixels_per_degree(
//      /* in     */ const uint32_t level
//      );

image_size_type upright_get_image_size(
      /* in     */ const uint32_t level
      );

#endif   // OPTICAL_UP_H

