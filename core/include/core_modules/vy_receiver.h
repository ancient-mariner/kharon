#if !defined(VY_RECEIVER_H)
#define VY_RECEIVER_H
#if !defined(_GNU_SOURCE)
#error "_GNU_SOURCE should be defined in makefile"
#endif   // _GNU_SOURCE
#include "pinet.h"
#include "logger.h"
#include "pixel_types.h"
#include <stdio.h>

// TODO extract config-loading code into independent function so
//    config data can be hot-reloaded

// receives VY image and distributes to subscribers
// data published as separate V and Y channels (that order)
// all images are of same size, and all are published as uint8_t[]

// mid/long term goal is for image producer to deliver images in
//    perspective format, not the common 'undistorted' output common
//    where equal distances in the real world are represented as
//    equal distances in the image (e.g., photo of a grid shows all
//    grid squares of equal size, not smaller with increasing eccentricity
//    which is how light arrives)
// historical note
// calculating perspective view on RPi takes >125ms for individual frame
//    at 2MP resolution, using trig algorithm (spherical approach from
//    optical up might be faster). however, perspective calculation is
//    presently deferred until time of spherical projection, to save 
//    information and avoid doing calculation twice

// the values CAM_ROWS, CAM_COLS, and CAM_N_PIX are defined in pin_types.h

#define  VY_ROWS  CAM_ROWS
#define  VY_COLS  CAM_COLS
#define  VY_N_PIX   (VY_ROWS * VY_COLS)


// output data has same format as 'uncorrected' image acquired by camera
//    (i.e., no perspective transform has been performed). V and Y channels
//    are of same resolution. border information is not stored as it's
//    determined from image directly (ie, where the borders are)
struct vy_receiver_output {
   double frame_request_time;
   image_size_type img_size[NUM_PYRAMID_LEVELS];
//   uint8_t v_chan[NUM_PYRAMID_LEVELS][VY_N_PIX];
//   uint8_t y_chan[NUM_PYRAMID_LEVELS][VY_N_PIX];
   // starting offset for data in chans for given level
   uint32_t chan_offset[NUM_PYRAMID_LEVELS];
   // pyramid level 0 is always at start of array, so can be read out
   //    w/o worrying about offets
   vy_pixel_type *chans;
};
typedef struct vy_receiver_output vy_receiver_output_type;

// don't need large queue as this is the preprocessed image. queue
//    should be long enough so that data isn't lost if system bogs
//    down
// 4 seconds should be enough. if system bogs down for longer than this,
//   older data will be stale (and there's also something seriously
//   wrong)
#define VY_QUEUE_LEN   24  


////////////////////////////////////////////////////////////////////////

#define VY_CLASS_NAME  "VY_receiver"

#define VY_LOG_LEVEL    LOG_LEVEL_DEFAULT

struct vy_class {
   int sockfd, connfd;
   char *camera_name;
   uint8_t camera_num;
   char *data_folder;
   // raw pixel data -- filled with incoming data. content is blurred during
   //    copying to output
   uint8_t *raw_v;
   uint8_t *raw_y;
   unsigned int *img_tmp;  // temporary buffer used for downsample blurring
   // remaps pixels to intermediate representation on unit sphere. once
   //    on sphere they are rotated to their correct position in world view
   const vector_type *sphere_map[NUM_PYRAMID_LEVELS];
   degree_type fov_horiz;
   degree_type fov_vert;
   image_size_type img_size[NUM_PYRAMID_LEVELS];
   //FILE *logfile;
   log_info_type *log;
   char device_name[MAX_NAME_LEN];
};
typedef struct vy_class vy_class_type;

// thread entry point
void * vy_class_init(void *);

// struct to pass config data to thread
struct vy_setup {
   uint32_t logging;
   uint8_t camera_num;
   char device_name[MAX_NAME_LEN];
};
typedef struct vy_setup vy_setup_type;


void module_config_load_cam_to_ship(
      /* in out */       log_info_type *log,
      /* in     */ const char *device_name,
      /* in     */ const char *module_name,
      /*    out */       matrix_type *dev2ship
      );


#endif   // VY_RECEIVER_H

