#if !defined(PANORAMA_H)
#define PANORAMA_H
#include "pin_types.h"
#include <stdio.h>
#include "datap.h"
#include "logger.h"
#include "image.h"
#include "pixel_types.h"
#include "core_modules/support/frame_heap.h"

#if INSERT_PHANTOM_IMAGE == 1
#include "core_modules/support/phantom.h"
#endif   // INSERT_PHANTOM_IMAGE == 1

#define PANORAMA_LOG_LEVEL    LOG_LEVEL_DEFAULT

// stitches Y channel from available cameras to generate panoramic
//    image
//
// takes 
//    optical_up output
//
// publishes pixel mapping array for each pyramid level. map stores
//    foreground and background pixels (bg where appropriate)
// north is column 0


// TODO consider optical up publishing accumulators (inset rad is level)
//
// memory consumption of panorama is too much to cache values
//    in the style of standard modules. a single frame of relatively
//    low-res panorama (3600x720) is ~46MB. Storing 128 frames (~25s@5Hz)
//    requires 6GB of RAM
// automatic compaction allows much longer time scale to be stored in
//    less space, by increasingly old frames having longer inter-frame
//    intervals. a list with allocated storage for 36 frames (48 in total,
//    with buffering, for 2.2GB) can store 6-8 minutes of image data
//

////////////////////////////////////////////////////////////////////////
// panorama coverage

#define NUM_PAN_COVERAGE_RADIALS    360
// NOTE if this constant is changed then things will break. code assumes
//    one radial per degree
// TODO migrate to using 256

// indicates what radials have image data. note that this is approximate
//    and accuracy is dependent on vessel attitude. first implementation
//    is that coverage extends +/-29 degrees from center of projected
//    optical-up image (of 62 degree image)
// stores 
//    0 if there's no image output on a radial
//    1 if image output is present on given radial
struct panorama_coverage {
   uint8_t radial[NUM_PAN_COVERAGE_RADIALS];
};
typedef struct panorama_coverage panorama_coverage_type;

// panorama coverage
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
// color grid
// OBSOLETE -- heavy hit on performance for little gain
#if 0

//// number of color grids in world view (ie, over 360 degrees)
//// note: this value is for reference only. it is hard-coded and things
////    will break if it changes. 256 gives ~1.4 degrees per grid
//// ie, a bam8 is used for representing 360 degrees
//#define NUM_COLOR_GRIDS_HORIZ     256

// width_deg must evenly devide 360 -- good values are 1,2,2.5,3,4,5,6
#define COLOR_GRID_UNIT_WIDTH_DEG   2.0f
#define COLOR_GRID_UNIT_HEIGHT_DEG  1.0f

#define NUM_COLOR_GRIDS_HORIZ ((uint32_t) (360.0f / COLOR_GRID_UNIT_WIDTH_DEG))

// decay time constant of colors
#define COLOR_GRID_TIME_CONSTANT_SECS     15.0
#define COLOR_GRID_TAU        \
      (1.0 / ((double) CAMERA_FPS * COLOR_GRID_TIME_CONSTANT_SECS))
// TODO consider in future having multiple time constants and multiple
//    distributions, to more easily detect changes

// number of bins in color distribution
// this must be a power of 2
#define COLOR_GRID_DIST_SIZE     64

//// describes the categorization of a small portion of the field of view.
////    square contains summary (over time) of what is observed in that 
////    portion of field of view
//struct pan_color_grid_unit {
//   // distribution of colors observed recently
//   // colors are recorded at each level
//   doulbe color_y[NUM_PYRAMID_LEVELS][COLOR_GRID_DIST_SIZE];
//// TODO FIXME v range is too narrow to have a meaningful distribution,
////    as NIR causes most values to be near 128. until this is fixed
////    at the camera end the color_v distribution shouldn't be used.
////    when it is, update keypoint/pixel_features.c:calc_pixel_features() 
////    to include it when generating color score
////    (fix: eg, downsampling full-res image and getting value for v that
////    doesn't induce discretization artifacts by magnifying small
////    integral offset from 128)
//   float color_v[NUM_PYRAMID_LEVELS][COLOR_GRID_DIST_SIZE];
//   // total area under each distribution (this will be the same for 
//   //    all color channels)
//   float num_samples[NUM_PYRAMID_LEVELS];
//};
//typedef struct pan_color_grid_unit pan_color_grid_unit_type;


// holds an array of color grid units
// grids are vertically aligned to a point just under the horizon. this is
//    to avoid having grids covering water areas in an image be mixed with
//    what's visible above the horizon, as that will greatly weaken
//    value of having a pixel-value distribution (what's above the horizon
//    will typically have a different distribution profile versus what's
//    below the horizon)
// ideally the split would be at the horizon, but realistically the cameras
//    may be a few tenths of a degree off, which may result in >horizon
//    pixels being mixed in with water pixels. also, the primary reason
//    for maintaining a distribution is to reduce correlated motion
//    in waves from forming a target, as water pixels should have common 
//    pixel values, while an actual target is more likely to have colors
//    that are distinct from the water. if a ghost target appears on the
//    horizon, that won't trigger a collision-avoidance panic as it will
//    have a large distance. also, there's limited correlated wave motion
//    observed near the horizon due the distance, so pixel distributions
//    near the horizon aren't so useful for noise elimination
// -> grids are based on bam8 size, so each grid is ~1.4 degrees tall/wide.
//    have center grid row's bottom be 1/4 height (.35 deg) below the
//    horizon. this will provide a comfortable distance from the horizon
//    so even w/ significant misalignment the horizon shouldn't mix with
//    water grid units, and that ~0.35 degrees should have largely
//    uninteresting for the sake of noise reduction. however, it may
//    be useful for detecting the presence of objects due to changes
//    in color profile
#define GRID_BELOW_HORIZON_OFFSET     0.35f
struct pan_color_grid {
   // top of highest grid row does not necessarily align with
   //    top of panorama. it's shifted up slightly so that grid
   //    row that includes the horizon extends a fixed distance
   //    (GRID_BELOW_HORIZON_OFFSET) below the horizon
   // top buffer is how many pixels tall this buffer is
   uint32_t top_buffer[NUM_PYRAMID_LEVELS];
//   uint32_t num_grids_horiz;
//   uint32_t num_grids_vert;
   image_size_type size;
   //
   pan_color_grid_unit_type *grid;
};
typedef struct pan_color_grid pan_color_grid_type;

#endif   // 0
// color grid
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
// panorama output

// WARNING -- panorama publishes data in a non-standard way
// a pointer to each panorama output slice is stored in a frame page.
// frame page contents should be fetched as a full list, which has
//    been compacted w/ increasing time since the frame was captured.
// THE DATA STORED IN VOID_QUEUE IS OUT OF ORDER. IT SHOULD NOT BE READ
//    DIRECTLY
// panorama data should be accessed through get_frame_list(), which
//    returns a linked list of all cached frames. the list is 
//    read-only and is thread-safe. all frames in the list are guaranteed
//    to be valid for 2 seconds.
// expected use case is that data is pulled asynchronously and that
//    no subscribers care when new data is available (because each
//    subscriber only cares about a small part of the visual scene
//    at a time, and won't immediaately examine an area, unless it
//    really wants to)
// 
// 
struct panorama_output {
   // pointers are offsets in image buffer
   // overlap pixels identify fg and bg cameras for a given point in
   //    the world image
   overlap_pixel_type *world_frame[NUM_PYRAMID_LEVELS];
   // when storing output to disk in raw form, world_frame pointers are
   //    useless. store the index as well so the pointers can be
   //    regenerated when loaded
   uint32_t world_frame_idx[NUM_PYRAMID_LEVELS];
   // size of each level (this doesn't change -- could be made a static const)
   // TODO migrate to using static const
   image_size_type frame_size[NUM_PYRAMID_LEVELS];
   // pyramid_ is allocation that the pyramids in the world_frame array 
   //    points into (ie, it's a heap that stores memory for all pyramid
   //    levels, and the world_frame array points to the appropriate
   //    location in the heap for a particular level)
   // it should not be accessed directly
   overlap_pixel_type *pyramid_;
   // recent history of colors seen at each region of world view
   // this is a pointer into panorama's color grid heap
   // color distribution that is published is actually a sum of 
   //    the grid units surrounding a point in space, to reduce influence
   //    of object in field of view biasing distribution (distribution
   //    is to approximate background)
   struct pan_color_grid *color_grid;
   // TODO have gaze publish radial data
   // area of visual field that panorama has image data
   panorama_coverage_type coverage;
   ////////////////////
   // approx attitude data at time of image
   degree_type heading;
   degree_type roll;
   degree_type pitch;
};
typedef struct panorama_output panorama_output_type;

// panorama log output
// logs are a single binary file, with blocks that contain the timestamp
//    and a panorama_output struct. specifically:
//    # frame 0
//       char[32]
//       panorama_output
//    # frame 1
//       char[32]
//       panorama_output
//    # frame...
#define PAN_LOG_OUTPUT_TIME_SIZE     32u
#define PAN_LOG_OUTPUT_BLOCK_SIZE    \
      ((uint32_t) ((PAN_LOG_OUTPUT_TIME_SIZE + sizeof(panorama_output_type))))


// NOTE: see above regarding how 'queue' is implemented before changing value
// (panorama does not use queue like a standard process)
#define PANORAMA_QUEUE_LEN      64

#define PANORAMA_CLASS_NAME  "panorama"

////////////////////////////////////////////////////////////////////////
//


struct panorama_class {
   // for writing image files
   char *data_folder;
   //
   struct frame_page_heap *frame_heap;
   // need to be able go backward and get datap from panorama class
   datap_desc_type *self;
   // flag indicating if panorama in replay mode
   uint32_t replay;
   // output type if logging
   // 0 is grayscale output
   // 1 is color. red images w/ overlap regions having G & B elements
   uint32_t output_type;
   // 
   log_info_type *log;
   // camera height above water when ship is level, in meters
   meter_type camera_height;
   // camera position forward of rotational axis, in meters
   meter_type camera_forward_position;
   // camera position starboard of centerline, in meters
   meter_type camera_starboard_position;
//   /////////////////////////////////////////////
//   // color grid -- part of layout
//   pan_color_grid_type *color_grid_heap;
//   uint32_t color_grid_queue_idx;
//   // master grid is copied onto heap with each frame page update
//   pan_color_grid_type master_color_grid;
#if INSERT_PHANTOM_IMAGE == 1
   /////////////////////////////////////////////////////////////////////
   // phantom
   uint32_t num_phantoms;
   phantom_image_type *phantoms[MAX_PHANTOMS];
   // phantom timers are based on stream start so they're not fully
   //    set until data is actually streaming. this flag is set to
   //    non-zero once timers are set
   uint32_t phantom_init_flag;
#endif // INSERT_PHANTOM_IMAGE == 1
};
typedef struct panorama_class panorama_class_type;
typedef struct panorama_class panorama_type;


// thread entry point
void * panorama_init(void *);


// struct to pass config data to thread
struct panorama_setup {
   uint32_t logging;
   // 0 or 1: 0 for normal output, 1 for colored, for aligning cams
   uint32_t output_type;
   meter_type camera_height_meters;
   meter_type camera_position_forward;
   meter_type camera_position_starboard;
};
typedef struct panorama_setup panorama_setup_type;

////////////////////////////////////////////////////////////////////////

const frame_page_type * panorama_get_frame_list(
      /* in     */ const panorama_class_type *pan
      );

// see keypoint's pixel_features for code of (v348)
//uint32_t get_color_grid_unit_idx(
//      /* in     */ const image_coordinate_type pos,
//      /* in     */ const pan_color_grid_type *grid,
//      /* in     */ const uint32_t level
//      );

//void get_camera_position(
//      /* in     */ const panorama_class_type *pan,
//      /*    out */       meter_type *height,
//      /*    out */       meter_type *forward_position,
//      /*    out */       meter_type *starboard_position
//      );

void insert_test_image(
      /* in out */       datap_desc_type *imu_dp,
      /* in     */ const char * config_name
      );

#endif   // PANORAMA_H
