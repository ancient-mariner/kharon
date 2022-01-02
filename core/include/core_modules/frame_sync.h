#if !defined(FRAME_SYNC_H)
#define FRAME_SYNC_H
#include "pin_types.h"
#include "pixel_types.h"
#include "logger.h"

#include "core_modules/optical_up.h"


// observes image data published by optical_up and groups frames 
//    together that ~correspond to the same point in time. 
//    published time is the time of the earliest frame in the
//    group
//
// takes 
//    optical_up output
//
// publishes list of cameras frames (upright) at a particular time
// list ordered by camera number (ie, list index = camera number)

// renumbering restructuring cams (need to adjust pan output and cam num tracking)

#define FRAME_SYNC_LOG_LEVEL     LOG_LEVEL_DEFAULT

struct frame_sync_output {
   // list stores frames from all input cameras, and NULL is stored
   //    if frame is not available from that camera for a given frame
   // consumer is expected to check each slot to see if data is available
   // NOTE: this points to a specific output entry in the optical_up
   //    queue. this isn't very safe or robust as there's nothing
   //    here to detect this slot in the output buffer being 
   //    overwritten. it's ok for now but thought should be given
   //    to strengthening this later
   optical_up_output_type  *frames[MAX_NUM_CAMERAS];
//   double t;
   // frame times are for logging and testing
//   double  frame_times[MAX_NUM_CAMERAS];
};
typedef struct frame_sync_output frame_sync_output_type;


// queue length needs to be duration that process can look back in
//    history to evaluate historical data -> now done at panorama
//    level
#define FRAME_SYNC_QUEUE_LEN      32

#define FRAME_SYNC_CLASS_NAME  "frame_sync"

////////////////////////////////////////////////////////////////////////
//

struct frame_sync_class {
   //FILE *logfile;
   log_info_type *log;
   uint8_t producer_to_cam_num[MAX_NUM_CAMERAS];
};
typedef struct frame_sync_class frame_sync_class_type;


// thread entry point
void * frame_sync_class_init(void *);


// struct to pass config data to thread
struct frame_sync_setup {
   uint32_t logging;
};
typedef struct frame_sync_setup frame_sync_setup_type;


#endif   // FRAME_SYNC_H
