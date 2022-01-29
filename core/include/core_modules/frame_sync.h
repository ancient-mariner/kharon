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

// ???? renumbering restructuring cams (need to adjust pan output 
//    and cam num tracking)

// linked list of recent frames 
// when unallocated, this is stored in the list 
//    frame_sync->frame_node_list_head
// when allocated, this is part of a sorted list (increasing by t) 
//    stored in the list frame_sync->active_frame_list_head
struct frame_node {
   optical_up_output_type *frame;
   double t;
   struct frame_node* next;
   struct frame_node* prev;
   uint8_t cam_num;
   uint32_t node_num;   // for debugging
};
typedef struct frame_node frame_node_type;

// number of frame nodes in use should max out at #cams * FPS * 2
// allocate twice this amount
#define FRAME_NODE_HEAP_SIZE  (4 * MAX_NUM_CAMERAS * CAMERA_FPS)

// if time since last frame set was generated is more than a certain
//    duration then assume a input frame was missed. this is that duration.
//    when a frame arrives after this interval then a partial frame set
//    is looked for around when the frame set should have been
#define MISSED_FRAME_INTERVAL_SEC   (1.5 * CAMERA_FRAME_INTERVAL_SEC)

// threshold or too much time having elapsed since previous frame for
//    all of pending frame data to be flushed as old frames are too old
//    and we need to focus on new ones
#define STREAM_DUMP_INTERVAL_SEC   1.5

// interval on either side of sync frame time that contributing 
//    image captures can be and be part of that frame
// this is 1msec longer than hardware capture interval
#define FRAME_ALIGN_SECS         (0.001 + HARDWARE_INTERFRAME_INTERVAL_SEC)

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
};
typedef struct frame_sync_output frame_sync_output_type;


// queue length needs to be duration that process can look back in
//    history to evaluate historical data -> now done at panorama
//    level so queue length can shrink (is 8 sufficient? -- maybe so
//    but sync requires minimal memory so use use comfortable number)
#define FRAME_SYNC_QUEUE_LEN      16

#define FRAME_SYNC_CLASS_NAME  "frame_sync"

////////////////////////////////////////////////////////////////////////
//

struct frame_sync_class {
   //FILE *logfile;
   log_info_type *log;
   uint8_t producer_to_cam_num[MAX_NUM_CAMERAS];
   //
   uint32_t num_input_cams;
   //
   frame_node_type* active_frame_list_head;
   double last_sync_time;
   //
   frame_node_type* frame_node_heap;
   frame_node_type* frame_node_list_head;
   uint32_t num_allocated_nodes;
};
typedef struct frame_sync_class frame_sync_class_type;
typedef struct frame_sync_class frame_sync_type;   // new style


// thread entry point
void * frame_sync_class_init(void *);


// struct to pass config data to thread
struct frame_sync_setup {
   uint32_t logging;
};
typedef struct frame_sync_setup frame_sync_setup_type;


#endif   // FRAME_SYNC_H
