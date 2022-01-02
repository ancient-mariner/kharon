#include "pin_types.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <errno.h>
#include <assert.h>
#include "datap.h"
#include "pinet.h"
#include "kernel.h"
#include "logger.h"

#include "core_modules/frame_sync.h"
#include "core_modules/optical_up.h"



////////////////////////////////////////////////////////////////////////
// custom functions and structures

//#include "project_image.c"
struct frame_time {
   optical_up_output_type  *frame;
   double t;
}; 
typedef struct frame_time frame_time_type;


struct frame_set {
   optical_up_output_type  *frame[MAX_NUM_CAMERAS];
   double frame_time[MAX_NUM_CAMERAS];
   double set_time;
};
typedef struct frame_set frame_set_type;


// width of interval that frames should fall within. this should be
//    approximately the hardware frame interval -- expand that by
//    small %age to provide a comfortable margin
#define FRAME_WINDOW    (1.330 / (double) HARDWARE_FRAME_RATE) 

static int32_t get_next_earliest_frame(
      /* in out */       datap_desc_type *self,
      /*    out */       frame_time_type *frame
      )
{
   // cycle through all producers and return time of earliest available
   //    frame
   // if none available, return -1
   int32_t early_idx = -1;
   frame->t = 1.0e30;
   const uint32_t n_producers = self->num_attached_producers;
   for (uint32_t i=0; i<n_producers; i++) {
      const producer_record_type *pr = &self->producer_list[i];
      const datap_desc_type *producer = pr->producer;
      if (pr->consumed_elements < producer->elements_produced) {
         // data available from this producer -- evaluate when
         const uint32_t idx = (uint32_t) 
               (pr->consumed_elements % producer->queue_length);
         const double t = producer->ts[idx];
         if (t < frame->t) {
            frame->frame = 
                  (optical_up_output_type *) dp_get_object_at(producer, idx); 
            frame->t = t;
            early_idx = (int32_t) i;
         }
      }
   }
   // pop this frame from the producer so we don't process it again
   if (early_idx >= 0) {
      self->producer_list[early_idx].consumed_elements++;
   }
   return early_idx;
} 


void log_frame_set(
      /* in     */ const frame_sync_class_type *sync,
      /* in     */ const frame_set_type *set
      );
void log_frame_set(
      /* in     */ const frame_sync_class_type *sync,
      /* in     */ const frame_set_type *set
      )
{
   char buf[STR_LEN];
   uint32_t chars_left = STR_LEN;
   int32_t cnt = snprintf(buf, chars_left, "(%.3f) ", set->set_time);
   if (cnt < 0)
      return;  // something wrong with the stream (unlikely, but possible)
   for (uint32_t i=0; i< MAX_NUM_CAMERAS; i++) {
      double t = -1;
      if (set->frame[i] != NULL)
         t = set->frame_time[i];
      chars_left = chars_left - (uint32_t) cnt;
      cnt = snprintf(&buf[STR_LEN - chars_left], chars_left, "%.3f ", t);
      if (cnt < 0)
         break;
   }
   buf[STR_LEN-1] = 0;
   log_debug(sync->log, buf);
}

static void add_frame_to_set(
      /* in out */       frame_set_type *set,
      /* in     */ const frame_time_type *next,
      /* in     */ const uint8_t cam_num
      )
{
   set->frame[cam_num] = next->frame;
   const double t = next->t;
   set->frame_time[cam_num] = t;
   if (t < set->set_time)
      set->set_time = t;
} 


static void clear_frame_set(
      /*    out */       frame_set_type *set
      )
{
   for (uint32_t i=0; i<MAX_NUM_CAMERAS; i++) {
      set->frame[i] = NULL;
      set->frame_time[i] = 0.0;
      set->set_time = -1.0;
   }
}


static void copy_frame_set_to_output(
      /* in out */       datap_desc_type *self,
      /* in     */ const frame_set_type *set
      )
{
   uint32_t idx = (uint32_t) (self->elements_produced % self->queue_length);
   self->ts[idx] = set->set_time;
   frame_sync_output_type *output = (frame_sync_output_type*)
         dp_get_object_at(self, idx);
   for (uint32_t i=0; i<MAX_NUM_CAMERAS; i++) {
      output->frames[i] = set->frame[i];
   }
}

static void init_frame_set(
      /*    out */       frame_set_type *set,
      /*    out */       frame_time_type *next,
      /* in     */ const uint32_t idx
      )
{
   clear_frame_set(set);
   set->frame[idx] = next->frame;
   const double t = next->t;
   set->frame_time[idx] = t;
   set->set_time = t;
}


////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

static void frame_sync_add_producer(
      /* in     */       struct datap_desc *self, 
      /* in     */       struct datap_desc *prod
      )
{
   frame_sync_class_type *sync = (frame_sync_class_type *) self->local;
   // make sure this is a supported producer type
   if (strcmp(prod->td->class_name, OPTICAL_UP_CLASS_NAME) == 0) {
      const uint16_t n = self->num_attached_producers++;
      if (n >= (MAX_NUM_CAMERAS - 1)) {
         // if more cameras are needed, it should be OK to adjust the
         //    max-camera constant and recompile (that might not be
         //    the case in downstream image stitching modules)
         log_err(sync->log, "Too many subscriptions (max of %d)\n", 
               MAX_NUM_CAMERAS);
         log_err(sync->log, "Consumer: %s\n", self->td->obj_name);
         log_err(sync->log, "Producer: %s (%s)\n", prod->td->obj_name, 
               prod->td->class_name);
         hard_exit(__func__, __LINE__);
      }
      self->producer_list[n].producer = prod;
      optical_up_class_type *up = (optical_up_class_type*) prod->local;
      uint8_t cam_num = up->camera_num;
      if (cam_num >= MAX_NUM_CAMERAS) {
         log_err(sync->log, "Producer's camera number outside of bounds (%d)", 
               cam_num);
         log_err(sync->log, "Consumer: %s\n", self->td->obj_name);
         log_err(sync->log, "Producer: %s (%s)\n", prod->td->obj_name, 
               prod->td->class_name);
         hard_exit(__func__, __LINE__);
      }
      // map between producer number and camera num
      sync->producer_to_cam_num[n] = cam_num;
   } else {
      log_err(sync->log, "Attempted to subscribe to incompatible producer\n");
      log_err(sync->log, "Consumer: %s\n", self->td->obj_name);
      log_err(sync->log, "Producer: %s (%s)\n", prod->td->obj_name, 
            prod->td->class_name);
      hard_exit(__func__, __LINE__);
   } 
}


////////////////////////////////////////////////////////////////////////
static void frame_sync_class_pre_run(struct datap_desc *self)
{
   // allocate storage
   // set element size and queue length
   self->element_size = sizeof(frame_sync_output_type);
   self->queue_length = FRAME_SYNC_QUEUE_LEN;
   self->ts = calloc(1, FRAME_SYNC_QUEUE_LEN * sizeof(*self->ts));
   self->void_queue = calloc(1, FRAME_SYNC_QUEUE_LEN * self->element_size);
}


////////////////////////////////////////////////////////////////////////


static void frame_sync_class_run(
      /* in out */       datap_desc_type *self
      )
{
   frame_sync_class_type *sync = (frame_sync_class_type *) self->local;
   log_info(sync->log, "Entering run()");
   // publish sets of frames that were taken at approximately the same time
   // keep track of 2 sets of frames and populate these sets as frames
   //    come in. if a frame comes in beyond the 2nd set, consider that
   //    the 1st set is done and publish it. additional frames for it,
   //    if they arrive later, are dropped
   frame_set_type   a;
   frame_set_type   b;
   clear_frame_set(&a);
   clear_frame_set(&b);
   // use pointers to the buffers so they're easier to swap
   frame_set_type *set0 = &a;
   frame_set_type *set1 = &b;
   //
   frame_time_type   next = { .frame=NULL, .t=0.0   };
   // 
   while ((self->run_state & DP_STATE_DONE) == 0) {
//log_info(sync->log, "Entering wait()");
      dp_wait(self);   // wait for data to become available
      // maintain two frame sets. as soon as we get a frame that belongs
      //    beyond the second set, close out the first set, shift the
      //    2nd to the first, and let the frame go the fresh 2nd set.
      // keep reading frames until no more are available
      while (1) {
         int32_t cam_idx = get_next_earliest_frame(self, &next);
         if (cam_idx < 0) {
            log_info(sync->log, "No more frames available");
            break;   // no more available frames
         }
//log_info(sync->log, "Processing next frame (t=%.3f)", next.t);
         uint8_t cam_num = sync->producer_to_cam_num[cam_idx];
         // see if frame is in set 0, 1 or beyond
         if (next.t < set0->set_time + FRAME_WINDOW) {
            // time is before set 1. see if in set 0
            if (next.t > set0->set_time - FRAME_WINDOW) {
//log_info(sync->log, "Add frame at t=%.3f to set 0", next.t);
               add_frame_to_set(set0, &next, cam_num);
            }  // else, frame belongs to earlier set. drop it
         } else if (next.t <= set1->set_time + FRAME_WINDOW) {
            // frame is in set 1
//log_info(sync->log, "Add frame at t=%.3f to set 1", next.t);
            add_frame_to_set(set1, &next, cam_num);
         } else {
            // the next frame is in the following set. close out 
            //    set0, publish it, and shift the buffers down
            // on initialization this branch is followed. check for t>0
            //    and publish if so (during init, t=0)
            if (set0->set_time > 0.0) {
//log_info(sync->log, "Publish set 0");
               // publish set 0
               // copy set0 to output buffer
               copy_frame_set_to_output(self, set0);
//log_frame_set(sync, set0);
               // report that data is available
               self->elements_produced++;
//log_info(sync->log, "Publishing element %ld", self->elements_produced);
               dp_signal_data_available(self);
            }
            // move 1 to 0. init the new 1
//log_info(sync->log, "Retiring set 0");
            frame_set_type *tmp = set0;
            set0 = set1;
            set1 = tmp;
            init_frame_set(set1, &next, (uint32_t) cam_idx);
         }
      }
   }
   log_info(sync->log, "Leaving run()");
}

////////////////////////////////////////////////////////////////////////
static void frame_sync_class_post_run(
      /* in out */       datap_desc_type *self
      )
{
   (void) self;
//   printf("%s in post_run\n", self->td->obj_name);
//   frame_sync_class_type *sync = (frame_sync_class_type *) self->local;
//   if (sync->logfile) {
//      fclose(sync->logfile);
//      sync->logfile = NULL;
//   }
}


////////////////////////////////////////////////////////////////////////
static void * frame_sync_get_object_at(
      /* in     */ const datap_desc_type *self,
      /* in     */ const uint32_t idx
      )
{  
   return &self->void_queue[idx * sizeof(frame_sync_output_type)];
} 


////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////
void * frame_sync_class_init(
      /* in     */       void *module_setup
      )
{
   datap_desc_type *self = dp_create();
   frame_sync_class_type *sync = calloc(1, sizeof(*sync));
   if (c_assert(module_setup != NULL)) {
      log_err(sync->log, "Internal error -- init passed NULL value");
      hard_exit(__func__, __LINE__);
   }
//   frame_sync_setup_type *setup = (frame_sync_setup_type*) module_setup;
   sync->log = get_logger(self->td->obj_name);
   report_thread_id(self, sync->log);
   // 
//   if (setup->logging != 0) {
//      // open logfile
//      char buf[STR_LEN];
//      // construct path
//      snprintf(buf, STR_LEN, "%s%s", get_log_folder_name(), 
//            self->td->obj_name);
//      sync->logfile = fopen(buf, "w");
//      if (sync->logfile == NULL) {
//         // non-fatal error. we just loose logging
//         log_err("Unable to create logfile for %s (%s)", 
//               self->td->obj_name, buf);
//      } else {
//         log_info("%s logging data to %s", self->td->obj_name, buf);
//         log_info("format: <frame time> <cam 0 time> <cam 1 time> ...");
//      }
//   } else {
//      sync->logfile = NULL;
//   }
   free(module_setup); // allocated on heap and it's no longer needed
   //
   self->local = sync;
   self->pre_run = frame_sync_class_pre_run;
   self->post_run = frame_sync_class_post_run;
   self->run = frame_sync_class_run;
   self->add_producer = frame_sync_add_producer;
   self->get_object_at = frame_sync_get_object_at;
   /////////////////
   // once initialization done, put into runtime mode
   dp_execute(self);
   return NULL;
}

