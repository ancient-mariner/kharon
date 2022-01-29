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

// wrapper for when a camera frame was captured. this isn't in the header
//    as it's a local structure that doesn't require usage elsewhere
struct frame_time {
   optical_up_output_type  *frame;
   double t;
};
typedef struct frame_time frame_time_type;

#include "align_frames.c"


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
      // TODO make sure the same cam hasn't subscribed twice. that could
      //    really screw some things up
      sync->num_input_cams++;
      uint32_t n = self->num_attached_producers++;
      if (sync->num_input_cams >= (MAX_NUM_CAMERAS - 1)) {
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
   //
   frame_sync_class_type *sync = (frame_sync_class_type *) self->local;
   sync->frame_node_heap =
         calloc(FRAME_NODE_HEAP_SIZE, sizeof *sync->frame_node_heap);
   frame_node_type* prev = NULL;
   for (uint32_t i=0; i<FRAME_NODE_HEAP_SIZE; i++) {
      frame_node_type *node = &sync->frame_node_heap[i];
      node->node_num = i;
      node->prev = prev;
      if (i > 0) {
         prev->next = node;
      } // else there's no prev to set
      prev = node;
   }
   // head of linked list is first node in heap
   sync->frame_node_list_head = sync->frame_node_heap;
}


////////////////////////////////////////////////////////////////////////


static void frame_sync_class_run(
      /* in out */       datap_desc_type *self
      )
{
   frame_sync_class_type *sync = (frame_sync_class_type *) self->local;
   log_info(sync->log, "Entering run()");
   // publish sets of frames that were taken at approximately the same time
   //
   while ((self->run_state & DP_STATE_DONE) == 0) {
      dp_wait(self);   // wait for data to become available
      // loop while there's data to read
      while (1) {
         frame_time_type next;
         int32_t cam_idx = get_next_earliest_frame(self, &next);
         if (cam_idx < 0) {
            //log_info(sync->log, "No more frames available");
            break;
         }
         uint8_t cam_num = sync->producer_to_cam_num[cam_idx];
         // submit frame for analysis and see if there's anything to be
         //    published. the logic for this would be cleaner if the
         //    publish loop was in the called function. to better integrate
         //    with tests, the loop is moved outside so it can be
         //    determined whether or not a set is published after each
         //    given frame
         add_frame_to_list(sync, &next, cam_num);
         double publish_time;
         do {
            publish_time = check_for_frame_set(sync, next.t);
            if (publish_time > 0.0) {
               // get storage for output
               uint32_t idx = (uint32_t)
                     (self->elements_produced % self->queue_length);
               frame_sync_output_type *out = (frame_sync_output_type*)
                     dp_get_object_at(self, idx);
               //
               uint32_t num_frames = build_frame_set(sync, out, publish_time);
               purge_old_frames(sync, publish_time);
               log_info(sync->log, "Set of %d frames at %.3f", num_frames,
                     publish_time);
               //
               self->ts[idx] = publish_time;
               self->elements_produced++;
               sync->last_sync_time = publish_time;
               dp_signal_data_available(self);
            }
         } while (publish_time > 0.0);
      }
   }
}

////////////////////////////////////////////////////////////////////////
static void frame_sync_class_post_run(
      /* in out */       datap_desc_type *self
      )
{
   (void) self;
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
   sync->log = get_logger(self->td->obj_name);
   report_thread_id(self, sync->log);
   //
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

