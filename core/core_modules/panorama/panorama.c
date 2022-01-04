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
#include "pinet.h"
#include "datap.h"
#include "kernel.h"
#include "lin_alg.h"

#include "core_modules/panorama.h"
#include "core_modules/frame_sync.h"
#include "core_modules/optical_up.h"

////////////////////////////////////////////////////////////////////////
// static vars and inlined code

#if INSERT_PHANTOM_IMAGE == 1
static const char phantom_config_file_[STR_LEN] = { 0 };
#endif // INSERT_PHANTOM_IMAGE


#include "support.c"
#include "frame_heap.c"
#if INSERT_PHANTOM_IMAGE == 1
#include "insert_phantom.c"
#endif // INSERT_PHANTOM_IMAGE
//#include "layout.c"

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

static void panorama_add_producer(
      /* in     */       struct datap_desc *self,
      /* in     */       struct datap_desc *prod
      )
{
   panorama_class_type *pan = (panorama_class_type *) self->local;
   // make sure this is a supported producer type
   if (strcmp(prod->td->class_name, FRAME_SYNC_CLASS_NAME) == 0) {
      ;
//   } else if (strcmp(prod->td->class_name, REPLAY_PAN_CLASS_NAME) == 0) {
//      panorama_class_type *panorama = (panorama_class_type *) self->local;
//      panorama->replay = 1;
   } else {
      log_err(pan->log, "Attempted to subscribe to incompatible producer\n");
      log_err(pan->log, "Consumer: %s\n", self->td->obj_name);
      log_err(pan->log, "Producer: %s (%s)\n", prod->td->obj_name,
            prod->td->class_name);
      hard_exit(__func__, __LINE__);
   }
   self->producer_list[self->num_attached_producers].producer = prod;
   self->producer_list[self->num_attached_producers].consumed_elements = 0;
   self->num_attached_producers++;
   if (self->num_attached_producers > 1) {
      log_err(pan->log, "Panorama can have only one producer "
            "(either frame-sync or replay-pan)\n");
      log_err(pan->log, "Consumer: %s\n", self->td->obj_name);
      log_err(pan->log, "Producer: %s (%s)\n", prod->td->obj_name,
            prod->td->class_name);
      hard_exit(__func__, __LINE__);
   }
}


////////////////////////////////////////////////////////////////////////
static void panorama_pre_run(struct datap_desc *self)
{
   const uint32_t pyramid_offset[NUM_PYRAMID_LEVELS] = {
      0,
      WORLD_WIDTH_PIX[0] * WORLD_HEIGHT_PIX[0]
//      WORLD_C0 * WORLD_R0 + WORLD_C1 * WORLD_R1
   };
#if INSERT_PHANTOM_IMAGE != 0
#warning "Phantom images are enabled"
#endif   // INSERT_PHANTOM_IMAGE
   panorama_class_type *pan = (panorama_class_type *) self->local;
   // sanity checks
   // make sure there's exactly one producer
   if (self->num_attached_producers != 1) {
      log_err(pan->log, "Panorama needs one frame-sync producer. "
            "It has zero");
      hard_exit(__func__, __LINE__);
   }
   // allocate storage
   // set element size and queue length
   self->element_size = sizeof(panorama_output_type);
   self->queue_length = PANORAMA_QUEUE_LEN;
   self->ts = calloc(1, PANORAMA_QUEUE_LEN * sizeof(*self->ts));
   self->void_queue = calloc(PANORAMA_QUEUE_LEN, self->element_size);
   //
   pan->frame_heap = create_heap();
   // break into heap and give each frame page a pointer into the
   //    void queue
   frame_page_type *node = pan->frame_heap->available_head;
   for (uint32_t i=0; i<MAX_FRAME_HEAP_ALLOC; i++) {
      node->frame = (panorama_output_type*) dp_get_object_at(self, i);
      node = node->_next;
   }
//   /////////////////////////////////////////////
//   // color grid
//   init_color_grid(&pan->master_color_grid);
//   pan->color_grid_heap =
//         malloc(PANORAMA_QUEUE_LEN * sizeof *pan->color_grid_heap);
//   for (uint32_t i=0; i<PANORAMA_QUEUE_LEN; i++) {
//      init_color_grid(&pan->color_grid_heap[i]);
//   }
   /////////////////////////////////////////////
   // output buffer
   // buffer is 1.5x the size of world at lowest pyramid level
   //    as all pyrs can be stored there
   // format:
   //    11111111...112222...2333...34...4
   uint32_t n_pix = (uint32_t)
         (3 * WORLD_HEIGHT_PIX[0] * WORLD_WIDTH_PIX[0] / 2);
   for (uint32_t i=0; i<PANORAMA_QUEUE_LEN; i++) {
      panorama_output_type *out =
            &((panorama_output_type*) self->void_queue)[i];
      out->pyramid_ = malloc(n_pix * sizeof *out->pyramid_);
//      out->color_grid = &pan->color_grid_heap[i];
      for (uint32_t lev=0; lev<NUM_PYRAMID_LEVELS; lev++) {
         out->world_frame[lev] = &out->pyramid_[pyramid_offset[lev]];
         out->world_frame_idx[lev] = pyramid_offset[lev];
      }
   }
   return;
}


////////////////////////////////////////////////////////////////////////

static void panorama_run(
      /* in out */       datap_desc_type *self
      )
{
   // producer can be either frame-sync or replay-pan. both are
   //    standard producers so use a generic representation
   producer_record_type *prod_rec = &self->producer_list[0];
   datap_desc_type *prod = prod_rec->producer;
   panorama_class_type *pan = (panorama_class_type *) self->local;
//   uint32_t n_grid_units = (uint32_t) (pan->master_color_grid.size.y *
//         pan->master_color_grid.size.x);
   // this is redundant w/ elements produced and is used for auto-compaction
   //    of frame list. keep it distinct for to clearly separate logic
   uint32_t compaction_count = 0;
//pan_color_grid_type *out_grid = NULL;
   //
   while ((self->run_state & DP_STATE_DONE) == 0) {
//      log_info(pan->log, "Waiting for data");
      dp_wait(self);   // wait for data to become available
      while (prod_rec->consumed_elements < prod->elements_produced) {
//         log_info(pan->log, "Consuming %ld of %ld",
//               prod_rec->consumed_elements, prod->elements_produced);
         ///////////////////////////////////////////////////////////////
         // get data source
         const uint32_t p_idx = (uint32_t)
               prod_rec->consumed_elements % prod->queue_length;
         const double t = prod->ts[p_idx];
         ///////////////////////////////////////////////////////////////
         // get data sink
         const uint32_t idx = (uint32_t)
               (self->elements_produced % self->queue_length);
         frame_page_type *page= allocate_page(pan->frame_heap);
         self->ts[idx] = t;
         page->t = t;
         ///////////////////////////////////////////////////////////////
         // write frames to panorama
         // clear storage buffers first
         clear_world_buffer(page->frame);
         // get data source
         frame_sync_output_type *sync_input = (frame_sync_output_type*)
               dp_get_object_at(prod, p_idx);
         // pull data from frame sync
         uint32_t active_frames = 0;
         // clear coverage array
         memset(&page->frame->coverage, 0, sizeof page->frame->coverage);
         for (uint32_t lev=0; lev<NUM_PYRAMID_LEVELS; lev++) {
            const image_size_type out_sz = {
                  .width = (uint16_t) WORLD_WIDTH_PIX[lev],
                  .height = (uint16_t) WORLD_HEIGHT_PIX[lev]
            };
            page->frame->frame_size[lev] = out_sz;
            // push each active frame (camera) to world view
            for (int8_t cam_num=0; cam_num<MAX_NUM_CAMERAS; cam_num++) {
               optical_up_output_type *frame = sync_input->frames[cam_num];
               // frames can be lost or simply not there. project frame
               //    if it's available
               if (frame != NULL) {
                  active_frames++;
                  project_frame_to_panorama(frame, frame->size[lev],
                        page->frame, out_sz, lev);
               }
            }
         }
         ///////////////////////////////////////////////////////////////
         // when processing complete
         ///////////////////////////////////////
         // insert phantoms, if those are in use
#if INSERT_PHANTOM_IMAGE == 1
         project_phantom_images(pan, page->frame, t);
#endif // INSERT_PHANTOM_IMAGE
//         ///////////////////////////////////////
//         // update color grid
//         push_to_color_grid(&pan->master_color_grid, page);
//         // copy master grid to output, and assign page pointer to that output
//         uint32_t grid_idx = pan->color_grid_queue_idx++;
//         if (grid_idx >= PANORAMA_QUEUE_LEN) {
//            grid_idx = 0;
//         }
//         out_grid = page->frame->color_grid;
//         build_output_color_dist(pan, out_grid);
         ///////////////////////////////////////////////////////////////
         prod_rec->consumed_elements++;   // total elements processed
         // if there were no active frames then this view is empty.
         //    don't publish it
//         log_info(pan->log,"Pan has %d frames", active_frames);
         if (active_frames == 0) {
            continue;
         }
         // add to frame list (this is an implicit publish)
         add_to_frames(pan->frame_heap, page);
         // auto-compact list by trimming every 4th frame periodically
         if (compaction_count & 1) {
            uint32_t val = compaction_count;
            frame_page_heap_type *heap = pan->frame_heap;
            frame_page_type *head = heap->frames;
            while (val & 1) {
               head = delete_fourth(heap, head);
               val >>= 1;
            }
            // keep compaction counter no bigger than the list, but
            //    make sure it's at least the as big as the list
            compaction_count &= 63;
            assert(63 > MAX_FRAME_HEAP_AVAILABLE);
         }
         compaction_count++;
         //
         if (pan->data_folder) {
            // export pan image at pyramid level 0
            write_panorama_image(pan, page->frame, t, 0);
         }
         ///////////////////////////////
         // even though dta is read differently from other processors,
         //    use std mechanism to report that data is available
         self->elements_produced++;
//         log_info(pan->log, "Signaling data available (sample %ld)",
//               self->elements_produced);
         dp_signal_data_available(self);
         //
      }
   }
   log_info(pan->log, "PANORAMA exit\n");
//write_array_image("pan_grid", &pan->master_color_grid);
//if (out_grid) {
//   write_array_image("out_grid", out_grid);
//}
}


////////////////////////////////////////////////////////////////////////
static void panorama_post_run(
      /* in out */       datap_desc_type *self
      )
{
   (void) self;
//   printf("%s in post_run\n", self->td->obj_name);
//   panorama_class_type *pan = (panorama_class_type *) self->local;
//   if (pan->log_file) {
//      fclose(pan->log_file);
//      free(pan->log_file);
//      pan->log_file = NULL;
//   }
}


////////////////////////////////////////////////////////////////////////
static void * panorama_get_object_at(
      /* in     */ const datap_desc_type *self,
      /* in     */ const uint32_t idx
      )
{
   return &self->void_queue[idx * sizeof(panorama_output_type)];
}


////////////////////////////////////////////////////////////////////////

const frame_page_type * panorama_get_frame_list(
      /* in     */ const panorama_class_type *pan
      )
{
//printf("%.3f PANORAMA returning frame t=%.3f  (%d avail)\n", now(), pan->frame_heap->frames->t, pan->frame_heap->available);
   // 't' is stored in frame
   return pan->frame_heap->frames;
}


////////////////////////////////////////////////////////////////////////
void * panorama_init(
      /* in     */       void *module_setup
      )
{
   panorama_class_type *pan = calloc(1, sizeof(*pan));
   datap_desc_type *self = dp_create();
   pan->log = get_logger(self->td->obj_name);
   report_thread_id(self, pan->log);
   if (c_assert(module_setup != NULL)) {
      log_err(pan->log, "Internal error -- init passed NULL value");
      hard_exit(__func__, __LINE__);
   }
   assert(module_setup != NULL); // 2nd assert to make scan-build happy
   panorama_setup_type *setup = (panorama_setup_type*) module_setup;
   set_log_level(pan->log, LOG_LEVEL_DEFAULT);
   pan->output_type = setup->output_type;
   pan->camera_height = setup->camera_height_meters;
   pan->camera_forward_position = setup->camera_position_forward;
   pan->camera_starboard_position = setup->camera_position_starboard;
   //
   if (setup->logging == 1) {
      const char *folder = get_log_folder_name();
      const char *name = self->td->obj_name;
      size_t len = strlen(folder) + strlen(name) + 2;
      pan->data_folder = malloc(len+1);
      sprintf(pan->data_folder, "%s%s", folder, name);
      if (mkdir(pan->data_folder, 0755) != 0) {
         log_err(pan->log, "Failed to create output directory '%s': %s",
               pan->data_folder, strerror(errno));
         // output directory not available. free allocated memory and
         //    don't output data
         free(pan->data_folder);
         pan->data_folder = NULL;
      } else {
         log_info(pan->log, "Created output directory %s", pan->data_folder);
      }
   }
   free(module_setup);   // allocated on heap and it's no longer needed
   //
   self->local = pan;
   pan->self = self; // keep a reverse link too
   self->pre_run = panorama_pre_run;
   self->post_run = panorama_post_run;
   self->run = panorama_run;
   self->add_producer = panorama_add_producer;
   self->get_object_at = panorama_get_object_at;
   /////////////////
   // once initialization done, put into runtime mode
   dp_execute(self);
   return NULL;
}

