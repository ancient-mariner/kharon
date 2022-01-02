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

#include "core_modules/optical_up.h"
#include "core_modules/attitude.h"
#include "core_modules/vy_receiver.h"


#include "project_image.c"

#define OPTICAL_UP_IMG_PRODUCER_IDX   0

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

static void optical_up_add_producer(
      /* in     */       struct datap_desc *self, 
      /* in     */       struct datap_desc *prod
      )
{
   optical_up_class_type *optical_up = (optical_up_class_type *) self->local;
   // make sure this is a supported producer type
   if (strcmp(prod->td->class_name, VY_CLASS_NAME) == 0) {
      // check number of subscriptions
      if (self->num_attached_producers > 1) {
         log_err(optical_up->log, "Too many producers attached to %s (%s)\n",
               self->td->obj_name, self->td->class_name);
      hard_exit(__func__, __LINE__);
      }
      self->producer_list[OPTICAL_UP_IMG_PRODUCER_IDX].producer = prod;
      self->num_attached_producers++;
      //
      vy_class_type *vy = (vy_class_type*) prod->local;
      for (uint32_t lev=0; lev<NUM_PYRAMID_LEVELS; lev++) {
         optical_up->vy_size[lev] = vy->img_size[lev];
      }
      strcpy(optical_up->camera_host, vy->device_name);
      optical_up->camera_num = vy->camera_num;
   } else if (strcmp(prod->td->class_name, ATTITUDE_CLASS_NAME) == 0) {
      // only one attitude subscription allowed
      if (optical_up->attitude_producer != NULL) {
         log_err(optical_up->log, "Attitude producer already attached to "
               "%s (%s)\n", self->td->obj_name, self->td->class_name);
      hard_exit(__func__, __LINE__);
      }
      // keep record of this producer but don't formally subscribe
      //    (data pulled only when it's needed, not when it's available)
      optical_up->attitude_producer = prod;
   } else {
      log_err(optical_up->log, 
            "Attempted to subscribe to incompatible producer\n");
      log_err(optical_up->log, "Consumer: %s\n", self->td->obj_name);
      log_err(optical_up->log, "Producer: %s (%s)\n", 
            prod->td->obj_name, prod->td->class_name);
      hard_exit(__func__, __LINE__);
   } 
}


////////////////////////////////////////////////////////////////////////
static void optical_up_class_pre_run(struct datap_desc *self)
{
   optical_up_class_type *optical_up = (optical_up_class_type *) self->local;
//
//   // sanity checks
//   if ((c_assert(VY_FOV_HORIZ_DEG == 62.2f) != 0) ||
//         (c_assert(VY_FOV_VERT_DEG == 48.8f) != 0)) {
//      log_err(optical_up->log, 
//            "Internal config error: FOV change -> readjust constants");
//   }
//   const uint32_t pyramid_offset[NUM_PYRAMID_LEVELS] = { 
//      0, 
//      SZ0 * SZ0
////      SZ0 * SZ0 + SZ1 * SZ1
//   };
      datap_desc_type *vy_prod = 
            self->producer_list[OPTICAL_UP_IMG_PRODUCER_IDX].producer;
      vy_class_type *vy = (vy_class_type*) vy_prod->local;
   // allocate and initialize storage 
   // accumulators
   for (uint32_t lev=0; lev<NUM_PYRAMID_LEVELS; lev++) {
      // accumulator size is output size
      // size of output is based on FOV and DPP. allow vy to be rotated
      //    up to 30 degrees before image starts getting truncated (at
      //    horizon). note that image expands 10% at 25 degrees off horizon
      //    so max rotation won't be realizable up there. but if the seas
      //    are so rough that a rotated image is that inclined, tracking is
      //    going to be a challenge in absolute terms
      // account for input image being submitted at up to 30 degrees tilt. any
      //    content pushed to accumulator outside of accum bounds is ignored
      // at 30 degree tilt, width will be (horiz*.866 + vert*0.5) * ppd
      double ppd = PIX_PER_DEG[lev];
      double dpp = DEG_PER_PIX[lev];
      optical_up->pix_per_degree[lev] = ppd;
      image_size_type sz = get_upright_size(vy->fov_horiz,
            vy->fov_vert, optical_up->pix_per_degree[lev]);
      optical_up->size[lev] = sz;
      if (lev == 0) {
         optical_up->size_horiz.degrees = (double) sz.x * dpp;
         optical_up->size_vert.degrees = (double) sz.y * dpp;
         uint32_t n_pix = (uint32_t) (sz.x * sz.y);
//printf("upright size: %d,%d  (%f,%f) ppd %f\n", sz.x, sz.y, (double) optical_up->size_horiz.degrees, (double) optical_up->size_vert.degrees, (double) ppd);
         // allocate blur buf here. lowest pyramid level is the largest.
         //    allocate buffer for it and it'll be large enough for the
         //    other layers
         optical_up->blur_buf = malloc(n_pix * sizeof *optical_up->blur_buf);
      }
      optical_up->accum[lev] = create_vy_accumulator(sz,
            optical_up->size_horiz, optical_up->size_vert);
   }
   // set element size and queue length
   self->element_size = sizeof(optical_up_output_type);
   self->queue_length = OPTICAL_UP_QUEUE_LEN;
   self->ts = calloc(1, OPTICAL_UP_QUEUE_LEN * sizeof(*self->ts));
   self->void_queue = calloc(1, OPTICAL_UP_QUEUE_LEN * self->element_size);
   // get necessary size of buffer to store output frame data in each
   //    output struct
   uint32_t n_pyr_pix = 0;
   for (uint32_t lev=0; lev<NUM_PYRAMID_LEVELS; lev++) {
      image_size_type sz = optical_up->size[lev];
      uint32_t n_pix = (uint32_t) (sz.x * sz.y);
      n_pyr_pix += n_pix;
   }
   // initialize data in each output buffer
   for (uint32_t i=0; i<OPTICAL_UP_QUEUE_LEN; i++) {
      optical_up_output_type *out = (optical_up_output_type *)
            dp_get_object_at(self, i);
      // build buffer
      out->pyramid_ = malloc(n_pyr_pix * sizeof *out->pyramid_);
      // set size values and set frame offsets into image buffer
      uint32_t offset = 0;
      for (uint32_t lev=0; lev<NUM_PYRAMID_LEVELS; lev++) {
         image_size_type sz = optical_up->size[lev];
         out->size[lev] = sz;
         uint32_t n_pix = (uint32_t) (sz.x * sz.y);
         out->frame[lev] = &out->pyramid_[offset];
         offset += n_pix;
      }
   }
   //
   // make sure both image and attitude subscribers are connected
   // if not, treat as hard error. we could let this module simply exit,
   //    but some downstream modules depending on it could block waiting
   //    for its input
   uint32_t errs = 0;
   if (optical_up->attitude_producer == NULL) {
      log_err(optical_up->log, "Process %s is does not have attitude input", 
            self->td->obj_name);
      errs++;
   }
   if (errs > 0) {
      hard_exit(__func__, __LINE__);
   }
   dp_reload_config(self);
}


////////////////////////////////////////////////////////////////////////

static void optical_up_class_run(
      /* in out */       datap_desc_type *self
      )
{
   struct producer_record *img_pr = 
         &self->producer_list[OPTICAL_UP_IMG_PRODUCER_IDX];
   datap_desc_type *img_producer = img_pr->producer;
   vy_class_type *vy = (vy_class_type*) img_producer->local;
   //
   optical_up_class_type *optical_up = (optical_up_class_type *) self->local;
   attitude_output_type att_out; // rotation matrix ship->world
   while ((self->run_state & DP_STATE_DONE) == 0) {
//      log_info(optical_up->log, "Waiting for data");
      dp_wait(self);   // wait for data to become available
      while (img_pr->consumed_elements < img_producer->elements_produced) {
//         log_info(optical_up->log, "Consuming %ld of %ld", 
//               img_pr->consumed_elements, img_producer->elements_produced);
         ///////////////////////////////
         // get data source
         const uint32_t p_idx = (uint32_t)
               img_pr->consumed_elements % img_producer->queue_length;
         vy_receiver_output_type *img_src = (vy_receiver_output_type*)
               dp_get_object_at(img_producer, p_idx);
         const double t = img_producer->ts[p_idx];
         // get approximate attitude X when frame captured
         // captured at ~30fps, so assume a 30ms delay from when
         //    frame is delivered relative to IMU report
         const double FRAME_DELAY = 0.030;
         enum attitude_query_state status = NA;
         get_attitude(optical_up->attitude_producer, optical_up->log, 
               t-FRAME_DELAY, &status, &att_out, 
               &optical_up->last_attitude_idx);
         if (status == PENDING) {
            // timestamp not yet available. wait and reprocess this frame
            log_info(optical_up->log, "Attitude timestamp not available");
            break;
         } else if (status == MISSING) {
            // no attitude information for this frame. alignment not possible
            // drop frame
            img_pr->consumed_elements++;
            continue;
         }
         ///////////////////////////////
         // get data sink
         const uint32_t idx = (uint32_t) 
               (self->elements_produced % self->queue_length);
         self->ts[idx] = t;
         optical_up_output_type *output = (optical_up_output_type *)
               dp_get_object_at(self, idx);
         ///////////////////////////////
         // process data
         raw_image_to_accumulators(optical_up, img_src, &att_out, vy,
               &output->world_center);
         // TODO if inverting latitude, explain why (image is upside down, but why)
         output->world_center.latitude.sangle32 =
               -output->world_center.latitude.sangle32;
         log_info(optical_up->log, 
               "  (%.3f)  image center at: lat=%f, lon=%f", t, 
               (double) output->world_center.lat.sangle32 * BAM32_TO_DEG,
               (double) output->world_center.lon.angle32 * BAM32_TO_DEG);
//printf("%.3f   %s image center at: lat=%f, lon=%f\n", t, self->td->obj_name, (double) output->world_center.lat, (double) output->world_center.lon);
         flatten_accumulators(optical_up, output);
         blur_output_r1(optical_up, output);
         if (optical_up->data_folder != NULL) {
            save_pnm_file(optical_up, t, output);
         }
//exit(0);
         // store copy of ship2world in output
         copy_matrix(&att_out.ship2world, &output->ship2world);
         output->heading.degrees = att_out.true_heading.degrees;
         img_pr->consumed_elements++;   // total elements processed
         //
         ///////////////////////////////
         // report that data is available
         self->elements_produced++;
         log_info(optical_up->log, "Signaling data available (sample %ld)",
                              self->elements_produced);
         dp_signal_data_available(self);
      }
   }
}


////////////////////////////////////////////////////////////////////////
//static void optical_up_class_post_run(
//      /* in out */       datap_desc_type *self
//      )
//{
//   printf("%s in post_run\n", self->td->obj_name);
//}


////////////////////////////////////////////////////////////////////////
static void * optical_up_get_object_at(
      /* in     */ const datap_desc_type *self,
      /* in     */ const uint32_t idx
      )
{  
   return &self->void_queue[idx * sizeof(optical_up_output_type)];
} 


////////////////////////////////////////////////////////////////////////
static void optical_up_reload_config(
      /* in out */       datap_desc_type *self
      )
{
   // only one input module. load its alignment matrix
   producer_record_type *pr = &self->producer_list[0];
   datap_desc_type *producer = pr->producer;
   optical_up_class_type *upright = (optical_up_class_type*) self->local;
   module_config_load_cam_to_ship(upright->log, upright->camera_host, 
         producer->td->obj_name, &upright->cam2ship);
}


////////////////////////////////////////////////////////////////////////
void * optical_up_class_init(
      /* in     */       void *optical_setup
      )
{
   datap_desc_type *self = dp_create();
   optical_up_class_type *upright = calloc(1, sizeof(*upright));
   upright->log = get_logger(self->td->obj_name);
   report_thread_id(self, upright->log);
   set_log_level(upright->log, OPTICAL_UP_LOG_LEVEL);
   if (c_assert(optical_setup != NULL)) {
      log_err(upright->log, "Internal error -- init passed NULL value");
      hard_exit(__func__, __LINE__);
   }
   assert(optical_setup != NULL);   // 2nd assert for scan-build
   optical_up_setup_type *setup = (optical_up_setup_type*) optical_setup;
   if (setup->logging == 1) {
      const char *folder = get_log_folder_name();
      const char *name = self->td->obj_name;
      size_t len = strlen(folder) + strlen(name) + 2;
      upright->data_folder = malloc(len+1);
      sprintf(upright->data_folder, "%s%s", folder, name);
      if (mkdir(upright->data_folder, 0755) != 0) {
         log_err(upright->log, "Failed to create output directory '%s': %s",
               upright->data_folder, strerror(errno));
         // output directory not available. free allocated memory and
         //    don't output data
         free(upright->data_folder);
         upright->data_folder = NULL;
      } else {
         log_info(upright->log, "Created output directory %s", 
               upright->data_folder);
      }
   } else {
      upright->data_folder = NULL;
   }
   free(optical_setup);   // allocated on heap and it's no longer needed
   //
   identity_matrix(&upright->cam2ship);
   upright->last_attitude_idx = 0;
   //
   self->local = upright;
   self->pre_run = optical_up_class_pre_run;
   //self->post_run = optical_up_class_post_run;
   self->run = optical_up_class_run;
   self->add_producer = optical_up_add_producer;
   self->get_object_at = optical_up_get_object_at;
   self->reload_config = optical_up_reload_config;
   /////////////////
   // once initialization done, put into runtime mode
   dp_execute(self);
   return NULL;
}

