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
#include "pinet.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <alsa/asoundlib.h>
#include <errno.h>
#include "datap.h"
#include "logger.h"
#include "timekeeper.h"

#include "routing/beeper.h"
#include "routing/driver.h"

static beeper_type *beeper_ = NULL;

#include "sound.c"
#include "course.c"

////////////////////////////////////////////////////////////////////////
static void beeper_add_producer(
      /* in out */       datap_desc_type *self,
      /* in     */       datap_desc_type *prod
      )
{
   int num = self->num_attached_producers;
   if (num >= 1) {
      fprintf(stderr, "Too many producers for beeper (max %d). Need "
            "to reconfigure or recompile", MAX_ATTACHED_PRODUCERS);
      fprintf(stderr, "Consumer: %s\n", self->td->obj_name);
      fprintf(stderr, "Producer: %s (%s)\n", prod->td->obj_name,
            prod->td->class_name);
      hard_exit(__FILE__, __LINE__);
   }
   //////////////////////////////////////////////
   // make sure this is a supported producer type
   if (strcmp(prod->td->class_name, DRIVER_CLASS_NAME) != 0) {
      fprintf(stderr, "Attempted to subscribe to incompatible producer\n");
      fprintf(stderr, "Consumer: %s\n", self->td->obj_name);
      fprintf(stderr, "Producer: %s (%s)\n", prod->td->obj_name,
            prod->td->class_name);
      hard_exit(__FILE__, __LINE__);
   }
   // add producer to list
   self->producer_list[num].producer = prod;
   self->producer_list[num].consumed_elements = 0;
   //
   self->num_attached_producers++;
}

////////////////////////////////////////////////////////////////////////

static void beeper_class_pre_run(
      /* in out */       struct datap_desc *dp
      )
{
   int rc = pthread_create(&beeper_->sound_tid, NULL, thread_main, dp);
   if (rc != 0) {
      fprintf(stderr, "Failed to create beeper thread: err=%d\n", rc);
      hard_exit(__FILE__, __LINE__);
   }
}

////////////////////////////////////////////////////////////////////////

static void beeper_class_run(
      /* in out */       struct datap_desc *self
      )
{
   const uint32_t num_producers = self->num_attached_producers++;
   log_info(beeper_->log, "Starting run now");
   //
   while ((self->run_state & DP_STATE_DONE) == 0) {
      dp_wait(self);   // wait for data to become available
      for (uint32_t i=0; i<num_producers; i++) {
         // producer is gaze. it's a standard producers so can use a
         //    generic representation
         producer_record_type *prod_rec = &self->producer_list[i];
         datap_desc_type *prod = prod_rec->producer;
         // process all data from this producer
         while (prod_rec->consumed_elements < prod->elements_produced) {
            ///////////////////////////////////////////////////////////////
            // get data source
            const uint32_t p_idx = (uint32_t)
                  prod_rec->consumed_elements % prod->queue_length;
            const double t = prod->ts[p_idx];
            driver_output_type *driver_data = (driver_output_type*)
                  dp_get_object_at(prod, p_idx);
            ///////////////////////////////
            // look at driver output and decide what sounds are appropriate
            examine_course_data(driver_data, t);
            ///////////////////////////////
            prod_rec->consumed_elements++;   // total elements processed
            ///////////////////////////////
            // report that data is available
            self->elements_produced++;
            dp_signal_data_available(self);
         }
      }
   }
}


////////////////////////////////////////////////////////////////////////
static void beeper_class_post_run(struct datap_desc *dp)
{
   (void) dp;
printf("POST Beeper -- pthread_join\n");
   pthread_join(beeper_->sound_tid, NULL);
printf("POST Beeper -- joined\n");
}


////////////////////////////////////////////////////////////////////////
static void * beeper_class_get_object_at(
      /* in     */ const datap_desc_type *dp,
      /* in     */ const uint32_t idx
      )
{
   return &dp->void_queue[idx * sizeof(beeper_output_type)];
}

////////////////////////////////////////////////////////////////////////
void * beeper_init(void *beeper_setup)
{
   if (beeper_ != NULL) {
      fprintf(stderr, "Multiple beepers defined -- can only have one\n");
      hard_exit(__FILE__, __LINE__);
   }
   if (beeper_setup == NULL) {
      fprintf(stderr, "Internal error -- init passed NULL value");
      hard_exit(__FILE__, __LINE__);
   }
   struct datap_desc *dp = dp_create();
   struct beeper_class *beeper = calloc(1, sizeof(*beeper));
   beeper_ = beeper;
   dp->local = beeper;
   beeper_->log = get_logger(BEEPER_CLASS_NAME);
   report_thread_id(dp, beeper_->log);
   set_log_level(beeper_->log, BEEPER_LOG_LEVEL);
   //
   free(beeper_setup);
   // allocate publish queues (dp->void_queue, dp->ts)
   dp->ts = calloc(1, BEEPER_QUEUE_LEN * sizeof(*dp->ts));
   dp->queue_length = BEEPER_QUEUE_LEN;
   dp->element_size = sizeof(beeper_output_type);
   dp->void_queue = calloc(1, BEEPER_QUEUE_LEN * dp->element_size);
   //
   //
   dp->add_producer = beeper_add_producer;
   dp->pre_run = beeper_class_pre_run;
   dp->post_run = beeper_class_post_run;
   dp->run = beeper_class_run;
   dp->get_object_at = beeper_class_get_object_at;
   // once initialization done, put into runtime mode
   dp_execute(dp);
   return NULL;
}

