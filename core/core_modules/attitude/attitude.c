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
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <errno.h>
#include <dirent.h>
#include <math.h>
#include "datap.h"
#include "lin_alg.h"
#include "sensor_packet.h"
#include "kernel.h"
#include "logger.h"
#include "timekeeper.h"

#include "core_modules/attitude.h"
#include "core_modules/imu_receiver.h"

// new style approach -- use static log entry to avoid requiring
//    parent pointer
static __thread log_info_type *log_ = NULL;

// magnetic declination
static declination_type declination_ = { .degrees = 0.0 };

// source amalgamation
#include "imu_streams.c"
#include "publish.c"
#include "stream_logic.c"

// publishes rotation matrix to align ship coordinate to world
//    coordinate frame
// mag vector is projected onto plane orthogonal to acc vector, which is up.
//    these 2 vectors are used to generate ship2world matrix
// logged data are up and north vectors from which rotation matrix
//    is calculated from

// on startup sensors are blind and there's literally no way to know
//    which way is up. compensate for this by using acc/mag initially
//    and slowly bringing gyro online. the interval here is how many
//    seconds until the gyro reaches full strength (ie, its maximal
//    influence determining roll, tilt and yaw).


// version 1: pull data from single IMU
// version 2: data from multiple IMUs, primary and secondary, with secondary
//             for when primary is unavailable
// version 3: combine data from multiple sources
// version 4: like 3, but with overhauled stream combination algorithm; reset
//             simplified

/** version 4  (post archive 334)
   Bugs in v3 resulted in poor attitude control. complexity of algorithm
   and obvious problems in fallback (priority) system led to complete rewrite.

   Attempted to remove reset but image artifacts are too high during
   (re)bootstrapping that the attitude signal is essentually useless.
   reset resumed, but only for initial bootstrap and after gyro signal
   is lost
**/

/** version 3  (post archive 302)
All modalities have prioritized elements
Publication is delayed to provide time for data to arrive. Window length
   is W milliseconds (e.g., 100), publication interval I, previous sample
   at time T. this is to simplify logic in attitude module, which is
   uncomfortably complicated (too many logic paths to achieve fastest-time
   response)
After a sample is received with timestamp greater than T+W+I, IMU producers
   are scanned and a sample is generated for T+I. only data available
   at that time is used (e.g., if an IMU producer is offline or delayed
   then it's values won't be used). imu data is used based on the
   priority of each modality in each IMU producer
Output data is complementary filtered. It was considered to disable
   complementary filter when ACC signal varied from baseline (e.g.,
   more than +/-10% from baseline), as strong acceleration could
   add noise to system, but the low weight of ACC,MAG in the filter
   would average out noise over time, and use of a cutoff threshold
   could itself introduce unexpected biases (e.g., if motion was
   above threshold in one direction but below in opposite, correcting
   signal to gyro would be biased)
When ACC or MAG data is unavailable for a period of time, that's OK, as
   the gyro is the primary data source, and some data is better than none.
When GYR data is unavailable, go into reset mode. reset mode should last
   3x the duration that GYR data is unavailable, up to maximum reset time

TODO provide weak bias to gyro so that Y is always positive. in event
   that ACC goes offline for a long period, an up Y is the best bet and
   it prevents the gyro from rotating to a pathological position.
   if MAG is unavailable then at worst case north will be lost. provide
   this artificial up whenever ACC is offline

Bootstrapping (reset) is done when gyro shows slow rotation (e.g., <2dps)
   and acc is stable (e.g., <5% change over 2 seconds). start with acc/mag
   weight at 1.0 and gyro at 0.0. slowly change balance over window
   (e.g., 5sec) until gyro reaches full strength (e.g., 0.995). time only
   considered when stability criteria met

modalities are "acc", "mag", "gyr"

**/

// use pointer array for xyzt queue, with array index being producer idx
// pointers reference elements in imu_history

////////////////////////////////////////////////////////////////////////

// the only supported producer is imu receiver. if additional producers
//    types are necessary, imu receivers should be moved into an
//    independent list
static void attitude_add_producer(
      /* in out */       datap_desc_type *self,
      /* in     */       datap_desc_type *prod
      )
{
   attitude_class_type *att = (attitude_class_type*) self->local;
   // make sure this is a supported producer type
   if (strcmp(prod->td->class_name, IMU_CLASS_NAME) == 0) {
      // add producer to list
      uint32_t num = self->num_attached_producers++;
      log_info(att->log, "%s is producer #%d", prod->td->obj_name, num);
      self->producer_list[num].producer = prod;
      self->producer_list[num].consumed_elements = 0;
   } else {
      fprintf(stderr, "Attempted to subscribe to incompatible producer\n");
      fprintf(stderr, "Consumer: %s\n", self->td->obj_name);
      fprintf(stderr, "Producer: %s (%s)\n", prod->td->obj_name,
            prod->td->class_name);
      hard_exit("attitude_class::add_producer", 1);
   }
}


////////////////////////////////////////////////////////////////////////

// the only supported producer is imu receiver. if additional producers
//    types are necessary, imu receivers should be moved into an
//    independent list
static void initialize_streams(
      /* in out */       datap_desc_type *self
      )
{
   attitude_class_type *att = (attitude_class_type*) self->local;
   att->master_gyro_idx = MAX_ATTACHED_PRODUCERS;
   //
   att->active_tau = 1.0f;
   // make sure this is a supported producer type
   for (uint32_t i=0; i<self->num_attached_producers; i++) {
      datap_desc_type *prod = self->producer_list[i].producer;
      // get priorities of this IMU producer and create streams
      imu_class_type *imu = (imu_class_type*) prod->local;
//printf("imu %s has priorities %d,%d,%d\n", prod->td->obj_name, imu->priority[IMU_GYR], imu->priority[IMU_ACC], imu->priority[IMU_MAG]);
      switch (imu->priority[IMU_GYR]) {
         case IMU_PRI_1:
            if (att->master_gyro_idx >= MAX_ATTACHED_PRODUCERS) {
               att->master_gyro_idx = i;
            }
            att->num_p1_gyr++;   // fall through
         case IMU_PRI_2:         // fall through
         case IMU_PRI_3:
            att->gyr_stream[i] = malloc(sizeof *att->gyr_stream[i]);
            resample_stream_init(att->gyr_stream[i], imu->priority[IMU_GYR]);
            break;
         default:
            ;
      };
      switch (imu->priority[IMU_ACC]) {
         case IMU_PRI_1:
            att->num_p1_acc++;   // fall through
         case IMU_PRI_2:         // fall through
         case IMU_PRI_3:
            att->acc_stream[i] = malloc(sizeof *att->acc_stream[i]);
            simple_stream_init(att->acc_stream[i], imu->priority[IMU_ACC]);
            break;
         default:
            ;
      };
      switch (imu->priority[IMU_MAG]) {
         case IMU_PRI_1:
            att->num_p1_mag++;   // fall through
         case IMU_PRI_2:         // fall through
         case IMU_PRI_3:
            att->mag_stream[i] = malloc(sizeof *att->mag_stream[i]);
            simple_stream_init(att->mag_stream[i], imu->priority[IMU_MAG]);
            break;
         default:
            ;
      };
   }
   ///////////////
   // make sure there are priority-1 inputs for all modalities
   if ((att->num_p1_gyr == 0) ||
         (att->num_p1_acc == 0) ||
         (att->num_p1_mag == 0)) {
      log_err(att->log, "Attitude requires at least one priority-1 input "
            "from each GYR, ACC and MAG");
      hard_exit(__func__, __LINE__);
   }
}
static void attitude_class_pre_run(
      /* in out */       datap_desc_type *dp
      )
{
   // allocate publish queues (dp->void_queue, dp->ts)
   dp->ts = calloc(1, ATTITUDE_QUEUE_LEN * sizeof(*dp->ts));
   dp->element_size = sizeof(attitude_output_type);;
   dp->queue_length = ATTITUDE_QUEUE_LEN;
   dp->void_queue = calloc(1, ATTITUDE_QUEUE_LEN * dp->element_size);
   dp_reload_config(dp);
   /////////////////////////////////////////////////////////////////////
   initialize_streams(dp);
}


////////////////////////////////////////////////////////////////////////
static void attitude_class_run(
      /* in out */       datap_desc_type *self
      )
{
   attitude_class_type *att = (attitude_class_type*) self->local;
   // if no producers there's nothing to do so exit
   if (self->num_attached_producers == 0) {
      // sanity check -- make sure there are no subscribers
      // this is a configuration error -- treat it as fatal
      if (self->num_attached_consumers > 0) {
         log_err(att->log, "Module has consumer(s) but no producer");
         hard_exit(__func__, __LINE__);
      }
      return;
   }
   /////////////////////////////////////////////////////////////////////
   // bootstrap
   // get first available output sample and use that as a base time
   //    for publishing data
   while ((self->run_state & DP_STATE_DONE) == 0) {
      dp_wait(self);
      // feed any available data to streams
      update_vector_streams(self, att);
      // when we have ACC and MAG data, select a time for the first
      //    sample to be published.
      //    if gyro is absent, publication time will automatically be
      //    reset to a time when there is data
      int avail_count = 0;
      // check ACC streams for content
      for (uint32_t i=0; i<MAX_ATTACHED_PRODUCERS; i++) {
         simple_vector_stream_type *stream = att->acc_stream[i];
         if ((stream != NULL) && (stream->timestamp.usec > 0)) {
            avail_count++;
            break;
         }
      }
      // check MAG streams for content
      for (uint32_t i=0; i<MAX_ATTACHED_PRODUCERS; i++) {
         simple_vector_stream_type *stream = att->mag_stream[i];
         if ((stream != NULL) && (stream->timestamp.usec > 0)) {
            avail_count++;
            break;
         }
      }
      if (avail_count < 2) {
         continue;   // MAG or ACC not available yet. keep waiting
      }
      // choose publication time and exit loop
      microsecond_timestamp_type present_t = timestamp_from_real(now());
      present_t.usec /= SAMPLE_DUR_USEC;
      present_t.usec *= SAMPLE_DUR_USEC;
      att->next_publish_time.usec = present_t.usec + SAMPLE_DUR_USEC;
      break;
   }
   att->init_timer.seconds = BOOTSTRAP_INTERVAL_SEC;
   /////////////////////////////////////////////////////////////////////
   // main processing loop
   uint32_t wait_for_data = 0;
   while ((self->run_state & DP_STATE_DONE) == 0) {
      if (wait_for_data) {
         dp_wait(self);
      }
      wait_for_data = 0;
      if ((self->run_state & DP_STATE_DONE) != 0) {
         break;
      }
      microsecond_timestamp_type present_t = timestamp_from_real(now());
      // see if there's content to be published by applying delay window
      //    to next timestamp. if not, go back to sleep and wait for data
      if (att->next_publish_time.usec < present_t.usec) {
         // previous publication is sufficiently far in the past. check to
         //    see if there's new content to publish
         // push new data to streams
         update_vector_streams(self, att);
         // timeout is when ACC and MAG streams are considered offline
         microsecond_type timeout =
               { .usec = present_t.usec - ACC_MAG_TIMEOUT_USEC };
         // check streams. if previous publication too far in past, force
         //    a new publication based on what's available. otherwise, only
         //    publish if sufficient data is available
//log_info(att->log, "now=%.3f, publish=%.3f, thresh=%.3f", (double) present_t.usec * 1.0e-6, (double) att->next_publish_time.usec * 1.0e-6, (double) (att->next_publish_time.usec + DELAY_WINDOW_US) * 1.0e-6);
         if (present_t.usec > att->next_publish_time.usec + DELAY_WINDOW_US) {
            // if no data published (return 0) then wait for more data.
            //    otherwise try to publish again
            if (force_publish_next_sample(self, att, timeout) == 0) {
               wait_for_data = 1;
            }
         } else {
            // if no data published (return 0) then wait for more data.
            //    otherwise try to publish again
            if (publish_if_available(self, att, timeout) == 0) {
               wait_for_data = 1;
            }
         }
      } else {
         wait_for_data = 1;
      }
   }
}

////////////////////////////////////////////////////////////////////////
static void attitude_class_post_run(
      /* in out */       datap_desc_type *dp
      )
{
//   printf("%s in post_run\n", dp->td->obj_name);
   attitude_class_type *attitude = (attitude_class_type*) dp->local;
   if (attitude->logfile != NULL) {
      fclose(attitude->logfile);
      attitude->logfile = NULL;
   }
}


////////////////////////////////////////////////////////////////////////
static void * attitude_get_object_at(
      /* in     */ const datap_desc_type *dp,
      /* in     */ const uint32_t idx
      )
{
   assert(idx < dp->queue_length);
   return &dp->void_queue[idx * sizeof(attitude_output_type)];
}


////////////////////////////////////////////////////////////////////////
void * attitude_class_init(void *att_setup)
{
   // TODO init class queues to NULL
   // TODO set reset flag
   // TODO make sure all of att class is initialized properly

   attitude_class_type *attitude = calloc(1, sizeof(*attitude));
   datap_desc_type *dp = dp_create();
   attitude->log = get_logger(dp->td->obj_name);
   report_thread_id(dp, attitude->log);
   log_ = attitude->log;
   // set static logger
   set_log_level(attitude->log, ATTITUDE_LOG_LEVEL);
   if (c_assert(att_setup != NULL)) {
      log_err(attitude->log, "Internal error -- init passed NULL value");
      hard_exit(__func__, __LINE__);
   }
   attitude_setup_type *setup = (attitude_setup_type *) att_setup;
   assert(setup != NULL);    // 2nd assert to keep scan-build happy
   //
   if (setup->logging == 1) {
      // open logfile
      char buf[STR_LEN];
      // construct path
      snprintf(buf, STR_LEN, "%s%s", get_log_folder_name(), dp->td->obj_name);
      attitude->logfile = fopen(buf, "w");
      if (attitude->logfile == NULL) {
         // non-fatal error. we just loose logging
         log_err(attitude->log, "Unable to create logfile %s", buf);
      } else {
         log_info(attitude->log, "logging data to %s", buf);
         log_info(attitude->log, "format: time  heading  pitch  roll");
         fprintf(attitude->logfile, "# time       \theading\tpitch   \troll (heading true)\n");
      }
   } else {
      attitude->logfile = NULL;
   }
   free(setup);   // allocated on heap and it's no longer needed
   ////////////////////////////////////////
   // variable initialization
   // calloc implicitly initializes stream arrays, counters, and
   //    vectors to zero
   // bootstrap timer set when used
   ////////////////////////////////////////
   // attitude produces data at a relatively high rate (e.g., 100Hz). don't
   //    alert consumers on every update, to reduce noise. update_interval
   //    is number of data samples that are collected before consumers
   //    are alerted (data is still published when it's available -- the
   //    broadcast notification is just less frequent)
   dp->update_interval = 4;
   //
   dp->local = attitude;
   dp->pre_run = attitude_class_pre_run;
   dp->post_run = attitude_class_post_run;
   dp->run = attitude_class_run;
   dp->add_producer = attitude_add_producer;
   dp->get_object_at = attitude_get_object_at;
   // once initialization done, put into runtime mode
   dp_execute(dp);
   return NULL;
}


// sets magnetic declination (called by mapping system on map refresh)
void set_declination(
      /* in     */ const declination_type decl
      )
{
   declination_ = decl;
}

