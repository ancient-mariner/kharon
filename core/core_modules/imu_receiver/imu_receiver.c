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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include "datap.h"
#include "pinet.h"
#include "logger.h"
#include "kernel.h"
#include "lin_alg.h"
#include "dev_info.h"
#include "sensor_packet.h"
#include "udp_sync.h"
#include "sensor_packet.h"

#include "core_modules/imu_receiver.h"

#define SAMPLE_INTERVAL_US    IMU_PRODUCER_INTERVAL_US

#include "receiver_logic.c"
#include "compass_correction.c"

////////////////////////////////////////////////////////////////////////
static void imu_class_pre_run(
      /* in out */       struct datap_desc *dp
      )
{
   // allocate dp->void_queue, dp->ts
   dp->ts = calloc(1, IMU_QUEUE_LEN * sizeof(*dp->ts));
   dp->element_size = sizeof(imu_output_type);;
   dp->queue_length = IMU_QUEUE_LEN;
   dp->void_queue = calloc(1, IMU_QUEUE_LEN * dp->element_size);
   // each receiver should signal frame arrival 1/N times for N
   //    receivers
   ////////////////////////////////////////
   struct imu_class *imu = (struct imu_class*) dp->local;
   const char * name = dp->td->obj_name;
   ////////////////////////////////////////
   // saving state for interplotion
   imu->prev_publish_t.usec = 0;
   // sanity check
   if (imu->priority[0] < 0) {
      // priority not set. complain and exit
      fprintf(stderr, "Need to call 'set_imu_priority() on %s\n", name);
      exit(1);
   }
   ////////////////////////////////////////
   // setup networking
   // this config data is static so don't include it in reload_config
   // get socket port
   char port_str[STR_LEN];
   FILE *fp = open_config_file_ro2(NULL, NULL, "endpoints", name, NULL);
   if (!fp) {
      log_err(imu->log, "Unable to open endpoint file for '%s'", name);
      goto err;
   }
   if (config_read_string(fp, port_str, STR_LEN) != 0) {
      log_err(imu->log, "Unable to determine listening port for '%s'", name);
      goto err;
   }
   if (fp) {
      fclose(fp);
   }
   //
   uint32_t port = (uint32_t) atoi(port_str);
   if (c_assert(port <= 0x00007fff)) {
      log_err(imu->log,
            "Configuration error: port size greater than int16 (%d)", port);
      goto err;
   }
   log_info(imu->log, "%s listening on port %d", name, port);
printf("%s listening on port %d\n", name, port);
   if ((imu->sockfd = init_server((int16_t) port)) < 0) {
      log_err(imu->log, "Failed to initialize server for %s", name);
      // treat as fatal error until recovery logic worked out
      goto err;
   }
   ////////////////////////////////////////
   // load config data (rotation matrix - device to ship)
   dp_reload_config(dp);
   //
   return;
err:
   if (fp)
      fclose(fp);
   hard_exit("imu_receiver::imu_class_pre_run", 1);
}

////////////////////////////////////////////////////////////////////////
// publishes sensor data received from network (primarily IMU data)
// gyro drift and magnetic correction assumed to be done at client
// complementary filter for acc and mag NO LONGER done at client.
//    a new module (attitude) is required to pull together different sensor
//    sources and integrate data from different modalities.
//    up/north calculations done in this separate module
static void imu_class_run(struct datap_desc *dp)
{
   struct imu_class *imu = (struct imu_class*) dp->local;
   log_info(imu->log, "Modality priorities (0-based): g:%d, a:%d m:%d",
      imu->priority[IMU_GYR], imu->priority[IMU_ACC], imu->priority[IMU_MAG]);
   struct sensor_packet_header header;
   double timestamp;
   uint32_t pkt_type;
   //
   char serial[SP_SERIAL_LENGTH];   // buffer for pulling serialized data
   //
   uint64_t report_level = 100;
   imu_sensor_packet_type ship_vectors;
//printf("%s entering main loop\n", dp->td->obj_name);
   while ((dp->run_state & DP_STATE_DONE) == 0) {
      // establish server
//printf("establish connection\n");
      start_network(dp);
      // send sync packet so client has current time
      send_sync_packet(UDP_SYNC_PACKET_TIME);
      // server is up. pull data from attached client
      while ((dp->run_state & DP_STATE_DONE) == 0) {
         ///////////////////////////////////////////////////////////////
         // fetch sensor data header
         if (recv_block(imu->connfd, &header, sizeof(header)) < 0) {
            log_err(imu->log, "Read error. Breaking connection");
            close(imu->connfd);
            imu->connfd = -1;
            break;
         }
//printf("%s received header block\n", dp->td->obj_name); fflush(stdout);
         // retrieve and handle metadata
         unpack_sensor_header(&header, &pkt_type, &timestamp);
         if (pkt_type != IMU_PACKET_TYPE) {
            log_err(imu->log, "Packet type error. Expected 0x%08x, got 0x%08x",
                  IMU_PACKET_TYPE, pkt_type);
            hard_exit("imu_receiver::imu_class_run", 1);
         }
         // remote log data returned in packet header
         if (header.log_data[0] != 0) {
            header.log_data[SENSOR_PACKET_LOG_DATA-1] = 0;
            log_info(imu->log, "remote log: '%s'", header.log_data);
         }
         ///////////////////////////////////////////////////////////////
         // fetch sensor data
         if (recv_block(imu->connfd, serial, SP_SERIAL_LENGTH) !=
               SP_SERIAL_LENGTH) {
            log_err(imu->log, "\n%s read error. Breaking connection",
                  dp->td->obj_name);
            close(imu->connfd);
            imu->connfd = -1;
            break;
         }
         // convert from network to local representation and log data
         // log data as it was received, transform sensor axes to
         //    ship axes, and store in output buffer
         unpack_data(timestamp, serial, &ship_vectors, imu);
         rotate_and_log(&ship_vectors, imu);
         // upsample data and publish
         publish_upsample(dp, imu, &ship_vectors);
         // report every Nth element, give or take
         if (dp->elements_produced > report_level) {
            report_level = (uint32_t) (dp->elements_produced + 100);
            log_info(imu->log, "Published %ld'th sample at %.3f", dp->elements_produced, timestamp);
         }
      }
   }
//printf("%s leaving main loop\n", dp->td->obj_name); fflush(stdout);
}


////////////////////////////////////////////////////////////////////////
static void imu_class_post_run(struct datap_desc *dp)
{
   struct imu_class *imu = (struct imu_class*) dp->local;
   log_info(imu->log, "in post_run()", dp->td->obj_name);
   if (imu->connfd >= 0) {
      close(imu->connfd);
      usleep(2000);     // give client a chance to close socket first
   }
   if (imu->sockfd >= 0) {
      close(imu->sockfd);
      imu->sockfd = -1;
   }
   if (imu->logfile) {
      fclose(imu->logfile);
      imu->logfile = NULL;
   }
}


////////////////////////////////////////////////////////////////////////
// induce process termination
// process won't abort if socket is waiting to accept a
//    connection, so shutdown socket
static void imu_abort(struct datap_desc *dp)
{
   struct imu_class *imu = (struct imu_class*) dp->local;
   log_info(imu->log, "%s aborting", dp->td->obj_name);
   if (imu->connfd >= 0) {
      if (shutdown(imu->connfd, SHUT_RDWR) != 0) {
         log_err(imu->log, "Error shutting down connection (%s)",
               strerror(errno));
      }
      imu->connfd = -1;
   }
   if (imu->sockfd >= 0) {
      if (shutdown(imu->sockfd, SHUT_RDWR) != 0) {
         log_err(imu->log, "Error shutting down socket (%s)", strerror(errno));
      }
      imu->sockfd = -1;
   }
}

////////////////////////////////////////////////////////////////////////
static void * imu_get_object_at(
      /* in     */ const datap_desc_type *self,
      /* in     */ const uint32_t idx
      )
{
   return &self->void_queue[idx * sizeof(imu_output_type)];
}


////////////////////////////////////////////////////////////////////////
static void imu_reload_config(
      /* in out */       datap_desc_type *self
      )
{
printf("Loading config\n");
   imu_class_type *imu = (struct imu_class*) self->local;
   module_config_load_imu_to_ship(self->td->obj_name, imu);
}


////////////////////////////////////////////////////////////////////////
void * imu_class_init(void * imu_setup)
{
   struct datap_desc *dp = dp_create();
   struct imu_class *imu = calloc(1, sizeof(*imu));
   if (c_assert(imu_setup != NULL)) {
      log_err(imu->log, "Internal error -- init passed NULL value");
      hard_exit(__func__, __LINE__);
   }
   imu_setup_type *setup = (imu_setup_type *) imu_setup;
   /////////////////////////////////////////////////////////////////////
   // setup logging
   imu->log = get_logger(dp->td->obj_name);
   report_thread_id(dp, imu->log);
   set_log_level(imu->log, IMU_LOG_LEVEL);
   imu->priority[0] = -1;  // flag as invalid
   /////////////////////////////////////////////////////////////////////
   // upsample values
   for (uint32_t i=0; i<NUM_IMU_CHANNELS; i++) {
      imu->recycle_timer_usec[i] = 0;
      zero_vector(&imu->recycle_value[i]);
   }
   imu->prev_publish_t.usec = 0;
   /////////////////////////////////////////////////////////////////////
   strcpy(imu->device_name, setup->device_name);
   if (setup->logging == 1) {
      // open logfile
      char buf[STR_LEN];
      // construct path
      snprintf(buf, STR_LEN, "%s%s", get_log_folder_name(), dp->td->obj_name);
      imu->logfile = fopen(buf, "w");
      if (imu->logfile == NULL) {
         // non-fatal error. we just loose logging
         log_err(imu->log, "Unable to create data logfile for %s (%s)",
               dp->td->obj_name, buf);
      } else {
         log_info(imu->log, "%s logging data to %s", dp->td->obj_name, buf);
         log_info(imu->log, "format: gyro (3), acc (3), mag (3), temp");
      }
   } else {
      imu->logfile = NULL;
   }
   free(setup);   // allocated on heap and it's no longer needed
   // load compass correction if it's available
   load_compass_correction(imu);
   //
   dp->local = imu;
   imu->sockfd = -1;
   imu->connfd = -1;
   // data frames are read all together, so no need for each imu producer to
   //    alert subscribers. the update signal (update_ctr -- see
   //    datap.h) initialization should be staggered so that multiple
   //    alerts from different IMU receivers aren't sent to subscribers
   //    at the same time, but as receivers can fall out of sync, this
   //    isn't directly possible.
   // TODO consider setting the update_interval to 1 so that an
   //    alert is set every time data is said to be available, but add
   //    auxiliary logic to control when a signal is sent that data is
   //    available
   dp->update_ctr = 0;
   dp->update_interval = 2;
   //
   dp->pre_run = imu_class_pre_run;
   dp->post_run = imu_class_post_run;
   dp->run = imu_class_run;
   dp->abort = imu_abort;
   dp->get_object_at = imu_get_object_at;
   dp->reload_config = imu_reload_config;
   // once initialization done, put into runtime mode
   dp_execute(dp);
   return NULL;
}

