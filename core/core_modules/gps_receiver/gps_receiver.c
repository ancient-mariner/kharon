#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <assert.h>
#include "datap.h"
#include "pinet.h"
#include "logger.h"
#include "kernel.h"
#include "lin_alg.h"
#include "dev_info.h"
#include "sensor_packet.h"
#include "udp_sync.h"
#include "sensor_packet.h"

#include "core_modules/gps_receiver.h"

#include "parsing.c"
#include "receiver_logic.c"

////////////////////////////////////////////////////////////////////////
static void gps_receiver_class_pre_run(
      /* in out */       struct datap_desc *dp
      )
{
   gps_receiver_class_type *gps = (gps_receiver_class_type*) dp->local;
   const char * name = dp->td->obj_name;
   ////////////////////////////////////////
   // setup networking
   // this config data is static so don't include it in reload_config
   // get socket port
   char port_str[STR_LEN];
   FILE *fp = open_config_file_ro2(NULL, NULL, "endpoints", name, NULL);
   if (!fp) {
      log_err(gps->log, "Unable to open endpoint file for '%s'", name);
      goto err;
   }
   if (config_read_string(fp, port_str, STR_LEN) != 0) {
      log_err(gps->log, "Unable to determine listening port for '%s'", name);
      goto err;
   }
   if (fp) {
      fclose(fp);
   }
   //
   uint32_t port = (uint32_t) atoi(port_str);
   if (c_assert(port <= 0x00007fff)) {
      log_err(gps->log, 
            "Configuration error: port size greater than int16 (%d)", port);
      goto err;
   }
   log_info(gps->log, "%s listening on port %d", name, port);
printf("%s listening on port %d\n", name, port);
   if ((gps->sockfd = init_server((int16_t) port)) < 0) {
      log_err(gps->log, "Failed to initialize server for %s", name);
      // treat as fatal error until recovery logic worked out
      goto err;
   }
   return;
err:
   if (fp)
      fclose(fp);
   hard_exit(__FILE__, __LINE__);
}

////////////////////////////////////////////////////////////////////////
// publishes GPS data pulled from network
static void gps_receiver_class_run(struct datap_desc *dp)
{
   struct gps_receiver_class *gps = (struct gps_receiver_class*) dp->local;
   uint8_t inbound[GPS_BLOCK_SIZE];
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
         // read data packet
         if (recv_block(gps->connfd, &inbound, sizeof(inbound)) < 0) {
            log_err(gps->log, "Read error. Breaking connection");
            close(gps->connfd);
            gps->connfd = -1;
            break;
         }
         parse_and_publish(dp, gps, inbound);
      }
   }
}


////////////////////////////////////////////////////////////////////////
static void gps_receiver_class_post_run(struct datap_desc *dp)
{
   struct gps_receiver_class *gps = (struct gps_receiver_class*) dp->local;
   if (gps->connfd >= 0) {
      close(gps->connfd);
      usleep(2000);     // give client a chance to close socket first
   }
   if (gps->sockfd >= 0) {
      close(gps->sockfd);
      gps->sockfd = -1;
   }
   if (gps->logfile) {
      fclose(gps->logfile);
      gps->logfile = NULL;
   }
}


////////////////////////////////////////////////////////////////////////
// induce process termination
// process won't abort if socket is waiting to accept a
//    connection, so shutdown socket
static void gps_receiver_abort(struct datap_desc *dp)
{
   struct gps_receiver_class *gps = (struct gps_receiver_class*) dp->local;
   log_info(gps->log, "aborting");
   if (gps->connfd >= 0) {
      if (shutdown(gps->connfd, SHUT_RDWR) != 0) {
         log_err(gps->log, "Error shutting down connection (%s)", 
               strerror(errno));
      }
      gps->connfd = -1;
   }
   if (gps->sockfd >= 0) {
      if (shutdown(gps->sockfd, SHUT_RDWR) != 0) {
         log_err(gps->log, "Error shutting down socket (%s)", strerror(errno));
      }
      gps->sockfd = -1;
   }
}

////////////////////////////////////////////////////////////////////////
static void * gps_receiver_get_object_at(
      /* in     */ const datap_desc_type *self,
      /* in     */ const uint32_t idx
      )
{
   return &self->void_queue[idx * sizeof(gps_receiver_output_type)];
}


////////////////////////////////////////////////////////////////////////
void * gps_receiver_init(void * gps_setup)
{
   datap_desc_type *dp = dp_create();
   gps_receiver_class_type *gps = calloc(1, sizeof(*gps));
   if (c_assert(gps_setup != NULL)) {
      log_err(gps->log, "Internal error -- init passed NULL value");
      hard_exit(__func__, __LINE__);
   }
   gps_receiver_setup_type *setup = (gps_receiver_setup_type *) gps_setup;
   // setup logging
   gps->log = get_logger(dp->td->obj_name);
   report_thread_id(dp, gps->log);
   set_log_level(gps->log, GPS_RECEIVER_LOG_LEVEL);
   /////////////////////////////////////////////////////////////////////
   strcpy(gps->device_name, setup->device_name);
   if (setup->logging == 1) {
      // open logfile
      char buf[STR_LEN];
      // construct path
      snprintf(buf, STR_LEN, "%s%s", get_log_folder_name(), dp->td->obj_name);
      gps->logfile = fopen(buf, "w");
      if (gps->logfile == NULL) {
         // non-fatal error. we just loose logging
         log_err(gps->log, "Unable to create data logfile for %s (%s)", 
               dp->td->obj_name, buf);
      } else {
         log_info(gps->log, "%s logging data to %s", dp->td->obj_name, buf);
      }
   } else {
      gps->logfile = NULL;
   }
   free(setup);   // allocated on heap and it's no longer needed
   //
   dp->local = gps;
   gps->sockfd = -1;
   gps->connfd = -1;
   /////////////////////////////////////////////
   // allocate dp->void_queue, dp->ts 
   dp->ts = calloc(1, GPS_RECEIVER_QUEUE_LEN * sizeof(*dp->ts));
   dp->element_size = sizeof(gps_receiver_output_type);;
   dp->queue_length = GPS_RECEIVER_QUEUE_LEN;
   dp->void_queue = calloc(1, GPS_RECEIVER_QUEUE_LEN * dp->element_size);
   // TODO consider setting the update_interval to 1 so that an
   //    alert is set every time data is said to be available, but add 
   //    auxiliary logic to control when a signal is sent that data is 
   //    available
//   dp->update_ctr = 0;
//   dp->update_interval = 2;
   //
   dp->pre_run = gps_receiver_class_pre_run;
   dp->post_run = gps_receiver_class_post_run;
   dp->run = gps_receiver_class_run;
   dp->abort = gps_receiver_abort;
   dp->get_object_at = gps_receiver_get_object_at;
   // once initialization done, put into runtime mode
   dp_execute(dp);
   return NULL;
}

