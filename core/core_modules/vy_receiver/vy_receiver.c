#include "pinet.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <errno.h>
#include "datap.h"
#include "lin_alg.h"
#include "logger.h"
#include "dev_info.h"
#include "timekeeper.h"

#include "core_modules/vy_receiver.h"

#include "receiver_logic.c"
#include "sphere.c"

/**
 * Camera acquisition is largely a hack. The sensor code is based
 *    on RaspiVidYUV.c, modified to send raw image data over the
 *    network at 1/4 resolution. acquisition at 1/2 which is
 *    supposed to use 2x2 binning. for YUV422, the color
 *    channels are 1/2 the resolution of the primary channel, so
 *    the primary channel is downsampled to match color channel
 *    resolution. only Y and V channels are sent (V is mostly a
 *    red/green channel). 
 *
 * Timing is effectively broken on the camera, but it is sufficient
 *    for testing purposes. Attemtps were made to synchronize the
 *    cameras using a 'heartbeat' thread, but this did nothing but
 *    make the cameras appear to be in sync. each camera runs
 *    on its own timer, which appears to be set on camera start,
 *    and frames are captured based on that camera, not when
 *    the capture signal is received (the heartbeat does nothing)
 *
 * Reported times should be sync'd between all nodes however
 *
 * Frame timing achieved by running cameras at maximal rate and
 *    taking frame that arrives nearest the desired time. this
 *    puts all frames w/in the frame capture interval, which may
 *    be as good as we can get from RPi cams
 *
 */

////////////////////////////////////////////////////////////////////////
static void vy_class_pre_run(
      /* in out */       struct datap_desc *dp
      )
{
   // allocate publish queues (dp->void_queue, dp->ts)
   dp->ts = calloc(1, VY_QUEUE_LEN * sizeof(*dp->ts));
   dp->queue_length = VY_QUEUE_LEN;
   dp->element_size = sizeof(vy_receiver_output_type);
   dp->void_queue = calloc(1, VY_QUEUE_LEN * dp->element_size);
   //
   struct vy_class *vy = (struct vy_class*) dp->local;
   log_info(vy->log, "in pre_run()");
   /////////////////////////////////////////////////////////////
   // setup networking
   // get socket port
   char port_str[STR_LEN];
   FILE *fp = open_config_file_ro2(NULL, NULL, "endpoints", 
         dp->td->obj_name, NULL);
   if (!fp) {
      log_err(vy->log, "Unable to open endpoint file for '%s'", 
            dp->td->obj_name);
      goto err;
   }
   if (config_read_string(fp, port_str, STR_LEN) != 0) {
      log_err(vy->log, "Unable to determine listening port for '%s'", 
            dp->td->obj_name);
      goto err;
   }
   uint32_t port = (uint32_t) atoi(port_str);
   if (c_assert(port <= 0x00007fff)) {
      log_err(vy->log, 
            "Configuration error: port size greater than int16 (%d)", port);
      goto err;
   }
   log_info(vy->log, "%s listening on port %d", dp->td->obj_name, port);
   if ((vy->sockfd = init_server((int16_t) port)) < 0) {
      log_err(vy->log, "Failed to initialize server for %s", dp->td->obj_name);
      // treat as fatal error until recovery logic worked out
      goto err;
   }
   vy->camera_name = calloc(1, STR_LEN);
   /////////////////////////////////////////////////////////////
   vy->raw_v = malloc(CAM_ROWS * CAM_COLS * sizeof vy->raw_v[0]);
   vy->raw_y = malloc(CAM_ROWS * CAM_COLS * sizeof vy->raw_y[0]);
   vy->img_tmp = malloc(CAM_ROWS * CAM_COLS * sizeof vy->img_tmp[0]);
   /////////////////////////////////////////////////////////////
   uint32_t offsets[NUM_PYRAMID_LEVELS];
   uint32_t tot_pix = 0;
   for (uint32_t lev=0; lev<NUM_PYRAMID_LEVELS; lev++) {
      offsets[lev] = tot_pix;
      image_size_type sz = vy->img_size[lev];
      uint32_t n_pix = (uint32_t) (sz.x * sz.y);
      tot_pix += n_pix;
   }
   for (uint32_t i=0; i<VY_QUEUE_LEN; i++) {
      vy_receiver_output_type *out = (vy_receiver_output_type*) 
            dp_get_object_at(dp, i);
      for (uint32_t lev=0; lev<NUM_PYRAMID_LEVELS; lev++) {
         out->chan_offset[lev] = offsets[lev];
         out->img_size[lev] = vy->img_size[lev];
      }
      out->chans = malloc(tot_pix * sizeof *out->chans);
   }
   //
   return;
err:
   hard_exit(__FILE__, __LINE__);
}


////////////////////////////////////////////////////////////////////////
static void vy_class_run(
      /* in out */      struct datap_desc *dp
      )
{
   //
//   set_acquisition_state(1);
   while ((dp->run_state & DP_STATE_DONE) == 0) {
      // wait for the next connection. there should only be one
      //    connection, but if client aborts (eg, segfault) then
      //    it'll need to reconnect
      wait_next_connection(dp);
      // server is up. keep pulling data until done flag is set
      pull_data(dp);
   }
}


////////////////////////////////////////////////////////////////////////
// process won't abort if socket is waiting to accept a
//    connection, so shutdown socket
static void vy_abort(struct datap_desc *dp)
{
   struct vy_class *vy = (struct vy_class*) dp->local;
   log_info(vy->log, "Abort");
   if (vy->connfd >= 0) {
      if (shutdown(vy->connfd, SHUT_RDWR) != 0) {
         log_err(vy->log, "Error shutting down connection (%s)", 
               strerror(errno));
      }
      vy->connfd = -1;
   }
   if (vy->sockfd >= 0) {
      if (shutdown(vy->sockfd, SHUT_RDWR) != 0) {
         log_err(vy->log, "Error shutting down socket", strerror(errno));
      }
      vy->sockfd = -1;
   }
}


////////////////////////////////////////////////////////////////////////
static void vy_class_post_run(struct datap_desc *dp)
{
   struct vy_class *vy = (struct vy_class*) dp->local;
   log_info(vy->log, "In post run");
   if (vy->connfd >= 0) {
      close(vy->connfd);
      usleep(250);   // give client a chance to close socket first
   }
   if (vy->sockfd >= 0) {
      close(vy->sockfd);
      vy->sockfd = -1;
   }
}


////////////////////////////////////////////////////////////////////////
static void * vy_class_get_object_at(
      /* in     */ const datap_desc_type *dp,
      /* in     */ const uint32_t idx
      )
{  
   return &dp->void_queue[idx * sizeof(vy_receiver_output_type)];
} 

////////////////////////////////////////////////////////////////////////
void * vy_class_init(void *vy_setup)
{
   struct datap_desc *dp = dp_create();
   struct vy_class *vy = calloc(1, sizeof(*vy));
   if (c_assert(vy_setup != NULL)) {
      log_err(vy->log, "Internal error -- vy_class_init passed NULL value");
      hard_exit(__FILE__, __LINE__);
   }
   assert(vy_setup != NULL);     // 2nd assert for scan-build
   vy_setup_type *setup = (vy_setup_type*) vy_setup;
   dp->local = vy;
   vy->sockfd = -1;
   vy->connfd = -1;
   vy->camera_num = setup->camera_num;
   strcpy(vy->device_name, setup->device_name);
   vy->data_folder = NULL;
   vy->log = get_logger(dp->td->obj_name);
   report_thread_id(dp, vy->log);
   set_log_level(vy->log, VY_LOG_LEVEL);
   //
   if (setup->logging == 1) {
      // write output files to subdirectory in main data directory
      // use process name as folder name
      const char *folder = get_log_folder_name();
      const char *name = dp->td->obj_name;
      size_t len = strlen(folder) + strlen(name) + 2;
      vy->data_folder = malloc(len+1);
      sprintf(vy->data_folder, "%s%s", folder, name);
      if (mkdir(vy->data_folder, 0755) != 0) {
         log_err(vy->log, "Failed to create output directory '%s': %s",
               vy->data_folder, strerror(errno));
         // output directory not available. free allocated memory and
         //    don't output data
         free(vy->data_folder);
         vy->data_folder = NULL;
      } else {
         log_info(vy->log, "Created output directory %s", vy->data_folder);
      }
//      // open logfile
//      char buf[STR_LEN];
//      // construct path
//      snprintf(buf, STR_LEN, "%s%s.log", folder, name);
//      vy->logfile = fopen(buf, "w");
//      if (vy->logfile == NULL) {
//         // non-fatal error. we just loose logging
//         log_err(vy->log, "Unable to create data logfile for %s (%s)", 
//               dp->td->obj_name, buf);
//      } else {
//         log_info(vy->log, "%s logging data to %s", dp->td->obj_name, buf);
//         log_info(vy->log, "format: gyro (3), acc (3), mag (3), temp");
//      }
   } else {
      vy->data_folder = NULL;
//      vy->logfile = NULL;
   }
   if (vy_setup != NULL)
      free(vy_setup);   // this was allocated on heap and isn't needed anymore
   // sphere map uses static storage shared between threads so it must
   //    be initialized in the single-threaded init call (pre_run has
   //    all processes running in parallel already)
   image_size_type sz[NUM_PYRAMID_LEVELS] = {
         { .x=CAM_COLS, .y=CAM_ROWS },
         { .x=CAM_COLS/2, .y=CAM_ROWS/2 }
   };
   degree_type h_fov = { .degrees = VY_FOV_HORIZ_DEG };
   degree_type v_fov = { .degrees = VY_FOV_VERT_DEG };
   for (uint32_t lev=0; lev<NUM_PYRAMID_LEVELS; lev++) {
      vy->img_size[lev] = sz[lev];
      vy->sphere_map[lev] = get_projection_map(sz[lev], h_fov, v_fov);
   }
   vy->fov_horiz.degrees = VY_FOV_HORIZ_DEG;
   vy->fov_vert.degrees = VY_FOV_VERT_DEG;
   //
   dp->pre_run = vy_class_pre_run;
   dp->post_run = vy_class_post_run;
   dp->run = vy_class_run;
   dp->abort = vy_abort;
   dp->get_object_at = vy_class_get_object_at;
   // once initialization done, put into runtime mode
   dp_execute(dp);
   return NULL;
}

