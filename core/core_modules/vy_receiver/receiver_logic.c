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
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <errno.h>
#include "datap.h"
#include "lin_alg.h"
#include "logger.h"
#include "dev_info.h"
#include "timekeeper.h"
#include "kernel.h"
#include "blur.h"

#include "core_modules/vy_receiver.h"

static void wait_next_connection(
      /* in out */      struct datap_desc *dp
      )
{
   struct vy_class *vy = (struct vy_class*) dp->local;
   while ((dp->run_state & DP_STATE_DONE) == 0) {
      log_info(vy->log, "VY waiting for connection");
      if ((vy->connfd = wait_for_connection(vy->sockfd,
            dp->td->obj_name)) < 0) {
         if (vy->connfd == -2) {
            continue;
         }
         // failed connection
         log_err(vy->log, "%s failed waiting for connection. Waiting to "
               "try again", dp->td->obj_name);
         sleep(5);
         continue;
      }
      log_info(vy->log, "VY reading stream ID");
      // read stream ID
      uint32_t magic;
      uint32_t response = HANDSHAKE_OK;
      if (recv_block(vy->connfd, &magic, sizeof(magic)) < 0) {
         // failed read
         log_err(vy->log, "%s failed to read handshake constant. "
               "Breaking connection.", dp->td->obj_name);
         close(vy->connfd);
         vy->connfd = -1;
         continue;
      }
      magic = htonl(magic);
      if (magic != VY_STREAM_ID) {
         log_err(vy->log, "%s received inappropriate magic number",
               dp->td->obj_name);
         log_err(vy->log, "Received 0x%08x, expected 0x%08x", magic,
               VY_STREAM_ID);
         response = HANDSHAKE_ERROR;
      }
      response = htonl(response);
      log_info(vy->log, "Sending handshake response");
      if (send_block(vy->connfd, &response, sizeof(response)) < 0) {
         log_err(vy->log, "%s failed to send handshake response",
               dp->td->obj_name);
         log_err(vy->log, "Breaking connection");
         close(vy->connfd);
         vy->connfd = -1;
         continue;
      }
      if (response != htonl(HANDSHAKE_OK)) {
         log_err(vy->log, "Breaking connection");
         close(vy->connfd);
         vy->connfd = -1;
         continue;
      }
      // successfully connected -- break from connect loop
      //    and start pulling data
      log_info(vy->log, "Successfully established connection");
      break;
   }
}


static int32_t write_pgm_file(
      /* in     */ const uint8_t * restrict v_chan,
      /* in     */ const uint8_t * restrict y_chan,
      /* in     */ const char * name,
      /* in     */ const uint32_t rows,
      /* in     */ const uint32_t cols
      )
{
   int32_t rc = -1;
//   log_info(vy->log, "Writing %s", path);
   FILE *fp = fopen(name, "w");
   if (!fp) {
      fprintf(stderr, "Error opening '%s' for writing\n", name);
      goto err;
   }
   // push stacked v and y channels to file
   fprintf(fp, "P5\n%d %d\n255\n", cols, 2*rows);
   size_t buf_sz = cols * rows;
   size_t n = fwrite(v_chan, buf_sz, 1, fp);
   if (n <= 0) {
      fprintf(stderr, "Error writing pgm file (v) '%s'\n", name);
      goto err;
   }
   n = fwrite(y_chan, buf_sz, 1, fp);
   if (n <= 0) {
      fprintf(stderr, "Error writing pgm file (y) '%s'\n", name);
      goto err;
   }
   rc = 0;
err:
   if (fp)
      fclose(fp);
   return rc;
}


//static int save_pgm_file(const char *dir, const char* filename,
//      struct datap_desc *uvy_dp, int32_t idx)
static int32_t log_to_pgm_file(
      /* in     */ const struct datap_desc *dp,
      /* in     */ const uint32_t idx
      )
{
   int32_t rc = -1;
   // filename is actual acquisition time. log stores equivalent of
   //    frame number to associate with this time
   //const double t = out->frame_request_time;
   const double t = dp->ts[idx];
   //
   struct vy_class *vy = (struct vy_class*) dp->local;
   char path[STR_LEN];
   snprintf(path, STR_LEN, "%s/%.3f.pgm", vy->data_folder, t);
   if (write_pgm_file(vy->raw_v, vy->raw_y, path,
            CAM_ROWS, CAM_COLS) != 0) {
      fprintf(stderr, "Error writing VY output %s\n", path);
      goto err;
   }
   rc = 0;
err:
   return rc;
}

static void pull_data(
      /* in out */      struct datap_desc *dp
      )
{
   vy_class_type *vy = (vy_class_type *) dp->local;
   // packet data
   double frame_request;
   double frame_received;
   uint32_t pkt_type;
   struct sensor_packet_header header;
   // loop until done, or connection broken
   while ((dp->run_state & DP_STATE_DONE) == 0) {
      // read packet header
      log_debug(vy->log, "Read packet header");
      if (recv_block(vy->connfd, &header, sizeof(header)) < 0) {
         log_err(vy->log, "%s failed to read vy packet header. "
               "Breaking connection.", dp->td->obj_name);
         close(vy->connfd);
         vy->connfd = -1;
         break;
      }
      //unpack_sensor_header(&header, &pkt_type, &remote_time);
      log_debug(vy->log, "Unpack packet header");
      unpack_sensor_header2(&header, &pkt_type, &frame_request,
            &frame_received);
      if (pkt_type != VY_PACKET_TYPE) {
         log_err(vy->log, "Packet type error\nExpected 0x%08x, "
               "received 0x%08x", VY_PACKET_TYPE, pkt_type);
         hard_exit(__func__, __LINE__);
      }
      // TODO FIXME find out why empty string is not null
      if (header.log_data[0] != 0) {
      //if (header.log_data[0] != 0) {
         header.log_data[SENSOR_PACKET_LOG_DATA-1] = 0;
         log_info(vy->log, "%s remote log: '%s'", dp->td->obj_name,
               header.log_data);
      }
      // store timestamp (provided in packet header)
      uint32_t idx = (uint32_t) (dp->elements_produced % dp->queue_length);
      vy_receiver_output_type *out = (vy_receiver_output_type*)
            dp_get_object_at(dp, idx);
      out->frame_request_time = frame_request;
      dp->ts[idx] = frame_received;
      log_info(vy->log, "Frame request %.4f, received %.4f",
               frame_request, frame_received);
      // decode packet
//printf("Receiving packet from %.3f\n", remote_time);
      // check dimension
      image_size_type dim;
      dim.rows = htons((uint16_t) header.custom_16[0]);
      dim.cols = htons((uint16_t) header.custom_16[1]);
      if ((dim.rows != CAM_ROWS) || (dim.cols != CAM_COLS)) {
         log_err(vy->log, "Frame size mismatch in %s", dp->td->obj_name);
         log_err(vy->log, "Expecting %dx%d, received %d,%d",
               CAM_COLS, CAM_ROWS, dim.cols, dim.rows);
         close(vy->connfd);
         vy->connfd = -1;
         // this is a configuration error that cannot be fixed and
         //    will otherwise repeat indefinitely
         hard_exit("vy_receiver::vy_class_run", 1);
      }
//printf("receiving frame: %d bytes\n", vy_stream_len);
      //////////////////////////////////
      // pull image frame from network and copy to output buffer
      // v channel
      //if (recv_block(vy->connfd, out->v_chan, CAM_N_PIX) < 0) {
      if (recv_block(vy->connfd, vy->raw_v, CAM_N_PIX) < 0) {
         log_err(vy->log, "\n%s read error, v-chan. Breaking connection",
               dp->td->obj_name);
         goto end;
      }
      // y channel
      if (recv_block(vy->connfd, vy->raw_y, CAM_N_PIX) < 0) {
         log_err(vy->log, "\n%s read error, y-chan. Breaking connection",
               dp->td->obj_name);
         goto end;
      }
      if ((dp->run_state & DP_STATE_PAUSE) != 0) {
         continue;
      }
      // copy to output, splitting into image pyramid
      for (uint32_t lev=0; lev<NUM_PYRAMID_LEVELS; lev++) {
         image_size_type sz = vy->img_size[lev];
         uint32_t n_pix = (uint32_t) (sz.x * sz.y);
         // copy 'raw' data to output
         const uint32_t offset = out->chan_offset[lev];
         vy_pixel_type *chan = &out->chans[offset];
         for (uint32_t i=0; i<n_pix; i++) {
            vy_pixel_type *pix = &chan[i];
            // EXPERIMENT
            // V channel signal gets washed out w/ NIR -- magnify it
            pix->v = (uint8_t) (128 + 2 * (vy->raw_v[i] - 128));
            //pix->v = vy->raw_v[i];
            pix->y = vy->raw_y[i];
//            // testing -- visualize data change
//            // alter 'raw' so content output to log
//            vy->raw_v[i] = pix->v;
         }
         // if data logging enabled, write pgm file of (raw) image
         if ((lev == 0) && (vy->data_folder != NULL)) {
            log_to_pgm_file(dp, idx);
         }
         // blur and downsample to prepare for next round
         uint32_t src_idx = 0;
         uint32_t dest_idx = 0;
         uint8_t *restrict raw_v = vy->raw_v;
         uint8_t *restrict raw_y = vy->raw_y;
         unsigned int *restrict img_tmp = vy->img_tmp;
         if (lev < (NUM_PYRAMID_LEVELS - 1)) {
            // blur each channel
            blur_image_r1(raw_v, img_tmp, raw_v, sz);
            blur_image_r1(raw_y, img_tmp, raw_y, sz);
            // downsample buffers. work done in-place, as except for first
            //    copy, src_index > dest_index
            for (uint32_t y=0; y<sz.y; y+=2) {
               for (uint32_t x=0; x<sz.x; x+=2) {
                  raw_v[dest_idx] = raw_v[src_idx];
                  raw_y[dest_idx] = raw_y[src_idx];
                  src_idx += 2;
                  dest_idx++;
               }
               src_idx += sz.x;
            }
         }
      }
      //////////////////////////////////
      // all done. let others know
      dp->elements_produced++;
//printf("Posting frame at %.6f. Remote time: %.6f\n", now(), remote_time);
      log_info(vy->log, "Signaling data available (%ld)",
            dp->elements_produced);
      dp_signal_data_available(dp);
   }
end:
   if (vy->connfd >= 0) {
      close(vy->connfd);
      vy->connfd = -1;
   }
}


//void module_config_load_dev_to_ship(
//      /* in     */ const char *module_name,
//      /*    out */       matrix_type *dev2ship
//      )
//{
//   FILE *fp = open_config_file_ro(NULL, "sensors", NULL, "cam_dev2ship");
//   if (!fp) {
//      log_err("Unable to open cam-2-ship file for '%s'", module_name);
//      goto err;
//   }
//   if (config_read_matrix(fp, dev2ship) != 0) {
//      log_err("Unable to read alignment dev2ship for '%s'", module_name);
//      goto err;
//   }
//err:
//   if (fp)
//      fclose(fp);
//}

void module_config_load_cam_to_ship(
      /* in out */       log_info_type *log,
      /* in     */ const char *device_name,
      /* in     */ const char *module_name,
      /*    out */       matrix_type *dev2ship
      )
{
   FILE *fp = open_config_file_ro2(get_environment(), device_name, "sensors",
         NULL, "cam_dev2ship");
   if (!fp) {
      log_err(log, "Unable to open cam-2-ship file for '%s'", module_name);
      goto err;
   }
   if (config_read_matrix(fp, dev2ship) != 0) {
      log_err(log, "Unable to read alignment dev2ship for '%s'",
            module_name);
      goto err;
   }
err:
   if (fp)
      fclose(fp);
}

