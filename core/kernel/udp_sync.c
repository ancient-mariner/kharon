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
#include <assert.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <errno.h>

#include "kernel.h"
#include "datap.h"
#include "pinet.h"
#include "timekeeper.h"
#include "udp_sync.h"

// sends broadcast UDP sync packets to synchronize clocks between nodes
//    and start/stop acquisition

// only one udp sync class permitted
static struct udp_sync_class *s_udp = NULL;

// keep track of thread ID in order to pass SIGUSR2, to wake thread
//    from slumber
static pthread_t s_tid;

// mutex to protect packet-sending code, as this is accessible from
//    other threads
static pthread_mutex_t s_sync_mutex;

// flag set on abort. used by sleep code to determine if sleep
//    interruption is due termination
static int32_t s_abort_flag = 0;

// sigusr2 handler -- this is a no-op. the role of sigusr2 is to
//    wake the thread if it's sleeping
static void sigusr2_callback(int sig)
{
   if (sig != SIGUSR2) {
      fprintf(stderr, "usr_sync unexpectedly received signal %d\n", sig);
      hard_exit(__FILE__, __LINE__);
   }
}

static void udp_sync_class_abort(struct datap_desc *unused)
{
   (void) unused;
printf("UDP sync -- abort\n");
   s_abort_flag = 1;
   pthread_kill(s_tid, SIGUSR2);
}


static void udp_sync_class_pre_run(struct datap_desc *dp)
{
printf("UDP pre-run\n");
   // allocate publish queues (dp->void_queue, dp->ts)
   dp->ts = malloc(UDP_SYNC_QUEUE_LEN * sizeof(*dp->ts));
   dp->void_queue = malloc(UDP_SYNC_QUEUE_LEN * UDP_SYNC_ELEMENT_SIZE);
   // set element size and queue length
   dp->element_size = UDP_SYNC_ELEMENT_SIZE;
   dp->queue_length = UDP_SYNC_QUEUE_LEN;
   //
   struct udp_sync_class *udp = (struct udp_sync_class*) dp->local;
   if (s_udp != NULL) {
      fprintf(stderr, "Multiple udp sync classes created. This is "
            "not supported\n");
      hard_exit(__FILE__, __LINE__);
   }
   s_udp = udp;
   s_tid = pthread_self();
   ////////////////
   // create socket
   if ((udp->sockfd = socket(AF_INET, SOCK_DGRAM, 0)) == -1) {
      perror("Failed to init udp socket");
      goto err;
   }
   ////////////////////////
   // prepare for broadcast
   memset((char *) &udp->sock_addr, 0, sizeof(udp->sock_addr));
   udp->sock_addr.sin_family = AF_INET;
   udp->sock_addr.sin_port = htons(UDP_SYNC_PORT);
   if (inet_aton(UDP_SYNC_DOMAIN, &udp->sock_addr.sin_addr) == 0) {
      perror("inet_aton error");
      goto err;
   }
   int32_t broadcast = 1;
   if (setsockopt(udp->sockfd, SOL_SOCKET, SO_BROADCAST,
            &broadcast, sizeof(broadcast)) != 0) {
      perror("setsockopt broadcast");
      goto err;
   }
   // set up thread to catch sigusr2
   signal(SIGUSR2, sigusr2_callback);
   // done
   return;
err:
   hard_exit(__FILE__, __LINE__);
}

static void udp_sync_class_post_run(struct datap_desc *dp)
{
printf("%s in post run\n", dp->td->obj_name);
   send_sync_packet(UDP_SYNC_PACKET_EXIT);
   struct udp_sync_class *udp = (struct udp_sync_class*) dp->local;
   if (udp->sockfd >= 0) {
      close(udp->sockfd);
      udp->sockfd = -1;
   }
   pthread_mutex_destroy(&s_sync_mutex);
}

// questions:
// 1) viable to switch cam resolution on the fly
// 2) performance penalty for doing so
// 3) if acq time is 3+ frames, min latency for acq is 2 frames + ??
// 4) is possible to get exposure time for frame (& thus measure acq latency)


void send_sync_packet(uint32_t type)
{
   if (!s_udp || (s_udp->sockfd < 0)) {
      printf("Sync packet send attempt before initialization\n");
      return;
   }
//printf("Sending sync packet (%d)\n", type);
   struct udp_sync_packet pack;
   memset(&pack, 0, sizeof(pack));
   assert((type & 0xffff0000) == 0);
   pack.packet_type = (uint16_t) type;
   //
   pthread_mutex_lock(&s_sync_mutex);
   // set packet timestamp
   snprintf(pack.timestamp, TIMESTAMP_STR_LEN, "%.6f", system_now());
   // send packet
   if (sendto(s_udp->sockfd, &pack, sizeof(pack), 0,
               &s_udp->sock_addr, sizeof(s_udp->sock_addr))==-1) {
      perror("UDP sync broadcast error");
      hard_exit(__FILE__, __LINE__);
   }
   pthread_mutex_unlock(&s_sync_mutex);
}

// returns 1 if sleep terminated unexpectedly and 0 otherwise
static int32_t sync_sleep(double seconds)
{
   increment_timef(&s_udp->bcast_time, seconds);
   int32_t rc;
   while ((rc = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME,
               &s_udp->bcast_time, NULL)) != 0) {
      if (rc != EINTR) {
         perror("UDP frame sync sleep problem");
         hard_exit(__FILE__, __LINE__);
      } else if (s_abort_flag) {
printf("nap interruption\n");
         return -1;
      }
   }
   return 0;
}

#define RUNNING_INTERVAL   20.0  // interval between time syncs
#define QUIET_INTERVAL     (2 * CAMERA_FRAME_INTERVAL)

static void udp_sync_class_run(struct datap_desc *dp)
{
printf("UDP run\n");
   struct udp_sync_class *udp = (struct udp_sync_class*) dp->local;
   clock_gettime(CLOCK_MONOTONIC, &udp->bcast_time);
   /////////////////////////////////////////////////////////////////////
   // approach:
   //    send time sync packet every X seconds
   //    before doing so, send packet to pause network traffic so
   //       packet reaches each node at ~ same time
   while ((dp->run_state & DP_STATE_DONE) == 0) {
      // send time packet
      // also send continue packet. if acquisition on, send that flag
      //    too as camera process may have started after 'on' was
      //    send and so won't know that it's supposed to be recording
      uint32_t type = UDP_SYNC_PACKET_CONTINUE | UDP_SYNC_PACKET_TIME;
      if (MESSAGE_BOARD.acquiring)
         type |= UDP_SYNC_PACKET_START_ACQ;
printf("Send sync\n");
      send_sync_packet(type);
      //////
      // run for X seconds, then send pause packet
      if (sync_sleep(RUNNING_INTERVAL) < 0)
         break;
      send_sync_packet(UDP_SYNC_PACKET_PAUSE);
      // wait for a second to make sure network is quiet and then send
      //    time sync packet
      if (sync_sleep(QUIET_INTERVAL) < 0)
         break;
      // time and continue packet sent at head of loop
   }
printf("UDP sync sending exit packet\n");
   send_sync_packet(UDP_SYNC_PACKET_EXIT);
}

void * udp_sync_class_init(void * unused)
{
printf("UDP init\n");
   (void) unused;
   struct datap_desc *dp = dp_create();
   struct udp_sync_class *udp = malloc(sizeof(*udp));
   dp->local = udp;
   udp->sockfd = -1;
   memset((char*) &udp->sock_addr, 0, sizeof(udp->sock_addr));
   //
   pthread_mutex_init(&s_sync_mutex, NULL);
   // udp->bcast_time is set in _run
   //
   dp->pre_run = udp_sync_class_pre_run;
   dp->post_run = udp_sync_class_post_run;
   dp->run = udp_sync_class_run;
   dp->abort = udp_sync_class_abort;
   // once initialization done, put into runtime mode
   dp_execute(dp);
   return NULL;
}

