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
#include <string.h>
#include <signal.h>
#include <stdlib.h>
#include <stdint.h>
#include <pthread.h>
#include <sys/socket.h>
#include <unistd.h>
#include <assert.h>
#include "kernel.h"
#include "datap.h"
#include "pinet.h"
#include "postmaster.h"
#include "udp_sync.h"
#include "logger.h"

pthread_barrier_t barrier_g;
pthread_mutex_t global_mutex_g;

// flag to prevent signal_exit from being called multiple times
static int32_t shutdown_flag_ = 0;

// triggers a shutdown of the kernel. it's not necessarily graceful
//    and it's prone to race conditions. it should be hardened -- TODO
void signal_exit(int x)
{
printf("------- signal_exit\n"); fflush(stdout);
   if (shutdown_flag_++ > 0)
      return;
printf("*** signal_exit\n"); fflush(stdout);
   if (x == SIGALRM)
      printf("\tSIGALRM\n");
   postmaster_quit(0);
   if (processor_list_ != NULL) {
      // set each thread's quit flag
      for (uint32_t i=0; i<num_threads_g; i++)
         dp_quit(thread_table_g[i].dp);
//   }
//   shutdown_endpoints();
//   //
//   if (processor_list_ != NULL) {
      // wait for threads to finish what they're doing
      // duration should be >> worst case scenario, to avoid deadlock
      // TODO have each thread report that it's done processing.
      //    wait for all threads to report in before continuing, or
      //    until e.g., 5 sec has elapsed
      usleep(1500000);
      // execute abort for thread. thread should have gone through
      //    post_run by now, or is still sleeping
      for (uint32_t i=0; i<num_threads_g; i++)
         dp_abort(thread_table_g[i].dp);
      usleep(2500);
      // in case thread is still sleeping, wake it up now
      for (uint32_t i=0; i<num_threads_g; i++)
         dp_wake(thread_table_g[i].dp);
   }
}

static char env_[STR_LEN] = { 0 };

const char * get_environment(void)    { return env_;    }

void set_environment_string(
      /* in     */ const char *env
      )
{
   strcpy(env_, env);
}


int32_t module_onoff(
      /* in     */ const char *module_name,
      /* in     */ const int32_t on_off
      )
{
   if (processor_list_ != NULL) {
      // set each thread's quit flag
      for (uint32_t i=0; i<num_threads_g; i++) {
         datap_desc_type *dp = thread_table_g[i].dp;
         if (strcmp(dp->td->obj_name, module_name) == 0) {
            if (on_off == 0) {
               dp->run_state |= DP_STATE_PAUSE;
            } else {
               dp->run_state &= ~DP_STATE_PAUSE;
            }
            return 0;
         }
      }
   }
   return -1;
}

#if 0

void set_acquisition_state(uint32_t on_off)
{
   // if acquisition is off and is being turned on, send a time sync
   //    packet. during initial startup it may not have been sent yet
   if (!MESSAGE_BOARD.acquiring && (on_off != 0)) {
      usleep(50000);
      send_sync_packet(UDP_SYNC_PACKET_TIME);
   }
   MESSAGE_BOARD.acquiring = on_off?1:0;
   if (MESSAGE_BOARD.acquiring) {
      send_sync_packet(UDP_SYNC_PACKET_START_ACQ);
   } else {
      send_sync_packet(UDP_SYNC_PACKET_STOP_ACQ);
   }
}

////////////////////////////////////////////////////////////////////////
// network services
// TODO consider deleting -- this code doesn't appear to be in use
//    services are managed by hard-coded ports in dev/ tree
//
// Each service is bound to a port locally
// When an endpoint request comes in (via the postmaster), the postmaster
//    claims and enpdpoint and returns the port number to the calling process.
//    That port will be marked as claimed until the service releases it,
//    which should happen if the client disconnects.
// Note: clients should have a timeout setting on their accept() calls,
//    and periodically release their ports for future use. This is to
//    account for situation where client binds port but fails to
//    connect -- the postmaster will have marked the port as being in use
//    and so won't assign it again, while the service won't have a
//    connection

static service_info_type service_ports_[MAX_NETWORK_SERVICES];
static uint32_t num_assigned_services_ = 0;

// register an available network service. returns port number that will
//    be assigned to this service instance
int16_t register_service(
      /* in     */ const uint32_t service_id
      )
{
   int16_t port_num = 0;
   if (num_assigned_services_ == 0) {
      memset(service_ports_, 0, MAX_NETWORK_SERVICES * sizeof *service_ports_);
   }
   if (num_assigned_services_ < MAX_NETWORK_SERVICES) {
      port_num = (int16_t) (get_postmaster_port() + 1 +
            (int16_t) num_assigned_services_);
printf("Service %d has port %d\n", service_id, port_num);
printf("postmaster port: %d\n", get_postmaster_port());
printf("assigned port: %d\n", port_num);
printf("# assigned: %d\n", num_assigned_services_);
      service_ports_[num_assigned_services_].service_id = service_id;
      service_ports_[num_assigned_services_].port_num = port_num;
      num_assigned_services_++;
   } else {
      log_err(get_kernel_log(),
            "Too many services registered -- cannot create new one. Max=%d",
            MAX_NETWORK_SERVICES);
   }
   return port_num;
}


// mark a port as being in-use
int16_t lookup_endpoint(
      /* in     */ const uint32_t service_id
      )
{
   printf("Looking up service %d (0x%08x)\n", service_id, service_id);
   for (uint32_t i=0; i<num_assigned_services_; i++) {
      service_info_type *info = &service_ports_[i];
      printf("Comparing to %d\n", info->service_id);
      if (info->service_id == service_id) {
         if (info->in_use == 0) {
            printf("Not in use -- returning %d\n", info->port_num);
            return info->port_num;
         }
      }
   }
   return 0;
}

////////////////////////////////////////////////////////////////////////
// TODO delete this code -- it appears to be not in use

// When a client connects to a service, the service needs to mark the
//    port as being in use, so subsequent clients can be sent to
//    a different port. When a connection is broken, the port needs
//    to be released and marked as being available again

// mark a port as being in-use
void claim_endpoint(
      /* in     */ const int16_t port_num
      )
{
   log_info(get_kernel_log(), "Claiming endpoint for port %d", port_num);
   for (uint32_t i=0; i<num_assigned_services_; i++) {
      service_info_type *info = &service_ports_[i];
      if (info->port_num == port_num) {
         info->in_use = 1;
         return;
      }
   }
   log_info(get_kernel_log(),
         "Failed to claim endpoint -- port %d not available", port_num);
}

// mark port as available so new service can use it
void release_endpoint(
      /* in     */ const int16_t port_num
      )
{
   log_info(get_kernel_log(), "Releasing endpoint for port %d", port_num);
   for (uint32_t i=0; i<num_assigned_services_; i++) {
      service_info_type *info = &service_ports_[i];
      if (info->port_num == port_num) {
         info->in_use = 0;
         return;
      }
   }
   log_info(get_kernel_log(),
         "Failed to release endpoint -- port %d not bound", port_num);
}
#endif   // 0

