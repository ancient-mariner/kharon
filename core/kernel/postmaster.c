#include "postmaster.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <assert.h>
#include <signal.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include "kernel.h"
#include "logger.h"
#include "mem.h"
#include "timekeeper.h"
#include "udp_sync.h"
#include "dev_info.h"
#include "routing/driver.h"

static enum postmaster_state postmaster_ = UNITIALIZED;

static struct memory_pool * pool_ = NULL;

static int sockfd_ = -1;

static log_info_type *log_ = NULL;

enum postmaster_state get_postmaster_state(void)
{
   return postmaster_;
}

void postmaster_quit(
      /* in     */ const int unused
      )
{
   (void) unused;
log_info(log_, "Postmaster quit");
printf("Postmaster quit\n");
   postmaster_ = DONE;
   if (sockfd_ >= 0) {
      printf("Postmaster shutdown socket\n");
      shutdown(sockfd_, SHUT_RDWR);
      sockfd_ = -1;
   }
}


////////////////////////////////////////////////////////////////////////
// helper procedures

static int16_t port_num_ = 0;

static int setup_networking(void)
{
   char buf[STR_LEN];
   FILE *fp = open_config_file_ro2(NULL, NULL, "endpoints", "postmaster", NULL);
   if (!fp) {
      log_err(log_, "Failed to open config 'dev/<host>/postmaster'");
      goto err;
   }
   if (config_read_string(fp, buf, STR_LEN) != 0) {
      log_err(log_, "Failed to determine network port for postmaster");
      goto err;
   }
   port_num_ = (int16_t) atoi(buf);
printf("postmaster on localhost port %d\n", port_num_);
   //
   if ((sockfd_ = init_server_backlog(port_num_, 0)) < 0) {
      log_err(log_, "Failed to initialize server for postmaster");
      goto err;
   }
//// protocol
//   if (check_protocol_version(sockfd_, COMMUNICATION_PROTOCOL_MAJOR, COMMUNICATION_PROTOCOL_MINOR) < 0) {
//      log_err(log_, "Failed to initialize server for postmaster (protocol)");
//      goto err;
//   }
err:
   if (fp)
      fclose(fp);
   return sockfd_;
}

int16_t get_postmaster_port(void)
{
   return port_num_;
}


// TODO investigate and document -- data field looks to use cached
//    memory. document how data is used and freed, and if it has to
//    be cached
static int read_request(
      /* in     */ const int connfd, 
      /*    out */       struct pm_request *req, 
      /*    out */       uint8_t **data
      )
{
printf("Reading request\n");
   // receive each field from pm_request in the order that it's defined
   uint32_t val;
   req->request_type = 0;
   // read fields one-by-one
   if (recv_block(connfd, &val, sizeof(val)) < 0) {
      goto err;
   }
   req->request_type = htonl(val);
   //
   if (recv_block(connfd, &val, sizeof(val)) < 0) {
      goto err;
   }
   req->header_bytes = htonl(val);
   //
   if (recv_block(connfd, &val, sizeof(val)) < 0) {
      goto err;
   }
   req->custom_0 = (int32_t) htonl(val);
   //
   if (recv_block(connfd, &val, sizeof(val)) < 0) {
      goto err;
   }
   req->custom_1 = (int32_t) htonl(val);
   //
   if (recv_block(connfd, &val, sizeof(val)) < 0) {
      goto err;
   }
   req->custom_2 = (int32_t) htonl(val);
   //
   if (req->header_bytes > 0) {
      *data = cache_malloc(pool_, req->header_bytes);
      if (recv_block(connfd, *data, req->header_bytes) != 
            (int32_t) req->header_bytes) {
         log_err(log_, "Problem reading request data (%d bytes)",
               req->header_bytes);
         goto err;
      }
   }
   return 0;
err:
   log_err(log_, "Error reading PM request packet (reported type is %d)",
         req->request_type);
   return -1;
}

static int send_response(
      /* in     */ const int connfd,
      /* in     */ const struct pm_request *req,
      /* in     */ const struct pm_response *resp, 
      /* in     */ const void * data
      )
{
   // send each field from pm_response in the order that it's listed
   uint32_t val;
   uint32_t T_BUF_LEN = 32u;
   char str[T_BUF_LEN];
   val = htonl(resp->request_type);
   if (send_block(connfd, &val, sizeof(val)) < 0) {
      goto err;
   }
   //
   val = htonl(resp->response_bytes);
   if (send_block(connfd, &val, sizeof(val)) < 0) {
      goto err;
   }
   //
   memset(str, 0, T_BUF_LEN);
   snprintf(str, T_BUF_LEN, "%.20e", now());
   if (send_block(connfd, str, T_BUF_LEN) < 0) {
      goto err;
   }
   //
   val = htonl((uint32_t) resp->custom_0);
   if (send_block(connfd, &val, sizeof(val)) < 0) {
      goto err;
   }
   //
   val = htonl((uint32_t) resp->custom_1);
   if (send_block(connfd, &val, sizeof(val)) < 0) {
      goto err;
   }
   //
   val = htonl((uint32_t) resp->custom_2);
   if (send_block(connfd, &val, sizeof(val)) < 0) {
      goto err;
   }
   //
   if (resp->response_bytes > 0) {
      if (send_block(connfd, data, resp->response_bytes) != 
            (int32_t) resp->response_bytes) {
         log_err(log_, "Problem sending response data (%d bytes)",
               resp->response_bytes);
         goto err;
      }
   }
   return 0;
err:
   log_err(log_, "Error sending PM response packet (type = %d)",
         req->request_type);
   return -1;
}

static void forward_request(
      /* in     */ const struct pm_request *req, 
      /* in     */ const uint8_t *req_data,
      /* in out */       struct pm_response *resp, 
      /* in out */       uint8_t **content
      )
{
   (void) req;
   (void) resp;
   // assume success
   resp->request_type = req->request_type;
   *content = NULL;
printf("Request type: %d\n", req->request_type);
   switch (req->request_type) {
      case PM_CMD_NULL:
printf("CMD NULL\n");
         send_sync_packet(UDP_SYNC_PACKET_TIME);
         break;
      case PM_CMD_ANNOTATION:
printf("CMD ANN\n");
         log_info(log_, "Annotation '%s'", (const char*) req_data);
         break;
      case PM_CMD_SHUTDOWN:
printf("CMD SHUTDOWN\n");
         log_info(log_, "Shutdown request");
         signal_exit(0);
         break;
      case PM_CMD_AUTOPILOT_ON:
printf("CMD OTTO ON\n");
         log_info(log_, "Autopilot-on request");
         set_autotracking(1);
         break;
      case PM_CMD_AUTOPILOT_OFF:
printf("CMD OTTO OFF\n");
         log_info(log_, "Autopilot-off request");
         set_autotracking(0);
         break;
      case PM_CMD_MODULE_RESUME:
printf("CMD MODULE ON\n");
         log_info(log_, "Module-on request");
         if (module_onoff((const char*) req_data, 1) != 0) {
            resp->request_type = PM_CMD_NULL;
         }
         break;
      case PM_CMD_MODULE_PAUSE:
printf("CMD MODULE OFF\n");
         log_info(log_, "Module-off request");
         if (module_onoff((const char*) req_data, 0) != 0) {
            resp->request_type = PM_CMD_NULL;
         }
         break;
      case PM_CMD_SET_HEADING:
printf("CMD SET HEADING\n");
         log_info(log_, "Set heading request");
         if (req->custom_0 < 0) {
            resp->request_type = PM_CMD_NULL;
         } else {
            set_autopilot_heading((uint32_t) req->custom_0);
         }
         break;
      case PM_CMD_SET_DESTINATION:
printf("CMD OTTO SET_DESTINATION\n");
         {
            world_coordinate_type dest;
            dest.x_deg = (double) req->custom_0 * BAM32_TO_DEG;
            if (dest.x_deg <= -180.0) {
               dest.x_deg += 360.0;
            } else if (dest.x_deg > 180.0) {
               dest.x_deg -= 360.0;
            }
            dest.y_deg = (double) req->custom_1 * BAM32_TO_DEG;
            meter_type rad = { .meters = (double) req->custom_2 };
            log_info(log_, "Set destination request to %.4f,%.4f, r=%.1f",
                  dest.x_deg, dest.y_deg, (double) rad.meters);
            set_destination(dest, rad);
         }
         break;
//      case PM_CMD_START_RECORDING:
//         log_info(log_, "Start recording (%f)", now());
//         set_acquisition_state(1);
//         break;
//      case PM_CMD_STOP_RECORDING:
//         log_info(log_, "Stop recording (%f)", now());
//         set_acquisition_state(0);
//         break;
      default:
         log_err(log_, "Postmaster received unrecognized command '%d'\n",
               req->request_type);
         resp->request_type = PM_CMD_NULL;
         break;
   };
}

static void handle_messages(void)
{
   int connfd = -1;;
   struct pm_request req;
   struct pm_response resp;
   memset(&resp, 0, sizeof(resp));
   uint8_t *req_data, *resp_data;
   while (postmaster_ == RUNNING) {
      if ((connfd = wait_for_connection(sockfd_, "postmaster")) < 0) {
         perror("Connection aborted during postmaster accept");
         continue;
      }
      // read request packet
      req_data = NULL;
      if (read_request(connfd, &req, &req_data) < 0) {
         log_err(log_, "Postmaster failed reading request packet");
         close(connfd);
         continue;
      }
      // handle request
      resp_data = NULL;
      memset(&resp, 0, sizeof(resp));
      forward_request(&req, req_data, &resp, &resp_data);
      if (req_data) {
         cache_free(pool_, req_data);
      }
      // send response
      // don't worry about error code, as we're not going to do anything
      //    on a send failure other than close the connection, and it's
      //    going to be closed anyway
      send_response(connfd, &req, &resp, resp_data);
      if (resp_data) {
         cache_free(pool_, resp_data);
      }
      //
      // close connection
      close(connfd);
   };
   log_info(log_, "Leaving postmaster message handler");
   if (connfd >= 0) {
      close(connfd);
   }
}

////////////////////////////////////////////////////////////////////////
// thread entry point

void * launch_postmaster(void *not_used)
{
   (void) not_used;
   log_ =  get_kernel_log();
//   signal(SIG_POSTMASTER_EXIT, postmaster_quit);
   if (postmaster_ != UNITIALIZED) {
      log_err(log_, "Internal error -- postmaster initialized twice");
      hard_exit("launch_postmaster", 1);
   }
   pool_ = create_memory_pool();
   if (setup_networking() < 0) {
      log_err(log_, "Error in setup_networking()");
      postmaster_ = ERROR;
      goto end;
   }
   postmaster_ = RUNNING;
   handle_messages();
end:
   if (sockfd_ >= 0)
      close(sockfd_);
   log_info(log_, "Postmaster exiting");
   return NULL;
}

#if defined TEST_POSTMASTER

int main(int argc, char **argv)
{
   (void) argc;
   (void) argv;
   launch_postmaster(NULL);
   return 0;
}

#endif   // TEST_POSTMASTER

