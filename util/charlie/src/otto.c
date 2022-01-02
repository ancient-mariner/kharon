#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "pinet.h"
#include "postmaster.h"

#include "charlie.h"


static void usage(
      /* in     */ const int argc, 
      /* in     */ const char **argv
      )
{
   (void) argc;
   printf("  Enables/disables sending NMEA commands to autopilot\n");
   printf("\n");
   printf("  Usage: %s [on | off]\n", argv[0]);
   printf("  Default state is 'off' but that's overridable in config file.\n");
   printf("\n");
   printf("  Autopilot should be connected before turning on\n");
   printf("\n");
   exit(1);
}


static void set_autopilot_state(
      /* in     */ const network_id_type *id,
      /* in     */ const uint32_t on_off
      )
{
   pm_request_type req;
   pm_response_type resp;
   memset(&req, 0, sizeof(req));
   req.request_type = on_off;
   /////////////////////////////////////////////////////////////////////
   // init connection
   int sockfd = -1;
   if ((sockfd = connect_to_server(id)) < 0) {
      printf("Error connecting to postmaster (at %s:%d)\n", id->ip, id->port);
      goto end;
   }
   /////////////////////////////////////////////////////////////////////
   // send data
   if (send_postmaster_request(sockfd, &req, NULL) != 0) {
      printf("Error sending packet to postmaster\n");
      goto end;
   }
   if (read_postmaster_response(sockfd, &req, &resp, NULL) != 0) {
      printf("Error reading packet from postmaster\n");
      goto end;
   }
   //
   double t = atof((char*) resp.t);
   if (resp.request_type == PM_CMD_NULL) {
      printf("%.3f   Postmaster indicated error executing request", t);
   } else {
      printf("%.3f   Autopilot set to %s\n", t, 
            on_off == PM_CMD_AUTOPILOT_ON ? "ON" : "OFF");
   }
end:
   if (sockfd >= 0) {
      close(sockfd);
   }
}


static void process_command(
      /* in     */ const int argc, 
      /* in     */ const char **argv,
      /* in     */ const network_id_type *id
      )
{
   if (argc != 2) {
      usage(argc, argv);
   }
   if (strcmp(argv[1], "on") == 0) {
      set_autopilot_state(id, PM_CMD_AUTOPILOT_ON);
   } else if (strcmp(argv[1], "off") == 0) {
      set_autopilot_state(id, PM_CMD_AUTOPILOT_OFF);
   } else {
      printf("Unrecognized parameter '%s'\n", argv[1]);
      usage(argc, argv);
   }
}


int main(const int argc, const char **argv)
{
   network_id_type id;
   set_device_dir_path(KHARON_DEVICE_DIR);
   get_postmaster_address(&id);
   process_command(argc, argv, &id);
   return 0;
}

