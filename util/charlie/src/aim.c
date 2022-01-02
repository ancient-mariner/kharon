#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include "pinet.h"
#include "postmaster.h"

#include "charlie.h"


static void usage(
      /* in     */ const int argc, 
      /* in     */ const char **argv
      )
{
   (void) argc;
   printf("  Set autopilot on specified course, disabling auto tracking\n");
   printf("\n");
   printf("  Usage: %s <heading>\n", argv[0]);
   printf("  where 'heading' is course to head, in degrees.\n");
   printf("  Turn off by setting negative value (tiller should neutralize)\n");
   printf("\n");
   exit(1);
}


static void set_heading(
      /* in     */ const network_id_type *id,
      /* in     */ const int32_t degs
      )
{
   pm_request_type req;
   pm_response_type resp;
   memset(&req, 0, sizeof(req));
   req.request_type = PM_CMD_SET_HEADING;
   req.custom_0 = degs;
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
   } else if (degs < 0) {
      printf("%.3f   Heading disabled\n", t);
   } else {
      printf("%.3f   Heading set to %03d\n", t, degs);
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
   errno = 0;
   int32_t degs = (int32_t) strtol(argv[1], NULL, 10);
   if (errno != 0) {
      printf("Unrecognized parameter '%s': %s\n", argv[1], strerror(errno));
      usage(argc, argv);
   } else if (degs >= 360) {
      printf("Requested heading '%s' is out of range\n", argv[1]);
      usage(argc, argv);
   } else {
      set_heading(id, degs);
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

