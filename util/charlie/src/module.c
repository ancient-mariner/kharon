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
   printf("  Enable/disable individual modules\n");
   printf("\n");
   printf("  Usage: %s <module name> <on | off>\n", argv[0]);
   printf("\n");
   printf("  When disabled, a module no longer reads or publishes data\n");
   printf("  Functionality not supported in all modules.\n");
   printf("\n");
   exit(1);
}


static void set_module_state(
      /* in     */ const network_id_type *id,
      /* in     */ const char *module_name,
      /* in     */ const uint32_t on_off
      )
{
   pm_request_type req;
   pm_response_type resp;
   memset(&req, 0, sizeof(req));
   req.request_type = on_off;
   req.header_bytes = (uint32_t) strlen(module_name);
   /////////////////////////////////////////////////////////////////////
   // init connection
   int sockfd = -1;
   if ((sockfd = connect_to_server(id)) < 0) {
      printf("Error connecting to postmaster (at %s:%d)\n", id->ip, id->port);
      goto end;
   }
   /////////////////////////////////////////////////////////////////////
   // send data
   if (send_postmaster_request(sockfd, &req, module_name) != 0) {
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
      printf("%.3f   Module %s set to %s\n", t, module_name,
            on_off == PM_CMD_MODULE_RESUME ? "ON" : "OFF");
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
   if (argc != 3) {
      usage(argc, argv);
   }
   const char * module_name = argv[1];
   if (strcmp(argv[2], "on") == 0) {
      set_module_state(id, module_name, PM_CMD_MODULE_RESUME);
   } else if (strcmp(argv[2], "off") == 0) {
      set_module_state(id, module_name, PM_CMD_MODULE_PAUSE);
   } else {
      printf("Unrecognized parameter '%s'\n", argv[2]);
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

