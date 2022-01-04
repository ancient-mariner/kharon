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
#include "dev_info.h"
#include "kill_monitor.h"
#include "logger.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <arpa/inet.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <errno.h>

#include "build_version_kill.h"

/*
Send shutdown signal to device
*/

static void send_message(
      /* in     */ const char *device_name
      )
{
printf("sending message to device %s\n", device_name);
   int32_t sockfd = -1;
   network_id_type net_id;
   log_info_type *log = get_kernel_log();
   FILE *fp;
   ///////////////////////////
   // get IP address of target
   fp = open_config_file_ro2(NULL, device_name, NULL, NULL, "ip_addr");
   if (!fp) {
      fprintf(stderr, "Failed to read config for device\n");
      goto done;
   }
   if (config_read_string(fp, net_id.ip, STR_LEN) != 0) {
      fprintf(stderr, "Failed to read config string\n");
      goto done;
   }
   fclose(fp);
   fp = NULL;
   //////////////////////////////////
   // get port number of kill process
   fp = open_config_file_ro2(NULL, device_name, "endpoints", NULL,
         "kill_monitor");
   if (!fp) {
      fprintf(stderr, "Failed to read endpoint for device\n");
      goto done;
   }
   int32_t port = 0;
   if (config_read_int(fp, &port) != 0)
      goto done;
   net_id.port = (uint32_t) port;
   fclose(fp);
   fp = NULL;
   ///////////////////////////////
   // connect to kill process port

printf("Connecting to %s:%d\n", net_id.ip, net_id.port);
   if ((sockfd = connect_to_server_no_wait(&net_id, 0)) < 0) {
      log_err(log, "Unable to connect to %s:%d\n", net_id.ip, net_id.port);
      goto done;
   }
   ////////////////////
   // send kill message
   char packet[KILL_PACKET_BYTES];
   memset(packet, 0, KILL_PACKET_BYTES);
   strcpy(packet, KILL_VERSION);
   strcpy(&packet[KILL_PAYLOAD_OFFSET], KILL_COMMAND_SHUTDOWN);
   send_block(sockfd, packet, KILL_PACKET_BYTES);
done:
   if (sockfd >= 0)
      close(sockfd);
}

int main(int argc, char **argv)
{
   printf("version %s\n", KILL_BUILD_VERSION);
   set_device_dir_path(DEFAULT_DEVICE_DIR_PATH);
   if (argc < 2) {
      printf("Usage: %s <device0> [<device1> [<device2> ...]]\n", argv[0]);
      return -1;
   }
   for (int32_t i=1; i<argc; i++) {
      send_message(argv[i]);
   }
   return 0;
}

