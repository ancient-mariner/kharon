#include "pinet.h"

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

#include "kill_monitor.h"
#include "build_version_kill.h"

/*
Responds to network queries to indicate process (and system) is alive.
Performs system shutdown on request

Listens on port 9000
Accepts UDP packets. First 4 bytes of payload is version string. Version
   is numerical ascii, starting at "001\0". Payload is 256 bytes. Total 
   packet size is 260 bytes. When a response is required, it is 256 bytes.
version 0001:
   char[4]     header
   char[256]   action
                  "alive?" returns "not dead"
                  "halt" performs shutdown of system
*/

// TODO move constants to header file in lib


// network storage
static int sockfd_ = -1;
static struct sockaddr_in  sock_addr_;

static int32_t read_version(
      /* in     */ const char *packet
      )
{
   errno = 0;
   int32_t vers = (int32_t) strtol(packet, NULL, 10);
   if (errno != 0) {
      perror("Unable to parse version number from packet");
   }
   return vers;
}

static void create_udp_socket(void)
{
   // set up socket
   if ((sockfd_ = socket(AF_INET, SOCK_DGRAM, 0)) == -1) {
      perror("Error creating udp receiver socket");
      hard_exit(__func__, 1);
   }
   // bind socket to listening port 
   memset((char *) &sock_addr_, 0, sizeof(sock_addr_));
   sock_addr_.sin_family = AF_INET;
   sock_addr_.sin_port = htons((int16_t) KILL_PORT);
   sock_addr_.sin_addr.s_addr = htonl(INADDR_ANY);
   // share port with others
   int one = 1;
   if (setsockopt(sockfd_, SOL_SOCKET, SO_REUSEADDR, &one, 
            sizeof(one)) != 0) {
      perror("Error setting SO_REUSEADDR in udp sync receiver");
      hard_exit(__func__, 2);
   }
   // bind port
   if (bind(sockfd_, (struct sockaddr*) &sock_addr_, 
         sizeof(sock_addr_))==-1) {
      perror("Error binding to port (udp sync receiver)");
      hard_exit(__func__, 3);
   }
}

static void shutdown_system(void)
{
   printf("SHUTDOWN\n");
   execl("/sbin/shutdown", "shutdown", "-P", "now", NULL);
}

static void reboot_system(void)
{
   printf("REBOOT\n");
   execl("/sbin/shutdown", "shutdown", "-r", "now", NULL);
}

static void send_alive_response(
      /* in     */ const int connfd
      )
{
   char buf[KILL_PAYLOAD_BYTES];
   strcpy(buf, KILL_COMMAND_ALIVE_RESPONSE);
   send_block(connfd, buf, KILL_PAYLOAD_BYTES);
}

int main(int argc, char **argv)
{
   (void) argc;
   (void) argv;
   printf("version %s\n", KILL_BUILD_VERSION);
   daemon(1, 0);
   // setup network
   create_udp_socket();
   /////////////////////////////
   // main body
   //
   // loop forever receiving packets, or at least until told to stop
   char packet[KILL_PACKET_BYTES];
   const int16_t port = (int16_t) KILL_PORT;
   int sockfd = init_server(port);
   if (sockfd < 0) {
      fprintf(stderr, "Unable to create server on port %d. Exiting\n", port);
      goto done;
   }
   int connfd = -1;
   while (1) {
      // wait for connection; don't check communication protocol version
      if ((connfd = wait_for_connection_no_check(sockfd, "kill-monitor")) < 0) {
         fprintf(stderr, "Connection error -- exiting\n");
         goto done;
      }
      if (recv_block(connfd, packet, KILL_PACKET_BYTES) < 0) {
         fprintf(stderr, "Error receiving data\n");
         goto done;
      }
      int32_t vers = read_version(packet);
      switch (vers) {
         case 1:
         {
            char *cmd = &packet[KILL_PAYLOAD_OFFSET];
            if (strcmp(KILL_COMMAND_ALIVE, cmd) == 0) {
               send_alive_response(connfd);
               close(connfd);
               connfd = -1;
            } else if (strcmp(KILL_COMMAND_SHUTDOWN, cmd) == 0) {
               close(connfd);
               connfd = -1;
               shutdown_system();
            } else if (strcmp(KILL_COMMAND_REBOOT, cmd) == 0) {
               close(connfd);
               connfd = -1;
               reboot_system();
            } else {
               fprintf(stderr, "Unknown packet command: '%s'\n", cmd);
            }
            break;
         }
         default:
            fprintf(stderr, "Unrecognized packet version (%d)\n", vers);
      };
   }
done:
   /////////////////////////////
   // shutdown
   if (sockfd_ != -1) {
      close(sockfd_);
      sockfd_ = -1;
   }
   return 0;
}

