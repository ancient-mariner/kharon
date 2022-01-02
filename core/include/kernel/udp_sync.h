#if !defined(UDP_SYNC_H)
#define UDP_SYNC_H
#if !defined(_GNU_SOURCE)
#define _GNU_SOURCE
#endif   // _GNU_SOURCE
#include <netinet/in.h>
#include "pinet.h"

////////////////////////////////////////////////////////////////////////

// sends UDP broadcast packet telling cameras to acquire next image
//
// module publishes time that frame requests were sent, based on
//    hub (server) clock
#define UDP_SYNC_QUEUE_LEN   128
#define UDP_SYNC_ELEMENT_SIZE   (sizeof(double))

////////////////////////////////////////////////////////////////////////

#define UDP_SYNC_CLASS_NAME  "udp_cam_sync"

// initial algorithm is to send image request packet every 200ms 
//    (5Hz, like initial acquisition)
// future approach can vary frequency, eg, send ping immediately after
//    previous frame processed (when processing helpful) or send less 
//    frequent pings (eg, calm conditions)
// TODO investigate if camera can change resolution on the fly

#define UDP_BCAST_INTERVAL   (UDP_SYNC_INTERVAL)

struct udp_sync_class {
   // networking
   int sockfd;
   struct sockaddr_in sock_addr;
   // used by clock_nanosleep -- stores time of next udp ping
   struct timespec bcast_time;
};

// thread entry point
void * udp_sync_class_init(void *);

void send_sync_packet(uint32_t type);

#endif   // UDP_SYNC_H

