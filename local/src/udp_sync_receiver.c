#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <stdint.h>
#include <arpa/inet.h>
#include <string.h>
#include <pthread.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <errno.h>

#include "udp_sync_receiver.h"
#include "pinet.h"
#include "timekeeper.h"

static void wake_heart(void);
//void register_camera(void);
static void create_udp_socket(void);
void * udp_receiver_main(void*);

// receives UDP sync packets and performs actions according to 
//    packet contents (eg, updating time, signaling frame capture)
// 
// 2 threads active -- UDP receiver and heartbeat thread. receiver
//    gets commands and sets state. heartbeat tracks state, state
//    changes and sends camera commands


// assign bogus value for initial value
static pthread_t s_tid = (pthread_t) -1;
static int s_thread_init = -1;

// network storage
static int s_sockfd = -1;
static struct sockaddr_in  s_sock_addr;


////////////////////////////////////////////////////////////////////////
// syncronization
//
// if camera registered then create a 'heartbeat' thread that sends a
//    SIGUSR1 signal to the camera to trigger frame acquisition
// heartbeat checks to see if camera output is paused before sending 
//    signal. if yes, thread enters indefinite sleep. on each waking,
//    thread checks to see if pause is over. when so, thread resumes
//    heartbeat pulses.

static int s_heartbeat_active = 0;
static pthread_t s_heartbeat_tid;
static pthread_t s_camera_tid;
static int s_camera_registered = 0;

// sigusr2 handler -- this is a no-op. the role of sigusr2 is to 
//    wake the thread if it's sleeping. callback does nothing
static void sigusr2_callback(int sig)
{
   if (sig != SIGUSR2) {
      fprintf(stderr, "usr_sync unexpectedly received signal %d\n", sig);
      hard_exit("udp_sync::sigusr2_callback", 2);
   }
}

static void wake_heart()
{
   if (s_heartbeat_active) {
      if (pthread_kill(s_heartbeat_tid, SIGUSR2) != 0) {
         perror("pthread_kill sigusr2 failure (heartbeat)");
      }
   }
}

static void * heartbeat_entry(void * not_used)
{
   (void) not_used;
   int streaming = 0;
   struct timespec ts;
   int rc;
   // set up thread to catch sigusr2
   signal(SIGUSR2, sigusr2_callback);
   // start main loop
   clock_gettime(CLOCK_MONOTONIC, &ts);
   while (s_heartbeat_active) {
      while (streaming != 0) {
         // send frame-capture signal unless pause indicated
         if (!MESSAGE_BOARD.acquiring || MESSAGE_BOARD.paused) {
            streaming = 0;
            break;
         }
         // camera thread response to SIGUSR1
         if (s_camera_registered) {
            // send a signal telling the camera to take a picture
            if (pthread_kill(s_camera_tid, SIGUSR1) != 0) {
               perror("pthread_kill sigusr1 failure (camera)");
            }
         }
         // wait for next frame
         increment_timef(&ts, CAMERA_FRAME_INTERVAL);
         while ((rc = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, 
                     &ts, NULL)) != 0) {
            if (rc != EINTR) {
               perror("heartbeat sleep problem");
               hard_exit("heartbeat_entry", 1);
            } else {
            }
         }
      }
      while (streaming == 0) {
         // wait until time to continue
         // on each sleep interruption, check pause state and resume
         //    frame capture when pause complete
         increment_timef(&ts, 1.0);
         rc = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);
         if (rc != 0) {
            if (rc != EINTR) {
               perror("heartbeat sleep problem");
               hard_exit("heartbeat_entry", 2);
            } else {
            }
         }
         if (MESSAGE_BOARD.acquiring && !MESSAGE_BOARD.paused) {
            // reset clock so frame capture signals are synced to wake time
            clock_gettime(CLOCK_MONOTONIC, &ts);
            streaming = 1;
            break;
         }
      }
   }
   return NULL;
}

// called by camera process. starts 'heartbeat' thread that sends 
//    periodic SIGUSR1 signals to camera process
void register_camera()
{
   s_heartbeat_active = 1;
   s_camera_tid = pthread_self();
   s_camera_registered = 1;
   if (pthread_create(&s_heartbeat_tid, NULL, heartbeat_entry, NULL) != 0) {
      perror("Error creating camera hearbeat thread");
      hard_exit("udp_sync_receiver:register_camera", 1);
   }
}

////////////////////////////////////////////////////////////////////////

static void create_udp_socket()
{
   // set up socket
   if ((s_sockfd = socket(AF_INET, SOCK_DGRAM, 0)) == -1) {
      perror("Error creating udp receiver socket");
      hard_exit(__func__, 1);
   }
   // bind socket to listening port 
   memset((char *) &s_sock_addr, 0, sizeof(s_sock_addr));
   s_sock_addr.sin_family = AF_INET;
   s_sock_addr.sin_port = htons(UDP_SYNC_PORT);
   s_sock_addr.sin_addr.s_addr = htonl(INADDR_ANY);
   // share port with others
   int one = 1;
   if (setsockopt(s_sockfd, SOL_SOCKET, SO_REUSEADDR, &one, 
            sizeof(one)) != 0) {
      perror("Error setting SO_REUSEADDR in udp sync receiver");
      hard_exit(__func__, 2);
   }
   // bind port
   if (bind(s_sockfd, (struct sockaddr*) &s_sock_addr, 
         sizeof(s_sock_addr))==-1) {
      perror("Error binding to port (udp sync receiver)");
      hard_exit(__func__, 3);
   }
}

// thread entry
void * udp_receiver_main(void *not_used)
{
   (void) not_used;
   /////////////////////////////
   // initialization
   //
   // setup network
   create_udp_socket();
   /////////////////////////////
   // main body
   //
   // loop forever receiving packets, or at least until told to stop
   int running = 1;
   unsigned int sz=sizeof(s_sock_addr);
   const size_t packet_len = sizeof(struct udp_sync_packet);
   struct udp_sync_packet packet;
   uint8_t *bpacket = (uint8_t*) &packet;
   ssize_t n;
   ssize_t bytes_read;
   // loop while listening
   while (running) {
      // wait for next packet
      bytes_read = 0;
      while ((size_t) bytes_read < packet_len) {
         n = recvfrom(s_sockfd, &bpacket[bytes_read], packet_len, MSG_WAITALL, 
               (struct sockaddr*) &s_sock_addr, &sz);
         if (n < 0) {
            perror("UDP sync receiver interrupted while listening");
            running = 0;
            break;
         }
         bytes_read += n;
      }
      if (!running)
         break;
//printf("Inbound sync packet (%08x) at %f\n", packet.packet_type, now());
      if (packet.packet_type & UDP_SYNC_PACKET_EXIT) {
         MESSAGE_BOARD.exit = 1;
         if (s_camera_registered) {
            if (pthread_kill(s_camera_tid, SIGUSR1) != 0) {
               perror("pthread_kill sigusr1 failure (comm thread)");
            }
         }
         break;
      } else {
         // exit not requested so process rest of packet
         if (packet.packet_type & UDP_SYNC_PACKET_TIME) {
            timekeeper_set_time(packet.timestamp);
         }
         if (packet.packet_type & UDP_SYNC_PACKET_PAUSE) {
            MESSAGE_BOARD.paused = 1;
         }
         if (packet.packet_type & UDP_SYNC_PACKET_CONTINUE) {
            MESSAGE_BOARD.paused = 0;
            // signal usr2 to wake up heartbeat thread
            wake_heart();
         }
         if (packet.packet_type & UDP_SYNC_PACKET_STOP_ACQ) {
            MESSAGE_BOARD.acquiring = 0;
         }
         if (packet.packet_type & UDP_SYNC_PACKET_START_ACQ) {
            MESSAGE_BOARD.acquiring = 1;
            // signal usr2 to wake up heartbeat thread
            wake_heart();
         }
      }
   }
   /////////////////////////////
   // shutdown
   //
   if (s_sockfd != -1) {
      close(s_sockfd);
      s_sockfd = -1;
   }
   return NULL;
}

int create_sync_receiver()
{
   // this was probably done elsewhere, but do it here just to make sure
   init_timekeeper();   
   // 
   if (s_thread_init >= 0) {
      fprintf(stderr, "Error - attempted to create multiple udp sync "
            "receivers\n");
      return -1;
   }
   s_thread_init = 1;
   if (pthread_create(&s_tid, NULL, udp_receiver_main, NULL) != 0) {
      perror("Error creating udp receiver thread");
      hard_exit(__func__, 1);
   }
   return 0;
}

void shutdown_sync_receiver()
{
   // break socket so thread can exit
   if (s_sockfd >= 0) {
      if (shutdown(s_sockfd, SHUT_RDWR) != 0) {
         perror("Error shutting down udp sync socket");
         hard_exit(__func__, 1);
      }
      s_sockfd = -1;
   }
   // join pthread
   if (s_thread_init >= 0) {
      pthread_join(s_tid, NULL);
      s_thread_init = -1;
   }
}

