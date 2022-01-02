#if !defined(_GNU_SOURCE)
#define _GNU_SOURCE
#endif   // _GNU_SOURCE
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <execinfo.h>
#include <signal.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/types.h>
#include <fcntl.h>
#include <errno.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <math.h>
#if defined(INTEL)
#include <xmmintrin.h>
#include <pmmintrin.h>
#endif   // INTEL
#include <assert.h>
#include "pinet.h"
#include "timekeeper.h"
#include "sensor_packet.h"
#include "lin_alg.h"
#include "logger.h"

////////////////////////////////////////////////////////////////////////
// globals

// TODO consider deprecating and switching to DEG_TO_PIX and PIX_TO_DEG 
//    to be consistent w/ BAM constants. or not -- PPD is a widely used
//    metric
double PIX_PER_DEG[NUM_PYRAMID_LEVELS] = { 0.0 };
double DEG_PER_PIX[NUM_PYRAMID_LEVELS] = { 0.0 };
uint32_t BAM32_PER_PIX[NUM_PYRAMID_LEVELS] = { 0 };

//double DEG_TO_PIX[NUM_PYRAMID_LEVELS] = { 0.0 };
//double PIX_TO_DEG[NUM_PYRAMID_LEVELS] = { 0.0 };
//uint32_t PIX_TO_BAM32[NUM_PYRAMID_LEVELS] = { 0 };

uint32_t WORLD_HEIGHT_PIX[NUM_PYRAMID_LEVELS] = { 0 };
uint32_t WORLD_WIDTH_PIX[NUM_PYRAMID_LEVELS] = { 0 };

double WORLD_HEIGHT_ABOVE_HORIZ_DEGS = -1.0;
double WORLD_HEIGHT_BELOW_HORIZ_DEGS = -1.0;
double WORLD_HEIGHT_DEGS = -1.0;

int globals_accessed_ = 0;

// sets pixels-per-degree and global variables that depend on it
void set_ppd(
      /* in     */ const double ppd
      )
{
   // this cannot be called after the values it changes have been used 
   // there's not presently a fool-proof way to prevent misdeeds, but at
   //    least try
   if (globals_accessed_) {
      fprintf(stderr, "Attempted to set variables related to "
            "pixels-per-degree after global variables we accessed\n");
      fprintf(stderr, "This sounds like a config error -- bailing out "
            "something breaks\n");
      hard_exit(__FILE__, __LINE__);
   }
   if (ppd <= 0.01) {
      fprintf(stderr, "Setting pix_per_degree to unexpectedly low value: %f\n",
            (double) ppd);
      hard_exit(__FILE__, __LINE__);
   }
   double scale = 1.0;
   for (uint32_t lev=0; lev<NUM_PYRAMID_LEVELS; lev++) {
      const double ppd_n = ppd * scale;
      // ppd
      PIX_PER_DEG[lev] = ppd_n;
      DEG_PER_PIX[lev] = 1.0 / ppd_n;
//      PIX_TO_DEG[lev] = DEG_PER_PIX[lev];
//      DEG_TO_PIX[lev] = PIX_PER_DEG[lev];
//      DEG_TO_PIX[lev] = ppd_n;
//      PIX_TO_DEG[lev] = 1.0 / ppd_n;
      // deg/pix is small and far from rolling over, so it's safe
      //    to use multiplication constant
      // bam32/pix = deg/pix * bam32/deg
      // bam32/pix = deg/pix * deg_to_bam32
      double pix_to_bam32 = 1.0 / ppd_n * DEG_TO_BAM32_;
      assert(pix_to_bam32 > 0.0);
      BAM32_PER_PIX[lev] = (uint32_t) (1.0 / ppd_n * DEG_TO_BAM32_);
      // world size
      WORLD_HEIGHT_PIX[lev] = (uint32_t) (WORLD_HEIGHT_DEGS * ppd_n + 0.0001);
      WORLD_WIDTH_PIX[lev] = (uint32_t) (WORLD_WIDTH_DEGS * ppd_n + 0.0001);
      //
      scale *= 0.5;
   }
}

void set_world_height(
      /* in     */ const double above_horizon, 
      /* in     */ const double below_horizon
      )
{
   WORLD_HEIGHT_ABOVE_HORIZ_DEGS = above_horizon;
   WORLD_HEIGHT_BELOW_HORIZ_DEGS = below_horizon;
   WORLD_HEIGHT_DEGS = above_horizon + below_horizon;
   if (PIX_PER_DEG[0] > 0.0) {
      // complete update
      set_ppd(PIX_PER_DEG[0]);
   }
}

////////////////////////////////////////////////////////////////////////

struct message_board MESSAGE_BOARD = { 1, 0, 0 };

void print_hex(uint8_t *buf, int n_bytes)
{
   for (int ctr=0; ctr<n_bytes; ctr+=16) {
      printf("%08x  ", ctr);
      for (int i=0; i<16; i++) {
         printf("%02x", buf[ctr+i]);
         if (i & 0x01)
            printf(" ");
      }
      printf(" ");
      for (int i=0; i<16; i++) {
         printf("%c", buf[ctr+i]<32?'.':buf[ctr+i]);
      }
      printf("\n");
   }
}


////////////////////////////////////////////////////////////////////////

// returns socket FD for connection to server
int connect_to_server(
      /* in     */ const network_id_type *net_id
      )
{
//fprintf(stderr, "Connect to server %s:%d\n", net_id->ip, net_id->port);
   int sockfd = -1;
   int rc;
   log_info_type *log = get_kernel_log();
   struct sockaddr_in conn;
   //
   if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
      log_err(log, "Unable to create socket: %s", strerror(errno));
      goto err;
   }
   // connect
   memset(&conn, 0, sizeof(conn));
   conn.sin_family = AF_INET;
   conn.sin_port = htons((uint16_t) net_id->port);
   inet_pton(AF_INET, net_id->ip, &conn.sin_addr);
   // try to connect
   if ((rc = connect(sockfd, (const struct sockaddr*) &conn, 
            sizeof(conn))) < 0) {
      log_err(log, "client unable to connect to %s:%d: %s", 
            net_id->ip, net_id->port, strerror(errno));
      goto err;
   }
   // send communication protocol number -- server will send '1' if 
   //    protocol numbers match and -1 if not
   if (send_protocol_version(sockfd, COMMUNICATION_PROTOCOL_MAJOR,
         COMMUNICATION_PROTOCOL_MINOR) != 0) 
      goto err;
   // if we made it here, the connection was created successfully and
   //    the server accepted our communication protocol version
   // exit connection block
   goto done;
err:
   if (sockfd >= 0) {
      log_err(log, "Connection error -- closing socket");
      close(sockfd);
      sockfd = -1;
   }
done:
   return sockfd;
}


int connect_to_server_no_wait(
      /* in     */ const network_id_type *net_id,
      /* in     */ const int check_protocol  // 1 for yes, 0 for no
      )
{
   int sockfd = -1;
   int rc;
   log_info_type *log = get_kernel_log();
   struct sockaddr_in conn;
   //
   if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
      log_err(log, "Unable to create socket: %s", strerror(errno));
      goto err;
   }
   // make socket be non-blocking (for connect)
   int fcntl_arg = 0;
   if( (fcntl_arg = fcntl(sockfd, F_GETFL, NULL)) < 0) { 
      log_err(log, "Error fcntl(..., F_GETFL) (%s)", strerror(errno)); 
      goto err;
   } 
   if( fcntl(sockfd, F_SETFL, fcntl_arg | O_NONBLOCK) < 0) { 
      log_err(log, "Error fcntl(..., F_SETFL) (%s)", strerror(errno)); 
      goto err;
   } 
   // connect
   memset(&conn, 0, sizeof(conn));
   conn.sin_family = AF_INET;
   conn.sin_port = htons((uint16_t) net_id->port);
   inet_pton(AF_INET, net_id->ip, &conn.sin_addr);
   // try to connect
   if ((rc = connect(sockfd, (const struct sockaddr*) &conn, 
            sizeof(conn))) < 0) {
      // the code here is supposed to work using a timeout value for
      //    the connect (timeout is the last param for select())
      // that's not working, but the connection does fail immediately
      //    if the target is not available, which is good enough
      if (errno == EINPROGRESS) { 
         fd_set fdset;
         FD_ZERO(&fdset);
         FD_SET(sockfd, &fdset);
         int res = select(sockfd+1, NULL, &fdset, NULL, NULL);
         if ((res < 0) && (errno != EINTR)) { 
            log_err(log, "Error connecting %d - %s", errno, strerror(errno)); 
            goto err;
         } else if (res > 0) {
            // Socket selected for write 
            socklen_t lon = sizeof(int); 
            int32_t valopt;
            res = getsockopt(sockfd, SOL_SOCKET, SO_ERROR, 
                  (void*)(&valopt), &lon);
            if (res < 0) { 
               log_err(log, "Error in getsockopt() %d - %s", errno, 
                     strerror(errno)); 
               goto err;
            } 
            // Check the value returned... 
            if (valopt) { 
               log_err(log, "Error in delayed connection() %d - %s", 
                     valopt, strerror(valopt)); 
               goto err;
            } 
         } else {
            log_err(log, "Timeout in select() -- cancelling");
            goto err;
         }
      } else {
         log_err(log, "Client unable to connect to %s:%d: %s", 
               net_id->ip, net_id->port, strerror(errno));
         goto err;
      }
   }
   // restore timeout
   fcntl(sockfd, F_SETFL, fcntl_arg);
   if (check_protocol) {
      // send communication protocol number -- server will send '1' if 
      //    protocol numbers match and -1 if not
      if (send_protocol_version(sockfd, COMMUNICATION_PROTOCOL_MAJOR,
            COMMUNICATION_PROTOCOL_MINOR) != 0) 
         goto err;
   }
   // if we made it here, the connection was created successfully and
   //    the server accepted our communication protocol version
   // exit connection block
   goto done;
err:
   if (sockfd >= 0) {
      close(sockfd);
      sockfd = -1;
   }
done:
   return sockfd;
}


int init_server(
      /* in     */ const int16_t port
      )
{
   return init_server_backlog(port, 0);
}

int init_server_backlog(
      /* in     */ const int16_t port,
      /* in     */ const int16_t backlog
      )
{
   int sockfd = -1;
   int opt = 1;
   struct sockaddr_in server;
   memset(&server, 0, sizeof(server));
   server.sin_family = AF_INET;
   server.sin_addr.s_addr = INADDR_ANY;
   server.sin_port = htons((uint16_t) port);
   log_info_type *log = get_kernel_log();
   log_info(log, "Server listening on port %d", port);
   // create socket
   sockfd = socket(AF_INET, SOCK_STREAM, 0);
   if (sockfd < 0) {
      log_err(log, "Unable to create server socket: %s", strerror(errno));
      goto err;
   }
   // set option so port number can be re-used to re-establish server
   if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) != 0) {
      log_err(log, "setsockopt(... SO_REUSEADDR ...): %s", strerror(errno));
      goto err;
   }
   // bind socket
   if (bind(sockfd, (struct sockaddr*) &server, sizeof(server)) < 0) {
      log_err(log, "Failed to bind server socket: %s", strerror(errno));
      goto err;
   }
   // listen on socket
   if (listen(sockfd, backlog) < 0) {
      log_err(log, "Failed to listen on server socket: %s", strerror(errno));
      goto err;
   }
   // server ready to accept connections
   goto done;
err:
printf("socket error -- check logs\n");
   if (sockfd >= 0)
      close(sockfd);
   sockfd = -1;
done:
printf("done creating socket (%d) on port %d\n", sockfd, port);
   return sockfd;
}

int wait_for_connection_no_check(int sockfd, const char* name)
{
   int connfd = -1;
   connfd = accept(sockfd, (struct sockaddr*) NULL, NULL);
   if (connfd < 0) {
      log_info_type *log = get_kernel_log();
      if (errno == EINVAL) {
         log_err(log, "[%s] Connection aborted during accept()", name);
         close(connfd);
         connfd = -2;
         goto done;
      } else {
         log_err(log, "[%s] Unable to accept connection: %s", name,
               strerror(errno));
      }
      goto err;
   }
   // 
   goto done;
err:
   if (connfd >= 0)
      close(connfd);
   connfd = -1;
done:
   return connfd;
}

int wait_for_connection(int sockfd, const char* name)
{
   int connfd = -1;
   connfd = accept(sockfd, (struct sockaddr*) NULL, NULL);
   if (connfd < 0) {
      log_info_type *log = get_kernel_log();
      if (errno == EINVAL) {
         log_err(log, "[%s] Connection aborted during accept()", name);
         close(connfd);
         connfd = -2;
         goto done;
      } else {
         log_err(log, "[%s] Unable to accept connection: %s", name,
               strerror(errno));
      }
      goto err;
   }
   // check client's communication protocol version
   if (check_protocol_version(connfd, COMMUNICATION_PROTOCOL_MAJOR,
            COMMUNICATION_PROTOCOL_MINOR) != 0)
   {
      log_info_type *log = get_kernel_log();
      log_err(log, "[%s] protocol version mismatch with connecting device", 
            name);
      goto err;
   }
   // 
   goto done;
err:
   if (connfd >= 0)
      close(connfd);
   connfd = -1;
done:
   return connfd;
}

////////////////////////////////////////////////////////////////////////

//// returns ip address for specified IPv4 network interface
//void get_host_ip(const char *iface, char *ip)
//{
//   getifaddrs(&addrs);
//   tmp = addrs;
//
//   while(tmp) {
//      if (tmp->ifa_addr && tmp->ifa_addr->sa_family == AF_INET) {
//         struct sockaddr_in *pAddr = (struct sockaddr_in*) tmp->ifa_addr;
//printf("%s: %s\n", tmp->ifa_name, inet_ntoa(pAddr->sin_addr));
//         if (strcmp(iface, tmp->ifa_name) == 0) {
//            sprintf(ip, "%s", inet_ntoa(pAddr->sin_addr));
//         }
//      }
//      tmp = tmp->ifa_next;
//   }
//   freeifaddrs(addrs);
//}
//

////////////////////////////////////////////////////////////////////////
// protocol checking

struct protocol_packet {
   uint32_t major, minor;
   char timestamp[TIMESTAMP_STR_LEN];  // stored as %.4f
};

// send communication protocol version to server and read server's version
// returns 0 on success, -1 on failure
int send_protocol_version(int sockfd, uint32_t major, uint32_t minor)
{
   int rc = -1;
   struct protocol_packet local_protocol, remote_protocol;
   memset(&local_protocol, 0, sizeof local_protocol);
   memset(&remote_protocol, 0, sizeof remote_protocol);
   // initialize client's protocol packet and send it
   local_protocol.major = htonl(major);
   local_protocol.minor = htonl(minor);
   double start = system_now();
   local_protocol.timestamp[0] = 0;
   if (write(sockfd, &local_protocol, sizeof(local_protocol)) != 
         sizeof(local_protocol)) {
      log_info_type *log = get_kernel_log();
      log_err(log, "Unable to send protocol version packet to server: %s", 
            strerror(errno));
      goto err;
   }
   // read server's response
   if (read(sockfd, &remote_protocol, sizeof(remote_protocol)) != 
         sizeof(remote_protocol)) {
      log_info_type *log = get_kernel_log();
      log_err(log, "Unable to read protocol packet from server: %s", 
            strerror(errno));
      goto err;
   }
   // compare protocol version numbers
   uint32_t server_major = htonl(remote_protocol.major);
   uint32_t server_minor = htonl(remote_protocol.minor);
   double end = system_now();
   // estimated server time is what's sent in the packet plus half
   //    of the duration from protocol send to recv
   double server_time = strtod(remote_protocol.timestamp, NULL) +
         (end - start) / 2.0;
   timekeeper_set_time_f(server_time);
   if ((major != server_major) || (minor != server_minor)) {
      log_info_type *log = get_kernel_log();
      log_err(log, "Communication local_protocol fault");
      log_err(log, "Client running %d.%d", major, minor);
      log_err(log, "Server running %d.%d", server_major, server_minor);
      goto err;
   }
   rc = 0;
err:
   return rc;
}

// returns 0 on success, -1 on failure
int check_protocol_version(int sockfd, uint32_t major, uint32_t minor)
{
   int rc = -1;
   struct protocol_packet local_protocol, remote_protocol;
   memset(&local_protocol, 0, sizeof local_protocol);
   memset(&remote_protocol, 0, sizeof remote_protocol);
   // partially initialize protocol packet describing server protocol
   local_protocol.major = htonl(major);
   local_protocol.minor = htonl(minor);
   // read client's protocol
   if (read(sockfd, &remote_protocol, sizeof(remote_protocol)) != 
         sizeof(remote_protocol)) {
      log_info_type *log = get_kernel_log();
      log_err(log, "Unable to read protocol version packet from client: %s", 
            strerror(errno));
      goto err;
   }
   // finish local protocol packet by writing present time
   snprintf(local_protocol.timestamp, TIMESTAMP_STR_LEN, "%.4f", system_now());
   // send response
   if (write(sockfd, &local_protocol, sizeof(local_protocol)) != 
         sizeof(local_protocol)) {
      log_info_type *log = get_kernel_log();
      log_err(log, "Unable to send protocol version packet to client: %s",
            strerror(errno));
      goto err;
   }
   // check protocol of client and signal error if it's incorrect
   uint32_t client_major = htonl(remote_protocol.major);
   uint32_t client_minor = htonl(remote_protocol.minor);
   if ((major != client_major) || (minor != client_minor)) {
      log_info_type *log = get_kernel_log();
      log_err(log, "Communication protocol fault");
      log_err(log, "Client running %d.%d", client_major, client_minor);
      log_err(log, "Server running %d.%d", major, minor);
      goto err;
   }
   rc = 0;
err:
   return rc;
}

////////////////////////////////////////////////////////////////////////
// 

int recv_block(int sockfd, void *data, uint32_t len)
{
   size_t bytes_left = (size_t) len;
   size_t bytes_read = 0;
   while (bytes_left > 0) {
      ssize_t n = read(sockfd, &((char*)data)[bytes_read], bytes_left);
      if (n < 0) {
         log_info_type *log = get_kernel_log();
         log_err(log, "Socket read error: %s", strerror(errno));
         return -1;
      } else if ((n == 0) && (len > 0)) {
         log_info_type *log = get_kernel_log();
         log_err(log, "Failure reading socket data. Aborting");
         return -1;
      }
      bytes_left -= (size_t) n;
      bytes_read += (size_t) n;
   }
   return (int) bytes_read;
}

int send_block(int sockfd, const void *data, uint32_t len)
{
   size_t bytes_left = (size_t) len;
   size_t bytes_written = 0;
   while (bytes_left > 0) {
      ssize_t n = write(sockfd, 
            &((const char*)data)[bytes_written], bytes_left);
      if (n < 0) {
         log_info_type *log = get_kernel_log();
         log_err(log, "Socket write error: %s", strerror(errno));
         return -1;
      } else if ((n == 0) && (len > 0)) {
         log_info_type *log = get_kernel_log();
         log_err(log, "Failure sending socket data. Aborting");
         return -1;
      }
      bytes_left -= (size_t) n;
      bytes_written += (size_t) n;
   }
   return (int) bytes_written;
}

// creates sensor packet header structure
void serialize_sensor_header(
      /* in     */ uint32_t type,
      /* in     */ double timestamp,
      /*    out */ struct sensor_packet_header *pkt)
{
   memset(pkt, 0, sizeof(*pkt));
   pkt->sensor_type = htonl(type);
   sprintf(pkt->timestamp, "%.4f", timestamp);
}

void serialize_sensor_header2(
      /* in     */ uint32_t type,
      /* in     */ double timestamp,
      /* in     */ double timestamp_2,
      /*    out */ struct sensor_packet_header *pkt)
{
   memset(pkt, 0, sizeof(*pkt));
   pkt->sensor_type = htonl(type);
   sprintf(pkt->timestamp, "%.4f", timestamp);
   sprintf(pkt->timestamp_2, "%.4f", timestamp_2);
}

// extracts data from sensor packet header
void unpack_sensor_header(
      /* in     */ struct sensor_packet_header *pkt,
      /*    out */ uint32_t *type,
      /*    out */ double *timestamp)
{
   *type = htonl((uint32_t) pkt->sensor_type);
   *timestamp = atof(pkt->timestamp);
}

// extracts data from sensor packet header
void unpack_sensor_header2(
      /* in     */ struct sensor_packet_header *pkt,
      /*    out */ uint32_t *type,
      /*    out */ double *t_request,
      /*    out */ double *t_received
      )
{
   *type = htonl((uint32_t) pkt->sensor_type);
   *t_request = atof(pkt->timestamp);
   *t_received = atof(pkt->timestamp_2);
}

// set all fields in sensor packet to 0
void clear_imu_sensor_packet(
      /* in out */        imu_sensor_packet_type *pkt
      )
{
   // clear vectors manually, to avoid requiring lin_alg
   for (uint32_t i=0; i<3; i++) { pkt->gyr.v[i] = 0.0f;  }
   for (uint32_t i=0; i<3; i++) { pkt->acc.v[i] = 0.0f;  }
   for (uint32_t i=0; i<3; i++) { pkt->mag.v[i] = 0.0f;  }
   for (uint32_t i=0; i<3; i++) { pkt->gps.v[i] = 0.0f;  }
//   zero_vector(&pkt->gyr);
//   zero_vector(&pkt->acc);
//   zero_vector(&pkt->mag);
//   zero_vector(&pkt->gps);
   pkt->temp = 0.0f;
   pkt->baro = 0.0f;
}

///////////////////////////////////////////////////////////////////////
// misc

// trims leading and trailing white space. if entire string is WS, returns
//    NULL. otherwise returns pointer to first non-WS char, which equals
//    the input 'str'
// string is moved to beginning of buffer with whitespace removed
char *trim_whitespace(char *str)
{
   // strip WS from beginning of string
   if (isspace((uint8_t) *str)) {
      // find first non-ws character
      char *src = str;
      while (isspace((uint8_t) *src)) {
         src++;
      }
      // if no non-WS content, return NULL
      if (*src == 0)
         return NULL;
      // copy content to front of buffer
      char *dest = str;
      while (*src) 
         *dest++ = *src++;
      *dest = 0;
   }
   // trim trailing space
   char * end = str + strlen(str) - 1;
   while (end > str && isspace((uint8_t) *end))
      end--;
   // terminate string
   *(end+1) = 0;
   // done
   return str;
}

char *trim_leading_whitespace(char *str)
{
   // strip WS from beginning of string
   if (isspace((uint8_t) *str)) {
      // find first non-ws character
      char *src = str;
      while (isspace((uint8_t) *src)) {
         src++;
      }
      // if no non-WS content, return NULL
      if (*src == 0)
         return NULL;
      // copy content to front of buffer
      char *dest = str;
      while (*src) 
         *dest++ = *src++;
      *dest = 0;
   }
   return str;
}

#if defined UNIT_TEST_PINET
static uint32_t test_trim_whitespace(void)
{
   char buf[256];
   strcpy(buf, "  this is a house  ");
   assert(trim_whitespace(buf) == buf);
   assert(strcmp(buf, "this is a house") == 0);
   strcpy(buf, "  \t\n ");
   assert(trim_whitespace(buf) == NULL);
}
#endif   // UNIT_TEST_PINET

///////////////////////////////////////////////////////////////////////
// exit, signal handlers and stack tracing


void hard_exit(const char *source, int code)
{
   log_info_type *log = get_kernel_log();
   log_err(log, "Hard exit by %s (%d)", source, code);
   print_stack_trace();
   fflush(stderr);
printf("hard_exit -> close_logs()\n");
   close_logs();
   fflush(stdout);
   log_errors_to_stdout();
   assert(0);
   _exit(code);
}

void hard_exit2(const char *file, const char *func, int code)
{
   log_info_type *log = get_kernel_log();
   log_err(log, "Hard exit by %s:%s (%d)", file, func, code);
   fflush(stderr);
printf("hard_exit2 -> close_logs()\n");
   close_logs();
   fflush(stdout);
   log_errors_to_stdout();
   assert(0);
   _exit(code);
}

// link with -rdynamic to get meaningful function names
void print_stack_trace()
{
	static const int MAX_FRAMES = 256;
	void *frames[MAX_FRAMES];
	int32_t sz = backtrace(frames, MAX_FRAMES);
	char **symbols = backtrace_symbols(frames, sz);
   log_info_type *log = get_kernel_log();
	for (int32_t i=0; i<sz; i++) {
		log_err(log, "%d\t%s", i, symbols[i]);
   }
	if (sz >= MAX_FRAMES) {
		log_err(log, "*** trace truncated at %d frames", MAX_FRAMES);
   }
	free(symbols);
}


void cb_fault(int x)
{
   fprintf(stderr, "cb_fault(%d)\n", x);
   fflush(stderr);
   log_info_type *log = get_kernel_log();
	if (x == SIGSEGV) {
		log_err(log, "*** SIGSEGV segmentation fault ***");
	} else if (x == SIGFPE) {
		log_err(log, "*** SIGFPE floating point exception ***");
	} else if (x == SIGILL) {
		log_err(log, "*** SIGILL illegal instruction exception ***");
	} else if (x == SIGBUS) {
		log_err(log, "*** SIGBUS bus error ***");
	}
	print_stack_trace();
	fflush(stderr);
	hard_exit(__func__, __LINE__);
}

////////////////////////////////////////////////////////////////////////

void flush_to_zero()
{
#if defined(INTEL)
   // TODO verify that ffast-math is used and that it sets FTZ
	_MM_SET_FLUSH_ZERO_MODE ( _MM_FLUSH_ZERO_ON);
#if defined(__SSE3__)
	_MM_SET_DENORMALS_ZERO_MODE(_MM_DENORMALS_ZERO_ON);
#endif // __SSE3__
#elif defined(RPI)
   __asm__ volatile (
      "FMRX r10,FPSCR\nORR r10,r10,#0x00800000\nFMXR FPSCR,r10\n"
      : : : "r10"
   );
#elif defined(RPI4)
   // pass
   // TODO verify that FTZ is set w/ fast-math on arm8, or find 
   //    assembler instructions to set it
#else
   // unexpected hardware -- treat this as an error so problem can
   //    be sorted out
#error "Unspecified/unsupported CPU"
#endif   // INTEL | RPI
}

