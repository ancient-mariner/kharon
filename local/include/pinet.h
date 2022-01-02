#if !defined(PINET_H)
#define PINET_H
#if !defined(_GNU_SOURCE)
#define _GNU_SOURCE
#endif   // _GNU_SOURCE
#include <math.h>
#include <time.h>
#include "pin_types.h"

// TODO make this configurable. the obvious way to do this is to set it
//    in the lua config file, but that file isn't loaded until after log
//    data is already being written. for now, hardcode the value.
//    perhaps this can be a value specified when compiling, as the repo
//    shouldn't move after it's installed, for most realistic use cases
#define LOG_DIRECTORY_ROOT    "/data/kharon/"

// when running on RPI, the dev directory is hard-coded. this is so
//    tasks requiring dev info know where to look and don't have to
//    be told
#define PI_DEV_ROOT     "/opt/kharon/data/dev/"

////////////////////////////////////////////////////////////////////////

#if defined(NDEBUG)
#define log_malloc(mem, size)
#define log_calloc(mem, n, size)
#elif defined(INTEL)
#define log_malloc(mem, size) \
      { log_info(log_, "MEM 0x%lx  %d bytes %s:%d",   \
            (uint64_t) mem, size, __FILE__, __LINE__); }
#define log_calloc(mem, n, size) \
      { log_info(log_, "MEM 0x%lx  %d x %d bytes %s:%d",   \
            (uint64_t) mem, n, size, __FILE__, __LINE__); }
#else
#define log_malloc(mem, size)
#define log_calloc(mem, n, size)
#endif   // NDEBUG

// TODO implement assertion handling mechanism like JPL (see c coding
//    guidelines) to gracefully handle error conditions

struct message_board {
   int acquiring;  // 1 for acquiring data, 0 for not
   int paused;    // 1 if data stream temporarily paused
   int exit;
};
extern struct message_board MESSAGE_BOARD;

// constants for device-receiver handshake
#define IMU_STREAM_ID   (0x31420001)
#define VY_STREAM_ID   (0x31420004)

// constants for remote producer mirroring
#define ASSOCIATOR_XFER_STREAM_ID   (0x43210001)

// server returns OK if it can handle specified input stream
#define HANDSHAKE_OK (0x28180000)
#define HANDSHAKE_ERROR (0xffff0000)

// creates sensor packet header structure
void serialize_sensor_header(
      /* in     */ uint32_t type,
      /* in     */ double timestamp,
      /*    out */ struct sensor_packet_header *pkt);

void serialize_sensor_header2(
      /* in     */ uint32_t type,
      /* in     */ double timestamp,
      /* in     */ double timestamp_2,
      /*    out */ struct sensor_packet_header *pkt);

// extracts data from sensor packet header
void unpack_sensor_header(
      /* in     */ struct sensor_packet_header *pkt,
      /*    out */ uint32_t *type,
      /*    out */ double *timestamp);

// extracts data from sensor packet header
void unpack_sensor_header2(
      /* in     */ struct sensor_packet_header *pkt,
      /*    out */ uint32_t *type,
      /*    out */ double *request_time,
      /*    out */ double *received_time
      );

// set all fields in sensor packet to 0
void clear_imu_sensor_packet(
      /* in out */        imu_sensor_packet_type *pkt
      );

const char * pinet_lib_version(void);

// takes source name (eg, 'rvid-cam') and provides connection info 
//    (IP + port) of receiving process
// returns 0 on success, -1 on failure
int get_network_target(const char *source, struct network_id *target);

// takes receiver name and provides connection info
// returns 0 on success, -1 on failure
int get_host_info(const char *receiver, struct network_id *host);


// all network connections should begin by sending communication protocol
//    version numbers, to ensure that all objects compiled against the
//    same code base, at least as far as what's expected in network
//    communications. whenever a protocol changes among any two network
//    nodes, the version numbers need to be altered
// discrepancies between version numbers should not be tolerated in any
//    circumstance
// server end of socket should respond with 1 if version matches and -1
//    if not. upon receiving -1, client should close connection
// all numbers here should be sent as signed int32, network byte order
#define COMMUNICATION_PROTOCOL_MAJOR   (3)
#define COMMUNICATION_PROTOCOL_MINOR   (2)
// 3.1 extend header to store information to be logged
// 3.2 IMU protocol changed to use 20 bits for sending floating points, from
//    16; data sent to indicate when IMU channel offline; GPS added
//   

#define GPS_BLOCK_SIZE     256

// multi-cast addressing, for time sync and synchronizing cameras
#define UDP_SYNC_PORT   6800
#define UDP_SYNC_DOMAIN   "192.168.1.255"

// wrappers for common connection establishment code
// returns socket file descriptor, or -1 on error
int init_server(
      /* in     */ const int16_t port
      );

int init_server_backlog(
      /* in     */ const int16_t port,
      /* in     */ const int16_t backlog
      );
// obsoleted version (jan-19)
//int init_server(int16_t port);
//int init_server_backlog(int16_t port, int16_t backlog);

// returns connection file descriptor, -1 on error, -2 on interrupt
int wait_for_connection(int sockfd, const char* name);

// returns connection file descriptor, -1 on error, -2 on interrupt
// does not check 
int wait_for_connection_no_check(int sockfd, const char* name);

// returns connection file descriptor, or -1 on error
int connect_to_server(
      /* in     */ const network_id_type *net_id
      );

// similar to connect_to_server() but returns immediately if server
//    is not available
// check of communication protocol version is optional
int connect_to_server_no_wait(
      /* in     */ const network_id_type *net_id,
      /* in     */ const int check_protocol  // 1 for yes, 0 for no
      );


// when establishing a connection, server checks clietnt's communication
//    protocol version to make sure it is supported
// client sends protocol version, server checks it
// each call returns 0 on success (ie, protocol numbers match) 
//    and -1 otherwise
// dt is the time difference of the client relative to the server
//    (in seconds, measured from clock_gettime)
int send_protocol_version(int sockfd, uint32_t major, uint32_t minor);
int check_protocol_version(int sockfd, uint32_t major, uint32_t minor);

// procedures return number of bytes sent/received, or -1 on error
int send_block(int sockfd, const void *data, uint32_t len);
int recv_block(int sockfd, void *data, uint32_t len);

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
//

// print stack trace to stderr
void print_stack_trace(void);

// call-back for handling sigsegv, sigfpe, sigill and sigbus
//    (ie, hard faults that are delivered thread-specifically)
// prints stack trace to stderr and calls exit(1)
void cb_fault(int x);

// calls _exit after printing caller to stderr
void hard_exit(const char *caller, int code);
void hard_exit2(const char *file, const char *func, int code);
#define HARD_EXIT(x)  hard_exit2(__FILE__, __func__, x)

////////////////////////////////////////////////////////////////////////

// flush denormals to zero
void flush_to_zero(void);

////////////////////////////////////////////////////////////////////////
// misc
void print_hex(uint8_t *buf, int n_bytes);

// trims leading and trailing white space. if entire string is WS, returns
//    NULL. otherwise returns pointer to first non-WS char
char *trim_whitespace(char *str);
char *trim_leading_whitespace(char *str);

#endif   // PINET_H
