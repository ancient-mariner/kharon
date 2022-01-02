#if !defined(PLAYA_H)
#define PLAYA_H
#if !defined _GNU_SOURCE
#define  _GNU_SOURCE
#endif  // _GNU_SOURCE
#include <signal.h>

#define RUNTIME_LOG_STDERR    "bob_err.txt"
#define RUNTIME_LOG_STDOUT    "bob_log.txt"

//struct message_board {
//   //int acquisition_state;  // 1 for writing data, 0 for not
//   int num_frames;
//   int acquiring;  // 1 for acquiring data, 0 for not
//   int paused;    // 1 if data stream temporarily paused
//};
//extern struct message_board MESSAGE_BOARD;

// signal usage
// from playa.c::init_signals
//    SIGINT 
//    SIGALARM
//       attempt graceful shutdown of system, through dp_abort, dp_wake
//    SIGSEGV 
//    SIGFPE 
//    SIGILL 
//    SIGBUS
//       print stack trace and hard exit
//    SIGRTMIN + 0   (= SIG_POSTMASTER_EXIT)
//       induces postmaster exit

//#define  SIG_POSTMASTER_EXIT  (SIGRTMIN + 0)

// debug/development code
// turn on (>0) or off (0) the streaming of data to disk
// (default is off)
void set_acquisition_state(uint32_t on_off);

// sets number of frames to capture
// -1 means unlimited
// 0 means frame acquisition off
// positve N means capture N (more) frames
// NOTE: this is the number of frames that are requested from each
//    camera. if a camera does not respond to a request, that frame
//    will be lost (ie, the count will be short)
//void set_number_frames(int n);

// callback for SIGINT to shut kernel down. also can be used internally
void signal_exit(int);

// returns name of active environment
const char * get_environment(void);

// used during initialization
void set_environment_string(
      /* in     */ const char *env
      );

// set/clear PAUSE flag in specified module
// returns -1 if module not found, 0 on success
int32_t module_onoff(
      /* in     */ const char *module_name,
      /* in     */ const int32_t on_off
      );

//////////////////////////////////////////////////////////////////////////
//// network services
//
//#define MAX_NETWORK_SERVICES  16
//
//struct service_info {
//   uint32_t   service_id;
//   int16_t    port_num;
//   int16_t    in_use;
//};
//typedef struct service_info service_info_type;
//
//// register an available network service. returns port number that will
////    be assigned to this service instance
//// port numbers are based on the postmaster's port number, starting one
////    after the postmaster. this means that the next MAX_NETWORK_SERVICES
////    ports must be free
//int16_t register_service(
//      /* in     */ const uint32_t service_id
//      );
//
//// claims endpoint when connection to client is established, so subsequent
////    clients can be forwarded to unused endpoint
//void claim_endpoint(
//      /* in     */ const int16_t port_num
//      );
//
//// returns unused port associated with this service ID
//int16_t lookup_endpoint(
//      /* in     */ const uint32_t service_id
//      );
//
//// when connection with client is broken, service needs to release port
////    so it's available for a new client
//void release_endpoint(
//      /* in     */ const int16_t port_num
//      );

#endif   // PLAYA_H

