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
#if !defined(POSTMASTER_H)
#define  POSTMASTER_H
#include "pinet.h"

// the postmaster exchanges control messages between the kernel and remote
//    processes (eg, a java control app)
// communication protocol is client connects, sends request, waits for
//    response, closes connection
// there is one postmaster process per app

void * launch_postmaster(void *arg);

enum postmaster_state { UNITIALIZED, RUNNING, DONE, ERROR };

enum postmaster_state get_postmaster_state(void);

//struct pm_request_header {
//};

// TODO stream data to disk when appropriate modules are active and data
//    is coming in. imu always active, so is always saved.
//    cam data is saved when frames are delivered (if don't want frames,
//    run without loading those modules)
// TODO stream IMU to flat file and import to database later. transactions
//    disrupt saving images as disk IO locks up w/ syncing
// TODO make sure cam command can over-ride previously set value


enum {
   PM_CMD_NULL,      // sends kernel time
   // core
//   PM_CMD_PING,            // see if bob's alive. returns kernel time
   PM_CMD_ANNOTATION,      // add string to kernel log
   PM_CMD_SHUTDOWN,        // shutdown bob
   // otto
   PM_CMD_AUTOPILOT_ON,    // turns auto-tracking on
   PM_CMD_AUTOPILOT_OFF,   // turns auto-tracking off
   // aim
   // set specific autopilot heading. implicit autopilot_off
   PM_CMD_SET_HEADING,
   // go
   PM_CMD_SET_DESTINATION,
   //       lon stored as BAM32 in custom0
   //       lat stored as BAM32 in custom1
   //       radius stored in custom2
   // module
   PM_CMD_MODULE_PAUSE,    // disables processing in module
   PM_CMD_MODULE_RESUME    // re-enables processing in module
};

struct pm_request {
   uint32_t request_type;
   // number of bytes of data that follow header
   //    (handled in type-dependent way)
   uint32_t header_bytes;  // should be called 'payload_bytes'
   // type-dependent values
   int32_t custom_0;
   int32_t custom_1;
   int32_t custom_2;
};
typedef struct pm_request pm_request_type;

struct pm_response {
   uint32_t request_type;  // returns request type, or PM_CMD_NULL if unknown
   // number of bytes of data that follow header
   uint32_t response_bytes;   // should be called 'payload_bytes'
   uint8_t t[32];    // server time that response was sent (text of float64)
   // type-dependent values
   int32_t custom_0;
   int32_t custom_1;
   int32_t custom_2;
};
typedef struct pm_response pm_response_type;

// returns port number used by postmaster
int16_t get_postmaster_port(void);

// sets postmaster quit flag and shuts down socket
void postmaster_quit(
      /* in     */ const int unused
      );

#endif  // POSTMASTER_H
