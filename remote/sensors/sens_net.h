#if !defined(SENS_NET_H)
#define SENS_NET_H
#include "pin_types.h"
#include "s2.h"

// send sensor packet to brain, using specified timestamp
// returns 0 on success, -1 on error
int32_t send_broadcast_timestamp(
      /* in     */ const int sockfd,
      /* in     */ const double timestamp,
      /* in     */ const consensus_sensor_type *consensus
      );

// send sensor packet to brain
// this is a wrapper for send_broadcast_timestamp, using t=now()
// returns 0 on success, -1 on error
int32_t send_broadcast(
      /* in     */ const int sockfd,
      /* in     */ const consensus_sensor_type *consensus
      );

#endif   // SENS_NET_H

