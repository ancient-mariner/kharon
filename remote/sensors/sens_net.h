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

