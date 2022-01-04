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
#include "sens_net.h"
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include "pinet.h"
#include "logger.h"
#include "sensor_packet.h"
#include "timekeeper.h"

// average all acc/mag data and stores it in sensor packet
static void build_sensor_packet(
      /*    out */       imu_sensor_packet_type *pkt,
      /* in     */ const consensus_sensor_type *consensus
      )
{
   for (uint32_t i=0; i<3; i++)
      pkt->gyr.v[i] = consensus->gyr_axis.v[i];
   for (uint32_t i=0; i<3; i++)
      pkt->acc.v[i] = consensus->acc.v[i];
   for (uint32_t i=0; i<3; i++)
      pkt->mag.v[i] = consensus->mag.v[i];
   for (uint32_t i=0; i<3; i++)
      pkt->gps.v[i] = consensus->latlon.v[i];
   pkt->temp = consensus->temp;
   pkt->baro = consensus->baro;
   pkt->state = consensus->state;
}

int32_t send_broadcast_timestamp(
      /* in     */ const int sockfd,
      /* in     */ const double when,
      /* in     */ const consensus_sensor_type *consensus
      )
{
   int32_t rc = 0;
   char serial[SP_SERIAL_LENGTH];
   // consolidate all sensor data into broadcast packet
   imu_sensor_packet_type pkt;
   build_sensor_packet(&pkt, consensus);
   // build packet header
   sensor_packet_header_type header;
   serialize_sensor_header(IMU_PACKET_TYPE, when, &header);
   // copy log_data into sensor header
   strcpy(header.log_data, consensus->log_data);
   if ((rc = send_block(sockfd, &header, sizeof(header))) < 0) {
      fprintf(stderr, "Socket write error. Shutting down link\n");
      goto end;
   }
   // send packet
   serialize_sensor_packet(&pkt, serial);
   if ((rc = send_block(sockfd, &serial, SP_SERIAL_LENGTH)) < 0) {
      fprintf(stderr, "Socket write error. Shutting down link\n");
      goto end;
   }
end:
   return rc;
}

int32_t send_broadcast(
      /* in     */ const int sockfd,
      /* in     */ const consensus_sensor_type *consensus
      )
{
   // in rare situations, it's possible for the system clock to be
   //    adjusted backward more than the sampling interval, resulting
   //    in sequential packets being out-of-order in time (this has
   //    been observed)
   // to prevent out-of-order packets, or excessively compressed ones,
   //    keep track of when the previously packet was sent and, if
   //    the subsequent packet is not sufficiently in the future of
   //    the previous one (e.g., half sampling interval) then adjust
   //    the time of the second packet so that it has that delay
   // window is 1/2 the sampling interval
   static const double WINDOW = (double) SAMPLING_RATE_BASE_US * 0.5e-6;
   static double prev_when_ = 0.0;
   //
   double when = now();
   if (when < prev_when_ - WINDOW) {
      when = prev_when_ + WINDOW;
   }
   prev_when_ = when;
   return send_broadcast_timestamp(sockfd, when, consensus);
}

