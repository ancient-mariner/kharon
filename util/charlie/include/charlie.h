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
#if !defined(CHARLIE_H)
#define CHARLIE_H
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include "pinet.h"
#include "postmaster.h"
#include "dev_info.h"

// name of environment variable storing host where postmaster resides
#define CHARLIE_HOST_STR      "CHARLIE_HOST"

#define KHARON_DEVICE_DIR     "/opt/kharon/data/dev/"

////////////////////////////////////////////////////////////////////////
// destination database

#define CHARLIE_DESTINATION_DB      "/opt/kharon/charlie/destinations.txt"
// format of destination DB is
//    <name> <longitude> <latitude> <radius>
// where 'radius' is how close we need to get to the target coordinates
//    (in meters) before declaring success. NOTE this shouldn't be
//    less than ~100m right now as terminal navigation in final map grid
//    is not implemented
// lines starting with '#' and empty lines are ignored

#define CHARLIE_DESTINATION_DB_NAME_LEN_MAX     64

struct dest_record {
   char name[CHARLIE_DESTINATION_DB_NAME_LEN_MAX];
   double latitude;
   double longitude;
   meter_type radius;
   uint32_t line_num;
};
typedef struct dest_record dest_record_type;

// destination database
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
// postmaster API

int get_postmaster_address(
      /*    out */       network_id_type *id
      );

int send_postmaster_request(
      /* in     */ const int sockfd,
      /* in     */ const struct pm_request *req,
      /* in     */ const void *data
      );

int read_postmaster_response(
      /* in     */ const int sockfd,
      /* in     */ const struct pm_request *req,
      /*    out */       struct pm_response *resp,
      /*    out */       uint8_t ** data
      );

#endif   // CHARLIE_H
