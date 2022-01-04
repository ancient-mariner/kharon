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
#if !defined(SENS_DB_H)
#define  SENS_DB_H
#include "s2.h"


void fetch_accel_config(
      /* in out */       sensor_runtime_type *dev
      );


void fetch_mag_config(
      /* in out */       sensor_runtime_type *dev
      );


void fetch_gyro_config(
      /* in out */       sensor_runtime_type *dev
      );


void fetch_temp_config(
      /* in out */       sensor_runtime_type *dev
      );

////////////////////////////////////////////////////////////////////////

void write_gyro_drift(
      /* in     */ const sensor_runtime_type *dev
      );

////////////////////////////////////////////////////////////////////////

int32_t resolve_endpoint_for_i2c(
      /*    out */       network_id_type *id
      );

#endif   // SENS_DB_H
