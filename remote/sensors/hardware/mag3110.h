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
#if !defined(MAG3110_H)
#define  MAG3110_H
#include "pin_types.h"
#include "../s2.h"

int32_t mag3110_setup(
      /* in out */       sensor_runtime_type *dev
      );

int32_t mag3110_check_whoami(
      /* in out */       sensor_runtime_type *dev
      );

void mag3110_update(
      /* in out */       sensor_runtime_type *dev,
      /*    out */       int32_t *data_available
      );

void mag3110_shutdown(
      /* in out */       sensor_runtime_type *dev
      );

#endif   // MAG3110_H
