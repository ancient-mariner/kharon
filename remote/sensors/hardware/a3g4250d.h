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
#if !defined(A3G4250D_H)
#define  A3G4250D_H
#include "pin_types.h"
#include "../s2.h"

int32_t a3g4250d_setup(
      /* in out */       sensor_runtime_type *dev
      );

int32_t a3g4250d_self_test(
      /* in out */       sensor_runtime_type *dev
      );

void a3g4250d_update(
      /* in out */       sensor_runtime_type *dev,
      /*    out */       int32_t *data_available
      );

void a3g4250d_shutdown(
      /* in out */       sensor_runtime_type *dev
      );

#endif   // A3G4250D_H
