#if !defined(LIS3MDL_H)
#define  LIS3MDL_H
#include "pin_types.h"
#include "../s2.h"

int32_t lis3mdl_setup(
      /* in out */       sensor_runtime_type *dev
      );

int32_t lis3mdl_check_whoami(
      /* in out */       sensor_runtime_type *dev
      );

void lis3mdl_update(
      /* in out */       sensor_runtime_type *dev,
      /*    out */       int32_t *data_available
      );

void lis3mdl_shutdown(
      /* in out */       sensor_runtime_type *dev
      );

#endif   // LIS3MDL_H
