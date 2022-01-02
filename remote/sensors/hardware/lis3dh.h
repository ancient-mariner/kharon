#if !defined(LIS3DH_H)
#define  LIS3DH_H
#include "pin_types.h"
#include "../s2.h"

int32_t lis3dh_setup(
      /* in out */       sensor_runtime_type *dev
      );

int32_t lis3dh_check_whoami(
      /* in out */       sensor_runtime_type *dev
      );

void lis3dh_update(
      /* in out */       sensor_runtime_type *dev,
      /*    out */       int32_t *data_available
      );

void lis3dh_shutdown(
      /* in out */       sensor_runtime_type *dev
      );

#endif   // LIS3DH_H
