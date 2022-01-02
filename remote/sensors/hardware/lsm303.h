#if !defined(LSM303_H)
#define  LSM303_H
#include "pin_types.h"
#include "../s2.h"

int32_t lsm303_setup(
      /* in out */       sensor_runtime_type *dev
      );

void lsm303_update(
      /* in out */       sensor_runtime_type *dev,
      /*    out */       int32_t *data_available
      );

void lsm303_shutdown(
      /* in out */       sensor_runtime_type *dev
      );

#endif   // LSM303_H
