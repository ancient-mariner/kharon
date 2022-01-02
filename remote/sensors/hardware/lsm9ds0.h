#if !defined(LSM9DS0_H)
#define  LSM9DS0_H
#include "pin_types.h"
#include "../s2.h"

int32_t lsm9ds0_setup(
      /* in out */       sensor_runtime_type *dev
      );

void lsm9ds0_update(
      /* in out */       sensor_runtime_type *dev,
      /*    out */       int32_t *data_available
      );

void lsm9ds0_shutdown(
      /* in out */       sensor_runtime_type *dev
      );

#endif   // LSM9DS0_H
