#if !defined(HMC6343_H)
#define  HMC6343_H
#include "pin_types.h"
#include "../s2.h"

int32_t hmc6343_setup(
      /* in out */       sensor_runtime_type *dev
      );

void hmc6343_update(
      /* in out */       sensor_runtime_type *dev,
      /*    out */       int32_t *data_available
      );

void hmc6343_shutdown(
      /* in out */       sensor_runtime_type *dev
      );

#endif   // HMC6343_H
