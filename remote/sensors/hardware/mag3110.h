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
