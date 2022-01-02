#if !defined(BMG160_H)
#define  BMG160_H
#include "pin_types.h"
#include "../s2.h"

int32_t bmg160_setup(
      /* in out */       sensor_runtime_type *dev
      );

void bmg160_update(
      /* in out */       sensor_runtime_type *dev,
      /*    out */       int32_t *data_available
      );

void bmg160_shutdown(
      /* in out */       sensor_runtime_type *dev
      );

#endif   // BMG160_H
