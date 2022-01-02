#if !defined(L3G_H)
#define  L3G_H
#include "pin_types.h"
#include "../s2.h"

int32_t l3g_setup(
      /* in out */       sensor_runtime_type *dev
      );

void l3g_update(
      /* in out */       sensor_runtime_type *dev,
      /*    out */       int32_t *data_available
      );

void l3g_shutdown(
      /* in out */       sensor_runtime_type *dev
      );

#endif   // L3G_H
