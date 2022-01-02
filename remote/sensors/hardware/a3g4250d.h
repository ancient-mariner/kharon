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
