#if !defined(SENS_DB_H)
#define  SENS_DB_H
#include "s2.h"


void fetch_accel_config(
      /* in out */       sensor_runtime_type *dev
      );


void fetch_mag_config(
      /* in out */       sensor_runtime_type *dev
      );


void fetch_gyro_config(
      /* in out */       sensor_runtime_type *dev
      );


void fetch_temp_config(
      /* in out */       sensor_runtime_type *dev
      );

////////////////////////////////////////////////////////////////////////

void write_gyro_drift(
      /* in     */ const sensor_runtime_type *dev
      );

////////////////////////////////////////////////////////////////////////

int32_t resolve_endpoint_for_i2c(
      /*    out */       network_id_type *id
      );

#endif   // SENS_DB_H
