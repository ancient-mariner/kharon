/***********************************************************************
* This file is part of kharon <https://github.com/ancient-mariner/kharon>.
* Copyright (C) 2019-2022 Keith Godfrey
*
* kharon is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, version 3.
*
* kharon is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with kharon.  If not, see <http://www.gnu.org/licenses/>.
***********************************************************************/
#include "sens_db.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include "logger.h"
#include "dev_info.h"
#include "lin_alg.h"


void fetch_accel_config(
      /* in out */       sensor_runtime_type *dev
      )
{
   FILE *fp;
   fp = open_config_file_ro2(PI_DEV_ROOT, NULL, "sensors/i2c", dev->name,
         "modality/accel/offset");
   if (fp) {
      config_read_vector(fp, &dev->accel.offset);
      fclose(fp);
   } else {
      fprintf(stderr, "WARNING: modality/accel/offset not found\n");
      // accel is probably already 0.0 but make sure
      zero_vector(&dev->accel.offset);
   }
   //
   fp = open_config_file_ro2(PI_DEV_ROOT, NULL, "sensors/i2c", dev->name,
         "modality/accel/scale");
   if (fp) {
      config_read_vector(fp, &dev->accel.scale);
      fclose(fp);
   } else {
      fprintf(stderr, "WARNING: modality/accel/scale not found\n");
      // set scale to 1.0 so it at least works
      for (uint32_t i=0; i<3; i++) {
         dev->accel.scale.v[i] = 1.0f;
      }
   }
   //
   fp = open_config_file_ro2(PI_DEV_ROOT, NULL, "sensors/i2c", dev->name,
         "modality/accel/i2c_addr");
   if (fp) {
      config_read_byte(fp, &dev->accel.accel_addr);
      fclose(fp);
   } else {
      fprintf(stderr, "ERROR: modality/accel/i2c_addr not found\n");
      // no recovery here, but at least this is a hint as to the error
   }
}


void fetch_mag_config(
      /* in out */       sensor_runtime_type *dev
      )
{
   FILE *fp;
   fp = open_config_file_ro2(PI_DEV_ROOT, NULL, "sensors/i2c", dev->name,
         "modality/mag/offset");
   if (fp) {
      config_read_vector(fp, &dev->mag.offset);
      fclose(fp);
   } else {
      fprintf(stderr, "WARNING: modality/mag/offset not found\n");
      // offset is probably already zero, but make sure
      zero_vector(&dev->mag.offset);
   }
   //
   fp = open_config_file_ro2(PI_DEV_ROOT, NULL, "sensors/i2c", dev->name,
         "modality/mag/scale");
   if (fp) {
      config_read_vector(fp, &dev->mag.scale);
      fclose(fp);
   } else {
      fprintf(stderr, "WARNING: modality/mag/scale not found\n");
      // default to 1.0 so at least there's some output
      for (uint32_t i=0; i<3; i++) {
         dev->mag.scale.v[i] = 1.0f;
      }
   }
   //
   fp = open_config_file_ro2(PI_DEV_ROOT, NULL, "sensors/i2c", dev->name,
         "modality/mag/softiron");
   if (fp) {
      config_read_matrix(fp, &dev->mag.softiron);
      fclose(fp);
   } else {
      fprintf(stderr, "WARNING: modality/mag/softiron not found\n");
      // default to identity matrix so at least there's some output
      identity_matrix(&dev->mag.softiron);
   }
   //
   fp = open_config_file_ro2(PI_DEV_ROOT, NULL, "sensors/i2c", dev->name,
         "modality/mag/i2c_addr");
   if (fp) {
      config_read_byte(fp, &dev->mag.mag_addr);
      fclose(fp);
   } else {
      fprintf(stderr, "ERROR: modality/mag/i2c_addr not found\n");
      // no recovery here, but at least this is a hint as to the error
   }
}


void fetch_gyro_config(
      /* in out */       sensor_runtime_type *dev
      )
{
   FILE *fp;
   fp = open_config_file_ro2(PI_DEV_ROOT, NULL, "sensors/i2c", dev->name,
         "modality/gyro/drift_dps");
   if (fp) {
      config_read_vector(fp, &dev->gyro.drift_dps);
      fclose(fp);
   } else {
      fprintf(stderr, "WARNING: modality/gyro/drift_dps not found\n");
      // dps array should already be zero, but make sure
      zero_vector(&dev->gyro.drift_dps);
   }
   //
   fp = open_config_file_ro2(PI_DEV_ROOT, NULL, "sensors/i2c", dev->name,
         "modality/gyro/i2c_addr");
   if (fp) {
      config_read_byte(fp, &dev->gyro.gyro_addr);
      fclose(fp);
   } else {
      fprintf(stderr, "ERROR: modality/gyro/i2c_addr not found\n");
      // no recovery here, but at least this is a hint as to the error
   }
}


void fetch_temp_config(
      /* in out */       sensor_runtime_type *dev
      )
{
//   read_config_val_1b(dev->config_root, "/modality/temp/i2c_addr",
//         &dev->temp.temp_addr);
   FILE *fp;
   fp = open_config_file_ro2(PI_DEV_ROOT, NULL, "sensors/i2c", dev->name,
         "modality/temp/i2c_addr");
   if (fp) {
      config_read_byte(fp, &dev->temp.temp_addr);
      fclose(fp);
   } else {
      fprintf(stderr, "ERROR: modality/temp/i2c_addr not found\n");
      // no recovery here, but at least this is a hint as to the error
   }
}

////////////////////////////////////////////////////////////////////////

void write_gyro_drift(
      /* in     */ const sensor_runtime_type *dev
      )
{
//   write_config_val_1v(dev->config_root, "/modality/gyro/drift_dps",
//         &dev->gyro.drift_dps);
   config_write_vector2(PI_DEV_ROOT, NULL, "sensors/i2c", dev->name,
         "modality/gyro/drift_dps", &dev->gyro.drift_dps);
}

////////////////////////////////////////////////////////////////////////

// resolve endpoint target of i2c devices for this host
// endpoint target file stores hostname and endpoint name of target
//    eg, "ghost imu_1"
// the endpoint target dev/<host>/endpoints/<name> stores the port number
// the file dev/<host>/ip_addr stores the IP address
int32_t resolve_endpoint_for_i2c(
      /*    out */       network_id_type *id
      )
{
   return resolve_sensor_endpoint("i2c_endpoint", id);
}

