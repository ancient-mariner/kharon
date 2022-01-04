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
#include "pinet.h"
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <errno.h>
#include <unistd.h>
#include "sens_lib.h"
#include "sens_db.h"
#include "time_lib.h"
#include "timekeeper.h"

#include "mag3110.h"

#define HARDWARE_NAME HARDWARE_NAME_MAG3110

#define UPDATE_INTERVAL_US    SAMPLING_RATE_BASE_US

#define DT_SEC       (1.0e-6 * ((double) UPDATE_INTERVAL_US))
#define DRIFT_TIME_CONST_SEC  900.0
#define DRIFT_TAU    (1.0 / (DRIFT_TIME_CONST_SEC / DT_SEC))

#define WARMUP_INTERVAL       WARMUP_INTERVAL_BASE

////////////////////////////////////////////////////////////////////////
#define WHO_AM_I                    0x07
#define WHO_AM_I_VALUE              0xC4

#define DR_STATUS                   0x00
#define OUT_X_MSB                   0x01
#define SYS_MOD                     0x08

#define DIE_TEMP                    0x0F
#define CTRL_REG1                   0x10
#define CTRL_REG2                   0x11
////////////////////////////////////////////////////////////////////////

#define ADDR   0x0E

const double MAG_GAIN = 1.0e-5;
const double TEMP_GAIN = 1.0;


////////////////////////////////////////////////////////////////////////

static void write_register(
      /* in out */       sensor_runtime_type *dev,
      /* in     */ const uint8_t reg,
      /* in     */ const uint8_t value
      )
{
   select_device(dev, dev->mag.mag_addr);
   if (i2c_smbus_write_byte_data(dev->hw_device, reg, value) < 0) {
      device_error(dev, __FILE__, __LINE__, reg, SENSOR_FLAG_MAG);
   }
}

////////////////////////////////////////////////////////////////////////

int32_t mag3110_check_whoami(
      /* in out */       sensor_runtime_type *dev
      )
{
   // check WHO_AM_I
   uint8_t cmd = WHO_AM_I;
   uint8_t data;
   select_device(dev, dev->mag.mag_addr);
   if (i2c_smbus_read_i2c_block_data(dev->hw_device, cmd, 1, &data) < 0)
   {
      device_error(dev, __FILE__, __LINE__, cmd, SENSOR_FLAG_MAG);
      goto err;
   }
   if (data != WHO_AM_I_VALUE) {
      fprintf(stderr, "WHO_AM_I check failed. Expected 0x%02x, got 0x%02x\n",
            WHO_AM_I_VALUE, data);
      goto err;
   }
   return 0;
err:
   return -1;
}

////////////////////////////////////////////////////////////////////////

static uint8_t check_data_available(
      /* in out */       sensor_runtime_type *dev
      )
{
   uint8_t data;
   select_device(dev, dev->mag.mag_addr);
   if (i2c_smbus_read_i2c_block_data(dev->hw_device, DR_STATUS, 1, &data) < 0)
   {
      device_error(dev, __FILE__, __LINE__, DR_STATUS, SENSOR_FLAG_MAG);
   }
   return data;
}

static void read_mag_data(
      /* in out */       sensor_runtime_type *dev,
      /* in out */       int16_t data[3]
      )
{
   uint8_t raw[6];
   uint8_t cmd = OUT_X_MSB;
   if (i2c_smbus_read_i2c_block_data(dev->hw_device, cmd,
         sizeof(raw), raw) < 0) {
      device_error(dev, __FILE__, __LINE__, cmd, SENSOR_FLAG_MAG);
   }
   // mag data order X Y Z Temp, big endian (convert to little)
   data[0] = (int16_t) (raw[0] | raw[1] << 8);
   data[1] = (int16_t) (raw[2] | raw[3] << 8);
   data[2] = (int16_t) (raw[4] | raw[5] << 8);
}

static int32_t read_temp_data(
      /* in out */       sensor_runtime_type *dev
      )
{
   uint8_t temp;
   uint8_t cmd = 0x80 | DIE_TEMP;
   if (i2c_smbus_read_i2c_block_data(dev->hw_device, cmd, 1, &temp) < 0) {
      device_error(dev, __FILE__, __LINE__, cmd, SENSOR_FLAG_TEMP);
   }
   dev->temp.celcius = (double) temp * TEMP_GAIN;
   return 0;
}


////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

int32_t mag3110_setup(
      /* in out */       sensor_runtime_type *dev
   )
{
   if (enable_device(dev, HARDWARE_NAME)) {
      goto err;
   }
   /////////////////////////////////////////////////////////////////////
   init_sensor_mag(&dev->mag, NULL, 1.0);
   fetch_mag_config(dev);
   //
   mag3110_check_whoami(dev);
   /////////////////////////////////////////////////////////////////////
   // 20Hz, 16x oversample, normal mode, active
   write_register(dev, CTRL_REG1, 0b01000001);
   // no automatic correction, automatic reset
   write_register(dev, CTRL_REG2, 0b10100000);
   /////////////////////////////////////////////////////////////////////
   for (uint32_t i=0; i<3; i++) {
      dev->mag.gain.v[i] = MAG_GAIN;
   }
   dev->temp.gain = TEMP_GAIN;
   /////////////////////////////////////////////////////////////////////
   dev->flags = SENSOR_FLAGS_MAG_TEMP;
   dev->waketime.tv_sec = 0;
   dev->waketime.tv_nsec = WARMUP_INTERVAL * TIME_ADD_1MS;
   ////////////
   return 0;
err:
   return -1;
}

void mag3110_update(
      /* in out */       sensor_runtime_type *dev,
      /* in out */       int32_t *data_available
   )
{
   uint8_t avail = check_data_available(dev);
   if (avail & 0x08) {
      /////////////////////////
      int16_t data[3];
      read_mag_data(dev, data);
      apply_gain(data, &dev->mag.gain, &dev->mag.mag);
//for (uint32_t i=0; i<3; i++) {
//   printf(" 0x%04x %.3f %.3f -- ", data[i], (double) data[i] * MAG_GAIN, dev->mag.mag.v[i]);
//}
//printf("\n");
      /////////////////////////
      read_temp_data(dev);
      /////////////////////////
      // update timer
      timeadd(&dev->waketime, 0, UPDATE_INTERVAL_US * TIME_ADD_1US);
      *data_available = 1;
   } else {
      *data_available = 0;
   }
}

void mag3110_shutdown(
      /* in out */       sensor_runtime_type *dev
   )
{
   close(dev->hw_device);
   dev->hw_device = -1;
}

