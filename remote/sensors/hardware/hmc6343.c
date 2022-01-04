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
#include "s2.h"
#include <stdio.h>
#include <stdint.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <errno.h>
#include <unistd.h>
#include "pinet.h"
#include "lin_alg.h"
#include "time_lib.h"
#include "sens_lib.h"
#include "sens_db.h"

#include "timekeeper.h"

#include "hmc6343.h"
#include "hmc6343_registers.h"

#define HARDWARE_NAME   HARDWARE_NAME_HMC6343

// This sensor should be run alongside a gyro or other sensor
//    with a higher sampling rate. It's internal rate is too low
#define UPDATE_INTERVAL_US    (4 * SAMPLING_RATE_BASE_US)   // -> 50ms

// there's a 5ms lag between reqeusting data and getting it
// data asked for twice (once for acc, 2nd for mag)
// there's also a 5ms lag between getting acc and asking for mag
#define WARMUP_INTERVAL       (WARMUP_INTERVAL_BASE - 15)

#if WARMUP_INTERVAL <= 500
#error "HMC 6343 requires at least 500ms to initialize"
#endif   // WARMUP_INTERVAL


static int32_t write_register(
      /* in out */       sensor_runtime_type *dev,
      /* in     */ const uint8_t reg,
      /* in     */ const uint8_t value
      )
{
   select_device(dev, dev->accel.accel_addr);
   if (i2c_smbus_write_byte_data(dev->hw_device, reg, value) < 0) {
      fprintf(stderr, "%s: write_register() error. reg=%d, value=%d\n",
            __FILE__, reg, value);
      device_error(dev, __FILE__, __LINE__, reg, dev->flags);
      return -1;
   }
   return 0;
}

////////////////////////////////////////////////////////////////////////

static int32_t read_acc_data(
      /* in out */       sensor_runtime_type *dev
      )
{
   uint8_t raw[6];
   int16_t data[3];
   uint8_t cmd = 0x33;
   select_device(dev, dev->accel.accel_addr);
   int hw = dev->hw_device;
   if (i2c_smbus_read_i2c_block_data(hw, cmd, sizeof(raw), raw) < 0) {
      device_error(dev, __FILE__, __LINE__, cmd, SENSOR_FLAG_ACC);
      return -1;
   }
   // acc ordered X, Y, Z big endian
   data[0] = (int16_t) (raw[1] | raw[0] << 8);
   data[1] = (int16_t) (raw[3] | raw[2] << 8);
   // left-hand system. invert Z to make right hand
   data[2] = (int16_t) (-(raw[5] | raw[4] << 8));
   //
   apply_gain(data, &dev->accel.gain, &dev->accel.up);
//   apply_gain_scale_offset(data, &dev->accel.gain,
//         &dev->accel.scale, &dev->accel.offset, &dev->accel.up);
   return 0;
}


static int32_t read_mag_data(
      /* in out */       sensor_runtime_type *dev
      )
{
   uint8_t raw[6];
   int16_t data[3];
   uint8_t cmd = 0x33;
   select_device(dev, dev->mag.mag_addr);
   int hw = dev->hw_device;
   if (i2c_smbus_read_i2c_block_data(hw, cmd, sizeof(raw), raw) < 0) {
      device_error(dev, __FILE__, __LINE__, cmd, SENSOR_FLAG_MAG);
      return -1;
   }
   // mag ordered X, Y, Z big endian
   // inverting signal to match polarity of existing sensors
   data[0] = (int16_t) (-(raw[1] | raw[0] << 8));
   data[1] = (int16_t) (-(raw[3] | raw[2] << 8));
   data[2] = (int16_t) (-(raw[5] | raw[4] << 8));
   //
   apply_gain(data, &dev->mag.gain, &dev->mag.mag);
//   apply_gain_scale_offset(data, &dev->mag.gain,
//         &dev->mag.scale, &dev->mag.offset, &dev->mag.mag);
   return 0;
}


////////////////////////////////////////////////////////////////////////
//


static void initialize_device(
      /* in out */       sensor_runtime_type *device
      )
{
   // 10Hz
   // TODO investigate -- this doesn't seem to change freq
   //    (still streams data at 5hz)
   write_register(device, OP_MODE2, 0b00000010);
   //
   sensor_accel_type *acc = &device->accel;
   sensor_mag_type *mag = &device->mag;
   const double acc_gain = 1.0 / 1044.0; // estimated
   const double mag_gain = 1.0 / 3100.0; // estimated
   for (uint32_t i=0; i<3; i++) {
      acc->gain.v[i] = acc_gain;
      mag->gain.v[i] = mag_gain;
   }
}


////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
// External facing code

int32_t hmc6343_setup(
      /* in out */       sensor_runtime_type *dev
      )
{
   if (enable_device(dev, HARDWARE_NAME))
      goto err;
   // hmc6343 has accel, mag and temp
   init_sensor_accel(&dev->accel, NULL, 1.0);
   init_sensor_mag(&dev->mag, NULL, 1.0);
   //
   fetch_accel_config(dev);

   fetch_mag_config(dev);
   //
   initialize_device(dev);
   // run in default mode, so no initializtion necessary
   // set flags last -- only if initialization successful
   dev->flags |= SENSOR_FLAG_ACC | SENSOR_FLAG_MAG;
   // set initial delay. needs 500ms startup time
   dev->waketime.tv_sec = 0;
   dev->waketime.tv_nsec = WARMUP_INTERVAL * TIME_ADD_1MS;
   dev->state = 0;
   return 0;
err:
   return -1;
}


void hmc6343_update(
      /* in out */       sensor_runtime_type *dev,
      /*    out */       int32_t *data_available
      )
{
   // 2 states
   //    0  send request for attitude, wait 5ms
   //    1  read acc, wait 5
   //    2  request mag, wait 5
   //    3  read mag, wait for next update
   switch (dev->state) {
      case 0:
         dev->state = 1;
         timeadd(&dev->waketime, 0, 5 * TIME_ADD_1MS);
         *data_available = 0;
         // request attitude data
         if (write_register(dev, 0x32, ACC_XYZ_BIG) != 0)
            fprintf(stderr, "error requesting accelerometer\n");
         break;
      case 1:
         dev->state = 2;
         timeadd(&dev->waketime, 0, 5 * TIME_ADD_1MS);
         *data_available = 0;
         if (read_acc_data(dev) != 0)
            fprintf(stderr, "error reading accelerometer\n");
         break;
      case 2:
         dev->state = 3;
         timeadd(&dev->waketime, 0, 5 * TIME_ADD_1MS);
         *data_available = 0;
         // request mag data
         if (write_register(dev, 0x32, MAG_XYZ_BIG) != 0)
            fprintf(stderr, "error requesting magnetometer\n");
         break;
      case 3:
         dev->state = 0;
         timeadd(&dev->waketime, 0, (UPDATE_INTERVAL_US - 15) * TIME_ADD_1US);
         if (read_mag_data(dev) != 0)
            fprintf(stderr, "error reading magnetometer\n");
         *data_available = 1;
         break;
      default:
         fprintf(stderr, "Unknown state in hmc6343: %d\n", dev->state);
         hard_exit(__func__, 1);
   };
}

void hmc6343_shutdown(
      /* in out */       sensor_runtime_type *dev
      )
{
   // NOTE: the close here belongs wrapped in a function the level where the
   //    value was enabled
   close(dev->hw_device);
   dev->hw_device = -1;
}

