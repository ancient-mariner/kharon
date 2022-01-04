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

#include "a3g4250d.h"
#include "a3g4250d_registers.h"

// FIFO on, acq at 200Hz, high-pass filter at 0.02Hz

// This has static variables. One s2 process can only support one instance
//    of this sensor

#define HARDWARE_NAME   HARDWARE_NAME_A3G4250D

#define UPDATE_INTERVAL_US    12500

#define WARMUP_INTERVAL    WARMUP_INTERVAL_BASE

//
static int16_t last_[3] = { 0, 0, 0 };
static int32_t stream_mode_ = 0;

static int32_t write_register(
      /* in out */       sensor_runtime_type *dev,
      /* in     */ const uint8_t reg,
      /* in     */ const uint8_t value
      )
{
   if (i2c_smbus_write_byte_data(dev->hw_device, reg, value) < 0) {
      fprintf(stderr, "%s: write_register() error. reg=%d, value=%d\n",
            __FILE__, reg, value);
      device_error(dev, __FILE__, __LINE__, reg, dev->flags);
      return -1;
   }
   return 0;
}

////////////////////////////////////////////////////////////////////////

static void set_bypass_mode(
      /* in out */       sensor_runtime_type *dev
      )
{
   // switch to bypass mode. watermark at 1
   write_register(dev, FIFO_CTRL_REG, 0b00000001);
   stream_mode_ = 0;
}

static void set_stream_mode(
      /* in out */       sensor_runtime_type *dev
      )
{
   // switch to stream mode. watermark at 1
   write_register(dev, FIFO_CTRL_REG, 0b01000001);
   stream_mode_ = 1;
}

static int32_t read_gyr_data(
      /* in out */       sensor_runtime_type *dev
      )
{
   // if this procedure is called then there's data available
   // device stream/fifo read protocol
   // if watermark flag set
   //    get size of fifo
   //    do a dummy read from 0x28 (increment bit 0)
   //    do a burst read (6 bytes) from 0x2a (Y low) to 29h (Y,Z,X)
   uint8_t cmd;
   uint8_t raw[6];
   int16_t data[3];
   int32_t accum[3] = { 0, 0, 0 };
   sensor_gyro_type *gyro = &dev->gyro;
   const int hw = dev->hw_device;
   uint8_t n, n_samples, status;
   uint32_t cnt = 0;
   /////////////////////////////////////////////////////////////////////
   if (stream_mode_ == 0) {
      set_stream_mode(dev);
      goto no_data;
   }
   /////////////////////////////////////////////////////////////////////
   // see if data is available -- use watermark bit
   cmd = 0x80 | FIFO_SRC_REG;
   if (i2c_smbus_read_i2c_block_data(hw, cmd, 1, &status) < 0) {
      device_error(dev, __FILE__, __LINE__, cmd, SENSOR_FLAG_GYRO);
      goto no_data;
   }
   if ((status & 0x80) == 0) {
      goto no_data; // watermark bit not set
   }
   // read FIFO queue
   n_samples = status & 0x1f;  // lower 5 bits is where count resides
   n = n_samples;
   do {
      // dummy read
      cmd = OUT_X_L; // increment bit = 0
      if (i2c_smbus_read_i2c_block_data(hw, cmd, 2, raw) < 0) {
         device_error(dev, __FILE__, __LINE__, cmd, SENSOR_FLAG_GYRO);
         goto no_data;
      }
      // burst read: Y, Z, X
//      cmd = 0x80 | OUT_Y_L;
//      if (i2c_smbus_read_i2c_block_data(hw, cmd, 2, &raw[2]) < 0) {
//         device_error(dev, __FILE__, __LINE__, cmd, SENSOR_FLAG_GYRO);
//         goto no_data;
//      }
//      cmd = 0x80 | OUT_Z_L;
//      if (i2c_smbus_read_i2c_block_data(hw, cmd, 2, &raw[4]) < 0) {
//         device_error(dev, __FILE__, __LINE__, cmd, SENSOR_FLAG_GYRO);
//         goto no_data;
//      }
      cmd = 0x80 | OUT_Y_L;
      if (i2c_smbus_read_i2c_block_data(hw, cmd, 4, &raw[2]) < 0) {
         device_error(dev, __FILE__, __LINE__, cmd, SENSOR_FLAG_GYRO);
         goto no_data;
      }
//      cmd = 0x80 | OUT_Z_L;
//      if (i2c_smbus_read_i2c_block_data(hw, cmd, 2, &raw[4]) < 0) {
//         device_error(dev, __FILE__, __LINE__, cmd, SENSOR_FLAG_GYRO);
//         goto no_data;
//      }
      cmd = 0x80 | OUT_X_L;
      if (i2c_smbus_read_i2c_block_data(hw, cmd, 2, &raw[0]) < 0) {
         device_error(dev, __FILE__, __LINE__, cmd, SENSOR_FLAG_GYRO);
         goto no_data;
      }
      data[0] = (int16_t) (raw[0] | raw[1] << 8);
      data[1] = (int16_t) (raw[2] | raw[3] << 8);
      data[2] = (int16_t) (raw[4] | raw[5] << 8);
      // sometimes the first sample is the same as the last sample on the
      //    previous pass. if so, drop it
      // put another way, keep the sample if it's not the first sample or
      //    if any of the sample values are different
      if ((n != n_samples) || (data[0] != last_[0]) ||
            (data[1] != last_[1]) || (data[2] != last_[2])) {
      // acc ordered X, Y, Z little endian
         accum[0] += data[0];
         accum[1] += data[1];
         accum[2] += data[2];
//printf("%2d  %4d,%4d,%4d    %7.3f,%7.3f,%7.3f\n", n,
//      data[0], data[1], data[2],
//      SIG_GAIN*data[0], SIG_GAIN*data[1], SIG_GAIN*data[2]);
         cnt++;
      } else {
//printf("  SAME (skip)\n");
      }
      // decrement
      n--;
      // if n < 0 then break (if n<0 for uint means is > than max)
   } while (n < 0x1f);
   if (cnt == 0) {
      // this is an exceptional case -- only one sample (shouldn't be
      //    possible) and it matched the previous last sample. put
      //    invalid data in last so we don't accidentally trigger again
      last_[0] = (int16_t) 0x8001;
      last_[1] = (int16_t) 0x7fff;
      last_[2] = (int16_t) 0x8001;
      goto no_data;
   }
   last_[0] = data[0];
   last_[1] = data[1];
   last_[2] = data[2];
   // apply gain and subtract drift
   const double dt = (double) cnt * UPDATE_INTERVAL_US * 1000;
   for (uint32_t i=0; i<3; i++) {
      // make average of signal (DPS) and return that
      double dtheta = (double) accum[i] / ((double) cnt);
      gyro->axis_dps.v[i] = dtheta * gyro->gain.v[i] -
            gyro->drift_dps.v[i] * dt;
   }
   return 0;
no_data:
   return -1;
}

static int32_t read_temp_data(
      /* in out */       sensor_runtime_type *dev
      )
{
   uint8_t temp;
   int hw = dev->hw_device;
   uint8_t cmd = 0x80 | OUT_TEMP;
   if (i2c_smbus_read_i2c_block_data(hw, cmd, 1, &temp) < 0) {
      device_error(dev, __FILE__, __LINE__, cmd, SENSOR_FLAG_TEMP);
      return -1;
   }
//printf("TEMP  %0x02x (%d)\n", temp);
   dev->temp.celcius = (double) (temp);
   return 0;
}

////////////////////////////////////////////////////////////////////////
//

static void initialize_device(
      /* in out */       sensor_runtime_type *device
      )
{
   // this sensor has a single device. set it here and forget about it
   select_device(device, device->gyro.gyro_addr);
   // 200Hz, 25Hz cutoff, xyz enabled
   write_register(device, CTRL_REG1, 0b01011111);
   // high-pass filter set for 0.02Hz
   write_register(device, CTRL_REG2, 0b00101001);
   // CTRL_REG3 no interrupts (default)
   // CTRL_REG4 little endian, no self test (default)
   // FIFO enabled, high-pass filter enabled
   write_register(device, CTRL_REG5, 0b01010001);
   // start in bypass mode. set flag and revert to stream mode
   //    on next read
   set_bypass_mode(device);
   //
   sensor_gyro_type *gyr = &device->gyro;
   const double gyr_gain = 0.00875;
   for (uint32_t i=0; i<3; i++) {
      gyr->gain.v[i] = gyr_gain;
   }
   device->temp.gain = 1.0;
}


////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
// External facing code

int32_t a3g4250d_setup(
      /* in out */       sensor_runtime_type *dev
      )
{
   // e.g., check apply gain (not updated since change to US); see if
   //    sensor is viable
   fprintf(stderr, "Logic needs to be examined and commented. not ready for prime time\n");
   goto err;
   if (enable_device(dev, HARDWARE_NAME))
      goto err;
   // a3g4250d has gyro and temp
   init_sensor_gyro(&dev->gyro, NULL, 1.0);
   init_sensor_temp(&dev->temp, NULL, 1.0);
   //
   fetch_gyro_config(dev);
   fetch_temp_config(dev);
   //
   initialize_device(dev);
   // set flags last
   dev->flags |= SENSOR_FLAG_GYRO | SENSOR_FLAG_TEMP;
   //
   dev->waketime.tv_sec = 0;
   dev->waketime.tv_nsec = WARMUP_INTERVAL * TIME_ADD_1MS;
   dev->state = 0;
   return 0;
err:
   return -1;
}

int32_t a3g4250d_self_test(
      /* in out */       sensor_runtime_type *dev
      )
{
   uint8_t cmd = 0x80 | WHO_AM_I;
   int hw = dev->hw_device;
   uint8_t data;
   if (i2c_smbus_read_i2c_block_data(hw, cmd, 1, &data) < 0) {
      device_error(dev, __FILE__, __LINE__, cmd, SENSOR_FLAG_MAG);
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

void a3g4250d_update(
      /* in out */       sensor_runtime_type *dev,
      /*    out */       int32_t *data_available
      )
{
   if (read_gyr_data(dev) != 0) {
      *data_available = 1;
   } else {
      *data_available = 0;
   }
   read_temp_data(dev);
   // update timer
   timeadd(&dev->waketime, 0, UPDATE_INTERVAL_US * TIME_ADD_1US);
}

void a3g4250d_shutdown(
      /* in out */       sensor_runtime_type *dev
      )
{
   close(dev->hw_device);
   dev->hw_device = -1;
}

