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
#include <unistd.h>
#include "pinet.h"
#include "lin_alg.h"
#include "time_lib.h"
#include "sens_lib.h"
#include "sens_db.h"

#include "lsm303.h"
#include "lsm303_registers.h"

#define HARDWARE_NAME   HARDWARE_NAME_LSM303

#define UPDATE_INTERVAL_US    (2 * SAMPLING_RATE_BASE_US)  // -> 25ms
#define DT_SEC       (1.0e-6 * ((double) UPDATE_INTERVAL_US))

#define WARMUP_INTERVAL       WARMUP_INTERVAL_BASE

static void write_acc_register(
      /* in out */       sensor_runtime_type *dev,
      /* in     */ const uint8_t reg,
      /* in     */ const uint8_t value
      )
{
   select_device(dev, dev->accel.accel_addr);
   if (i2c_smbus_write_byte_data(dev->hw_device, reg, value) < 0) {
      device_error(dev, __FILE__, __LINE__, reg, SENSOR_FLAG_ACC);
   }
}

static void write_mag_register(
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

static void read_acc_data(
      /* in out */       sensor_runtime_type *dev,
      /*    out */       int16_t data[3]
      )
{
   uint8_t raw[6];
   uint8_t cmd = 0x80 | OUT_X_L_A;
   select_device(dev, dev->accel.accel_addr);
   int hw = dev->hw_device;
   if (i2c_smbus_read_i2c_block_data(hw, cmd, sizeof(raw), raw) < 0) {
      device_error(dev, __FILE__, __LINE__, cmd, SENSOR_FLAG_ACC);
   }
   // acc ordered X, Y, Z little endian
   data[0] = (int16_t) (raw[0] | raw[1] << 8);
   data[1] = (int16_t) (raw[2] | raw[3] << 8);
   data[2] = (int16_t) (raw[4] | raw[5] << 8);
}

static void read_mag_data(
      /* in out */       sensor_runtime_type *dev,
      /*    out */       int16_t data[3]
      )
{
   uint8_t raw[6];
   uint8_t cmd = 0x80 | OUT_X_H_M;
   select_device(dev, dev->mag.mag_addr);
   int hw = dev->hw_device;
   if (i2c_smbus_read_i2c_block_data(hw, cmd, sizeof(raw), raw) < 0) {
      device_error(dev, __FILE__, __LINE__, cmd, SENSOR_FLAG_MAG);
   }
   // mag data order X Z Y, big endian
   data[0] = (int16_t) (raw[1] | raw[0] << 8);
   data[2] = (int16_t) (raw[3] | raw[2] << 8);
   data[1] = (int16_t) (raw[5] | raw[4] << 8);
}


static void read_temp_data(
      /* in out */       sensor_runtime_type *dev,
      /*    out */       int16_t data[1]
      )
{
   uint8_t raw[2];
   uint8_t cmd = 0x80 | TEMP_OUT_H_M;
   select_device(dev, dev->temp.temp_addr);
   int hw = dev->hw_device;
   if (i2c_smbus_read_i2c_block_data(hw, cmd, sizeof(raw), raw) < 0) {
      device_error(dev, __FILE__, __LINE__, cmd, SENSOR_FLAG_TEMP);
   }
   // 12-bit big endian H: 0xff   L: 0xf0
   data[0] = (int16_t) ((raw[0] << 4) |
         ((raw[1] >> 4) & 0x000f));
}

////////////////////////////////////////////////////////////////////////
//

#define ACC_DATA_AVAILABLE_SIZE     1

static void acc_data_available(
      /* in     */       sensor_runtime_type *dev,
      /*    out */       uint8_t data[ACC_DATA_AVAILABLE_SIZE]
      )
{
   uint8_t cmd = 0x80 | STATUS_REG_A;
   select_device(dev, dev->accel.accel_addr);
   int hw = dev->hw_device;
   if (i2c_smbus_read_i2c_block_data(hw, cmd, ACC_DATA_AVAILABLE_SIZE,
         data) < 0)
   {
      device_error(dev, __FILE__, __LINE__, cmd, SENSOR_FLAG_ACC);
   }
}


static void pull_acc_data(
      /* in out */       sensor_runtime_type *dev
      )
{
   uint8_t avail[1];
   // fetch amount of data available
   acc_data_available(dev, avail);
   if (avail[0] & 0x08) {
//printf("ACC data available\n");
      // pull data
      int16_t data[3];
      read_acc_data(dev, data);
      apply_gain(data, &dev->accel.gain, &dev->accel.up);
//      apply_gain_scale_offset(data, &dev->accel.gain,
//            &dev->accel.scale, &dev->accel.offset, &dev->accel.up);
//print_vec(&dev->accel.up, "ACC data");
//printf("ACC: %d, %d, %d  \n", data[0], data[1], data[2]);
   }
}

////////////////////////////////////////////////////////////////////////
// mag

#define MAG_DATA_AVAILABLE_SIZE     1

static void mag_data_available(
      /* in     */       sensor_runtime_type *dev,
      /*    out */       uint8_t data[MAG_DATA_AVAILABLE_SIZE]
      )
{
   uint8_t cmd = 0x80 | SR_REG_M;
   select_device(dev, dev->mag.mag_addr);
   int hw = dev->hw_device;
   if (i2c_smbus_read_i2c_block_data(hw, cmd, MAG_DATA_AVAILABLE_SIZE,
         data) < 0)
   {
      device_error(dev, __FILE__, __LINE__, cmd, SENSOR_FLAG_MAG);
   }
}

static void pull_mag_temp_data(
      /* in out */       sensor_runtime_type *dev
      )
{
   uint8_t avail[1];
   // fetch amount of data available
   mag_data_available(dev, avail);
   if (avail[0] & 0x01) {
//printf("MAG data available\n");
      // pull data
      int16_t data[3];
      read_mag_data(dev, data);
      apply_gain(data, &dev->mag.gain, &dev->mag.mag);
//      apply_gain_scale_offset(data, &dev->mag.gain,
//            &dev->mag.scale, &dev->mag.offset, &dev->mag.mag);
      // pull temp data too
      int16_t raw_temp[1];
      read_temp_data(dev, raw_temp);
      dev->temp.celcius = dev->temp.gain * ((double) raw_temp[0]);
//printf("MAG: %d, %d, %d    TEMP: %d\n", data[0], data[1], data[2], raw_temp[0]);
//print_vec(&dev->accel.mag, "MAG data");
//printf("    temp=%f\n", (double) dev->temp.celcius);
   }
}

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////



static void initialize_device(
      /* in out */       sensor_runtime_type *device
      )
{
   sensor_accel_type *acc = &device->accel;
   sensor_mag_type *mag = &device->mag;
   sensor_temp_type *temp = &device->temp;
   /////////////////////////////////////////////////////////////////////
   // accelerometer
   // keep acc at lower rate than driver as high fidelity not required,
   // REG1 25Hz, xyz enabled
	write_acc_register(device, CTRL_REG1_A, 0b00110111);
   // REG2 no filtering
	write_acc_register(device, CTRL_REG2_A, 0b00110111);
   // REG4 +/- 4G, high-res
	write_acc_register(device, CTRL_REG4_A, 0b00011000);
   // docs say 2mg/LSB, but value from lsm9ds0 works much better
   const double acc_gain = 0.000122;
   for (uint32_t i=0; i<3; i++)
      acc->gain.v[i] = acc_gain;
   // magnetometer
   // REG1 temp enabled, 15Hz
   write_mag_register(device, CRA_REG_M, 0b10010000);
   // REG6 +/- 1.9 Gauss
   write_mag_register(device, CRB_REG_M, 0b01000000);
   // REG7 continuous mode
   write_mag_register(device, MR_REG_M, 0b00000000);
   //
   mag->gain.v[0] = 1.0 / 855.0;
   mag->gain.v[1] = 1.0 / 855.0;
   mag->gain.v[2] = 1.0 / 760.0;
   temp->gain = 1.0 / 8.0;
}


////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
// External facing code

int32_t lsm303_setup(
      /* in out */       sensor_runtime_type *dev
      )
{
   if (enable_device(dev, HARDWARE_NAME))
      goto err;
   // lsm303 has accel, mag and temp
   init_sensor_accel(&dev->accel, NULL, 1.0);
   init_sensor_mag(&dev->mag, NULL, 1.0);
   init_sensor_temp(&dev->temp, NULL, 1.0);
   //
   fetch_accel_config(dev);
   fetch_mag_config(dev);
   fetch_temp_config(dev);
   // on failure, initialize device will disable subsensors or
   //    will disable the entire device -> no need to check for errors
   initialize_device(dev);
   // set flags last -- only if initialization successful
   dev->flags |= SENSOR_FLAGS_ACC_MAG_TEMP;
   // set initial delay
   dev->waketime.tv_sec = 0;
   dev->waketime.tv_nsec = WARMUP_INTERVAL * TIME_ADD_1MS;
   return 0;
err:
   return -1;
}


void lsm303_update(
      /* in out */       sensor_runtime_type *dev,
      /*    out */       int32_t *data_available
      )
{
   pull_acc_data(dev);
   pull_mag_temp_data(dev);
   // update timer
   timeadd(&dev->waketime, 0, UPDATE_INTERVAL_US * TIME_ADD_1US);
   *data_available = 1;
}

void lsm303_shutdown(
      /* in out */       sensor_runtime_type *dev
      )
{
   // NOTE: the close here belongs wrapped in a function the level where the
   //    value was enabled
   close(dev->hw_device);
   dev->hw_device = -1;
}

