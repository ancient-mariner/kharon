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

#include "lis3mdl.h"
#include "lis3mdl_registers.h"

#define HARDWARE_NAME   HARDWARE_NAME_LIS3MDL

// query interval
#define UPDATE_INTERVAL_US    (2 * SAMPLING_RATE_BASE_US)   // -> 25ms

#define WARMUP_INTERVAL       (WARMUP_INTERVAL_BASE)


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

static void mag_data_available(
      /* in     */       sensor_runtime_type *dev,
      /*    out */       uint8_t *data
      )
{
   uint8_t cmd = 0x80 | STATUS_REG;
   int hw = dev->hw_device;
   if (i2c_smbus_read_i2c_block_data(hw, cmd, 1, data) < 0) 
   {
      device_error(dev, __FILE__, __LINE__, cmd, SENSOR_FLAGS_MAG_TEMP);
   }
}

static int32_t read_mag_data(
      /* in out */       sensor_runtime_type *dev
      )
{
   uint8_t raw[8];
   int16_t data[4];
   uint8_t cmd = 0x80 | OUT_X_L;
   int hw = dev->hw_device;
   if (i2c_smbus_read_i2c_block_data(hw, cmd, 6, raw) < 0) {
      device_error(dev, __FILE__, __LINE__, cmd, SENSOR_FLAGS_MAG_TEMP);
      return -1;
   }
   cmd = 0x80 | TEMP_OUT_L;
   if (i2c_smbus_read_i2c_block_data(hw, cmd, 2, &raw[6]) < 0) {
      device_error(dev, __FILE__, __LINE__, cmd, SENSOR_FLAGS_MAG_TEMP);
      return -1;
   }
   // mag data order X Y Z Temp, little endian
   data[0] = (int16_t) (raw[0] | raw[1] << 8);
   data[1] = (int16_t) (raw[2] | raw[3] << 8);
   data[2] = (int16_t) (raw[4] | raw[5] << 8);
   data[3] = (int16_t) (raw[6] | raw[7] << 8);
   apply_gain(data, &dev->mag.gain, &dev->mag.mag);
//   apply_gain_scale_offset(data, &dev->mag.gain, 
//         &dev->mag.scale, &dev->mag.offset, &dev->mag.mag);
   dev->temp.celcius = 25.0 + dev->temp.gain * ((double) data[3]);
//printf("TEMP %04x  (%.3f  %.3f)  gain: %.3f\n", (uint16_t) data[3], dev->temp.celcius, (double) data[3], dev->temp.gain);
   return 0;
}

////////////////////////////////////////////////////////////////////////
//

static void initialize_device(
      /* in out */       sensor_runtime_type *device
      )
{
   // this sensor has a single device. set it here and forget about it
   select_device(device, device->mag.mag_addr);
   // ultra-high-resolution mode xy; 40Hz; temp enabled
   write_register(device, CTRL_REG1, 0b11111000);
   // +/- 4 gauss
   write_register(device, CTRL_REG2, 0b00000000);
   // continuous conversion mode
   write_register(device, CTRL_REG3, 0b00000000);
   // 'ultra-high-performance mode' z; little endian
   write_register(device, CTRL_REG4, 0b00001100);
   // output registers not updated until MSb and LSb have been read
   write_register(device, CTRL_REG5, 0b01000000);
   //
   sensor_mag_type *mag = &device->mag;
   const double mag_gain = 1.0 / 6842.0;
   for (uint32_t i=0; i<3; i++) {
      mag->gain.v[i] = mag_gain;
   }
   device->temp.gain = 1.0 / 8.0;
}

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
// External facing code

int32_t lis3mdl_setup(
      /* in out */       sensor_runtime_type *dev
      )
{
   if (enable_device(dev, HARDWARE_NAME))
      goto err;
   // lis3mdl has mag and temp
   init_sensor_mag(&dev->mag, NULL, 1.0);
   init_sensor_temp(&dev->temp, NULL, 1.0);
   //
   fetch_mag_config(dev);
   //
   initialize_device(dev);
   // run in default mode, so no initializtion necessary
   // set flags last -- only if initialization successful
   dev->flags |= SENSOR_FLAGS_MAG_TEMP;
   // set initial delay. needs 500ms startup time
   dev->waketime.tv_sec = 0;
   dev->waketime.tv_nsec = WARMUP_INTERVAL * TIME_ADD_1MS + TIME_ADD_12_5MS;
   dev->state = 0;
   return 0;
err:
   return -1;
}
//
//
int32_t lis3mdl_check_whoami(
      /* in out */       sensor_runtime_type *dev
      )
{
   // check WHO_AM_I
   uint8_t cmd = 0x80 | WHO_AM_I;
   int hw = dev->hw_device;
   uint8_t data;
   if (i2c_smbus_read_i2c_block_data(hw, cmd, 1, &data) < 0) 
   {
      device_error(dev, __FILE__, __LINE__, cmd, SENSOR_FLAGS_MAG_TEMP);
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


void lis3mdl_update(
      /* in out */       sensor_runtime_type *dev,
      /*    out */       int32_t *data_available
      )
{
   uint8_t avail = 0;
   *data_available = 0;
   // see if there's data available
   mag_data_available(dev, &avail);
   if (avail & 0x08) {
      // pull data
      read_mag_data(dev);
      *data_available = 1;
   } 
   timeadd(&dev->waketime, 0, UPDATE_INTERVAL_US * TIME_ADD_1US);
}


void lis3mdl_shutdown(
      /* in out */       sensor_runtime_type *dev
      )
{
   // NOTE: the close here belongs wrapped in a function the level where the
   //    value was enabled
   close(dev->hw_device);
   dev->hw_device = -1;
}

