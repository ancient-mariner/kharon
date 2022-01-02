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

#include "lis3dh.h"
#include "lis3dh_registers.h"

// FIFO off, acquisition rate at 50Hz. It is assumed that polling rate is
//    higher (e.g., 100Hz)
// NOTE for this device, read of data appears to fail if new data is not
//    available (e.g., when reading at twice acquisition frequency, half
//    of read attempts will fail)

#define HARDWARE_NAME   HARDWARE_NAME_LIS3DH

// update interval should be less than sensors sampling interval
#define UPDATE_INTERVAL_US    (2 * SAMPLING_RATE_BASE_US)   // -> 25ms

#define WARMUP_INTERVAL    WARMUP_INTERVAL_BASE


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

static int32_t read_acc_data(
      /* in out */       sensor_runtime_type *dev
      )
{
   uint8_t cmd;
   uint8_t raw[6];
   int16_t data[3];
   int hw = dev->hw_device;
   /////////////////////////////////////////////////////////////////////
   ///////////////////////////////////////////////////////////////
   cmd = 0x80 | STATUS_REG;
   if (i2c_smbus_read_i2c_block_data(hw, cmd, 1, raw) < 0)
   {
      fprintf(stderr, "Error reading status register\n");
      device_error(dev, __FILE__, __LINE__, cmd, SENSOR_FLAGS_ACC_TEMP);
      goto no_data;
   }
   /////////////////////////////////////////////////////////////////////
   if ((raw[0] & 0x0f) == 0) {
      goto no_data;
   }
   cmd = 0x80 | OUT_X_L;
   if (i2c_smbus_read_i2c_block_data(hw, cmd, sizeof(raw), raw) < 0) {
      fprintf(stderr, "Error reading XYZ buffers\n");
      device_error(dev, __FILE__, __LINE__, cmd, SENSOR_FLAGS_ACC_TEMP);
      goto no_data;
   }
   // acc ordered X, Y, Z little endian
   data[0] = (int16_t) (raw[0] | raw[1] << 8);
   data[1] = (int16_t) (raw[2] | raw[3] << 8);
   data[2] = (int16_t) (raw[4] | raw[5] << 8);
//printf("DATA  0x%04x  0x%04x  0x%04x\n", (uint16_t) data[0], (uint16_t) data[1], (uint16_t) data[2]);
   apply_gain(data, &dev->accel.gain, &dev->accel.up);
//   apply_gain_scale_offset(data, &dev->accel.gain, 
//         &dev->accel.scale, &dev->accel.offset, &dev->accel.up);
   /////////////////////////////////////////////////////////////////////
   // temp  TODO figure out why this isn't providing temp data
   cmd = 0x80 | OUT_ADC3_L;
   if (i2c_smbus_read_i2c_block_data(hw, cmd, 2, raw) < 0) {
      fprintf(stderr, "Error reading temperature\n");
      device_error(dev, __FILE__, __LINE__, cmd, SENSOR_FLAGS_ACC_TEMP);
      goto no_data;
   }
   // temp data feeds through ADC-3. temp is 8-bit while ADC provides
   //    10 bits. examining data stream provides no clues as to how
   //    the bits returned can represent temperature. the closest is
   //    dividint tdata by 32, but that's very unlikely to be right.
   //    output -1 to indicate unreliability of signal
   //int16_t tdata = (int16_t) (raw[0] | raw[1] << 8);                             
   //dev->temp.celcius = (float) (tdata/32);  
   dev->temp.celcius = -1.0f;
   /////////////////////////////////////////////////////////////////////
   return 0;
no_data:
   return -1;
}

////////////////////////////////////////////////////////////////////////
//

static void initialize_device(
      /* in out */       sensor_runtime_type *device
      )
{
   // this sensor has a single device. set it here and forget about it
   select_device(device, device->accel.accel_addr);
   // ADC and temp enabled
   // TODO figure out temp output -- present readings quite possibly noise
   write_register(device, TEMP_CFG_REG, 0b11000000);
   // 25Hz, xyz enabled, normal/hi-res mode
   write_register(device, CTRL_REG1, 0b00110111);
   // no filters
   write_register(device, CTRL_REG2, 0b00000000);
   // registers not updated until read; +/-2G, high-res output mode
   write_register(device, CTRL_REG4, 0b10001000);
   // no FIFO
   //
   sensor_accel_type *acc = &device->accel;
   // TODO investigate -- gain should be 1/1000, not 1/16000
   const double acc_gain = 1.0 / 16000.0; 
   for (uint32_t i=0; i<3; i++) {
      acc->gain.v[i] = acc_gain;
   }
   device->temp.gain = 1.0;
}

/**
registers are not updated until read. a stale value may thus be stored
and read out on the first sample. the parent process (e.g., 's2') should
discard the first value from the stream
**/

////////////////////////////////////////////////////////////////////////
// External facing code

int32_t lis3dh_setup(
      /* in out */       sensor_runtime_type *dev
      )
{
   if (enable_device(dev, HARDWARE_NAME))
      goto err;
   // lis3dh has accel and temp
   init_sensor_accel(&dev->accel, NULL, 1.0);
   init_sensor_temp(&dev->temp, NULL, 1.0);
   //
   fetch_accel_config(dev);
   fetch_temp_config(dev);
   //
   initialize_device(dev);
   // run in default mode, so no initializtion necessary
   // set flags last -- only if initialization successful
   dev->flags |= SENSOR_FLAGS_ACC_TEMP;
   // 
   dev->waketime.tv_sec = 0;
   dev->waketime.tv_nsec = WARMUP_INTERVAL * TIME_ADD_1MS;
   dev->state = 0;
   return 0;
err:
   return -1;
}

int32_t lis3dh_check_whoami(
      /* in out */       sensor_runtime_type *dev
      )
{
   uint8_t cmd = 0x80 | WHO_AM_I;
   int hw = dev->hw_device;
   uint8_t data;
   if (i2c_smbus_read_i2c_block_data(hw, cmd, 1, &data) < 0) {
      device_error(dev, __FILE__, __LINE__, cmd, SENSOR_FLAGS_ACC_TEMP);
   }
   if (data != WHO_AM_I_VALUE) {
      fprintf(stderr, "WHO_AM_I check failed. Expected 0x%02x, got 0x%02x\n",
            WHO_AM_I_VALUE, data);
      // treat this is fatal
      goto err;
   }
   return 0;
err:
   return -1;
}

void lis3dh_update(
      /* in out */       sensor_runtime_type *dev,
      /*    out */       int32_t *data_available
      )
{
   // pull data
   if (read_acc_data(dev) == 0) {
      *data_available = 1;
//print_vec(&dev->accel.up, "ACC data");
//printf("ACC: %d, %d, %d  \n", data[0], data[1], data[2]);
   } else {
      *data_available = 0;
   }
   // update timer
   timeadd(&dev->waketime, 0, UPDATE_INTERVAL_US * TIME_ADD_1US);
}

void lis3dh_shutdown(
      /* in out */       sensor_runtime_type *dev
      )
{
   close(dev->hw_device);
   dev->hw_device = -1;
}

