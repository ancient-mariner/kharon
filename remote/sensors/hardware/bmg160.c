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
#include "timekeeper.h"

#include "timekeeper.h"

#include "bmg160.h"
#include "bmg160_registers.h"

#define HARDWARE_NAME   HARDWARE_NAME_BMG160

#define UPDATE_INTERVAL_US    SAMPLING_RATE_BASE_US   // 12.5ms

#define DT_SEC       (1.0e-6 * ((double) UPDATE_INTERVAL_US))
#define DRIFT_TIME_CONST_SEC  900.0
#define DRIFT_TAU    (1.0 / (DRIFT_TIME_CONST_SEC / DT_SEC))

#define WARMUP_INTERVAL    WARMUP_INTERVAL_BASE

#define DRIFT_LOG_INTERVAL    (5.0 * 60.0)
static double log_timer_ = 0.0;


static void write_gyr_register(
      /* in out */       sensor_runtime_type *dev,
      /* in     */ const uint8_t reg, 
      /* in     */ const uint8_t value
      )
{
   select_device(dev, dev->gyro.gyro_addr);
   if (i2c_smbus_write_byte_data(dev->hw_device, reg, value) < 0) {
      device_error(dev, __FILE__, __LINE__, reg, SENSOR_FLAG_GYRO);
   }
}

////////////////////////////////////////////////////////////////////////


static void read_temp_data(
      /* in out */       sensor_runtime_type *dev,
      /*    out */       int16_t data[1]
      )
{
   uint8_t raw[1];
   const uint8_t cmd = TEMP_ADDR;
   select_device(dev, dev->temp.temp_addr);
   int hw = dev->hw_device;
   if (i2c_smbus_read_i2c_block_data(hw, cmd, sizeof(raw), raw) < 0) {
      device_error(dev, __FILE__, __LINE__, cmd, SENSOR_FLAG_TEMP);
   }
   data[0] = (int16_t) ((int8_t) raw[0]);
}

////////////////////////////////////////////////////////////////////////
//

// querying avaiable data and pulling data were merged when investigating
//    performance problems (turned out it was i2c baud rate)
// should make style of gyro data consistent with acc and mag
static void pull_gyro_data(
      /* in out */       sensor_runtime_type *dev
      )
{
   // fetch amount of data available
   select_device(dev, dev->gyro.gyro_addr);
   const int hw = dev->hw_device;
   uint8_t fifo[1];
   const uint8_t cmd =  FIFO_STAT_ADDR;
   if (i2c_smbus_read_i2c_block_data(hw, cmd, 1, fifo) < 0) {
      device_error(dev, __FILE__, __LINE__, cmd, SENSOR_FLAG_GYRO);
   }
   uint32_t cnt = (uint32_t) (fifo[0] & 0x7f);
//printf("count: %d\n", cnt);
   // process data if it's available
   if (cnt > 0) {
      // data transfered as bytes in little endian order. have
      //    data be written directly to int16 memory
      union {
         uint8_t raw[6];
         int16_t vals[3];
      } bucket;
      // 
      int32_t accum[3];
      for (uint32_t i=0; i<3; i++)
         accum[i] = 0;
      for (uint32_t i=0; i<cnt; i++) {
         if (i2c_smbus_read_i2c_block_data(hw, FIFO_DATA_ADDR,
                  sizeof(bucket.raw), bucket.raw) < 0) {
            device_error(dev, __FILE__, __LINE__, FIFO_DATA_ADDR, 
                  SENSOR_FLAG_GYRO);
         }
         accum[0] += (int32_t) bucket.vals[0];
         accum[1] += (int32_t) bucket.vals[1];
         accum[2] += (int32_t) bucket.vals[2];
//printf("    %d,%d,%d (%.3f,%.3f,%.3f\n", accum[0], accum[1],accum[2],accum[0]/131.2,accum[1]/131.2,accum[2]/131.2);
//printf("    %4d  %4d  %4d\n", bucket.vals[0], bucket.vals[1], bucket.vals[2]);
      }
      for (uint32_t i=0; i<3; i++)
         accum[i] /= (int32_t) cnt;
      // average rotation data over measured period
      update_gyro_drift(dev, accum, DRIFT_TAU);
   }
   write_gyr_register(dev, FIFO_CONFIG_1_ADDR, 0b10000000);
}



////////////////////////////////////////////////////////////////////////
// mag


static void pull_temp_data(
      /* in out */       sensor_runtime_type *dev
      )
{
   // pull temp data too
   int16_t raw_temp[1];
   read_temp_data(dev, raw_temp);
   // a reading of 0 is 23C
   dev->temp.celcius = 23.0 + dev->temp.gain * ((double) raw_temp[0]);
}

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////



static void initialize_device(
      /* in out */       sensor_runtime_type *device
      )
{
   sensor_gyro_type *gyr = &device->gyro;
   sensor_temp_type *temp = &device->temp;
   /////////////////////////////////////////////////////////////////////
   // gyro
   // 250dps
   write_gyr_register(device, RANGE_ADDR, 0b00000011);
   // 200Hz, filter bandwidth=32Hz
   write_gyr_register(device, BW_ADDR, 0b10000100);
//   // high filter disable; MSB shadowing enabled
//   write_gyr_register(device, RATE_HBW_ADDR, 0b00000000);
   // start out in bypass mode. after first data read, switch to stream
   write_gyr_register(device, FIFO_CONFIG_1_ADDR, 0b00000000);
   // 
   const double gyr_gain = 1.0 / 131.2;
   for (uint32_t i=0; i<3; i++)
      gyr->gain.v[i] = gyr_gain;
   //
   temp->gain = 0.5;
}


////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
// External facing code

int32_t bmg160_setup(
      /* in out */       sensor_runtime_type *dev
      )
{
   if (enable_device(dev, HARDWARE_NAME))
      goto err;
   // bmg160 has gyro and temp
   init_sensor_gyro(&dev->gyro, NULL, 1.0);
   init_sensor_temp(&dev->temp, NULL, 1.0);
   dev->temp.celcius = 0.0;
   // get config data (eg, offsets, i2c addresses, etc)
   fetch_gyro_config(dev);
   fetch_temp_config(dev);
   // on failure, initialize device will disable subsensors or
   //    will disable the entire device -> no need to check for errors
   initialize_device(dev);
   // set flags last -- only if initialization successful
   dev->flags |= SENSOR_FLAGS_GYRO_TEMP;
   // set initial delay
   dev->waketime.tv_sec = 0;
   dev->waketime.tv_nsec = WARMUP_INTERVAL * TIME_ADD_1MS;
   return 0;
err:
   return -1;
}


void bmg160_update(
      /* in out */       sensor_runtime_type *dev,
      /*    out */       int32_t *data_available
      )
{
   // log drift data periodically
   double t = now();
   if (t > log_timer_) {
      log_timer_ = t + DRIFT_LOG_INTERVAL;
      vector_type *drift_dps = &dev->gyro.drift_dps;
      snprintf(dev->log_data, SENSOR_PACKET_LOG_DATA, 
            "drift dps: %.3f, %.3f, %.3f", (double) drift_dps->v[0],
            (double) drift_dps->v[1], (double) drift_dps->v[2]);
   }
   pull_gyro_data(dev);
   pull_temp_data(dev);
   // update timer
   timeadd(&dev->waketime, 0, UPDATE_INTERVAL_US * TIME_ADD_1US);
   *data_available = 1;
}

void bmg160_shutdown(
      /* in out */       sensor_runtime_type *dev
      )
{
   // NOTE: the close here belongs wrapped in a function the level where the
   //    value was enabled
   close(dev->hw_device);
   dev->hw_device = -1;
   // write drift info to config file
   write_gyro_drift(dev);
}

