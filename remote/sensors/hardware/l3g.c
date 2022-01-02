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

#include "l3g.h"
#include "l3g_registers.h"

#define HARDWARE_NAME   HARDWARE_NAME_L3G

#define UPDATE_INTERVAL_US    SAMPLING_RATE_BASE_US   // 12.5ms

#define DT_SEC       (1.0e-6 * ((double) UPDATE_INTERVAL_US))
#define DRIFT_TIME_CONST_SEC  900.0
#define DRIFT_TAU    (1.0 / (DRIFT_TIME_CONST_SEC / DT_SEC))

#define WARMUP_INTERVAL       WARMUP_INTERVAL_BASE

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
   uint8_t cmd = 0x80 | OUT_TEMP;
   select_device(dev, dev->temp.temp_addr);
   int hw = dev->hw_device;
   if (i2c_smbus_read_i2c_block_data(hw, cmd, sizeof(raw), raw) < 0) {
      device_error(dev, __FILE__, __LINE__, cmd, SENSOR_FLAG_TEMP);
   }
   data[0] = (int16_t) raw[0];
}

////////////////////////////////////////////////////////////////////////
//


//static void update_gyro_drift(
//      /* in out */       sensor_runtime_type *dev,
//      /* in     */ const int32_t accum[3]
//      )
//{
//   vector_type *drift_dps = &dev->gyro.drift_dps;
//   vector_type *axis_dps = &dev->gyro.axis_dps;
//   vector_type *gain = &dev->gyro.gain;
//   // store rotation in axis, converting acquired signal to deg-per-sec
//   for (uint32_t i=0; i<3; i++) {
//      axis_dps->v[i] = ((double) accum[i]) * gain->v[i];
//   }
//   // drift estimate calcualted by taking long-term average of sensor output
//   for (uint32_t i=0; i<3; i++) {
//      drift_dps->v[i] = (1.0f - DRIFT_TAU) * drift_dps->v[i] +
//            DRIFT_TAU * axis_dps->v[i];
//   }
//   // subtract drift estimate from signal
//   for (uint32_t i=0; i<3; i++) {
//      axis_dps->v[i] -= drift_dps->v[i];
//   }
//}

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
   uint8_t cmd = 0x80 | FIFO_SRC_REG;
   if (i2c_smbus_read_i2c_block_data(hw, cmd, 1, fifo) < 0) {
      device_error(dev, __FILE__, __LINE__, cmd, SENSOR_FLAG_GYRO);
   }
   const uint32_t cnt = (uint32_t) (fifo[0] & 0x0f);
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
         if (i2c_smbus_read_i2c_block_data(hw, (0x80 | OUT_X_L),
                  sizeof(bucket.raw), bucket.raw) < 0) {
            device_error(dev, __FILE__, __LINE__, cmd, SENSOR_FLAG_GYRO);
         }
         accum[0] += (int32_t) bucket.vals[0];
         accum[1] += (int32_t) bucket.vals[1];
         accum[2] += (int32_t) bucket.vals[2];
      }
      for (uint32_t i=0; i<3; i++)
         accum[i] /= (int32_t) cnt;
      // average rotation data over measured period
      update_gyro_drift(dev, accum, DRIFT_TAU);
   }
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
   dev->temp.celcius = dev->temp.gain * ((double) raw_temp[0]);
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
   //    stream (fifo) mode
   //    190Hz reads, cutoff=25
   //    245dps
   // 95/25; device on, all axes reporting
   write_gyr_register(device, CTRL_REG1, 0b00011111);
   // REG2 for filtering [disabled -- high pass filter in software]
   // REG3 misc setup (default)
   // REG4 250dps
   write_gyr_register(device, CTRL_REG4, 0b00000000);
   // REG5 fifo enable, hp-filter disable
   write_gyr_register(device, CTRL_REG5, 0b01000000);
   // FIFO_CTRL_REG stream mode, [unused] watermark=16
   write_gyr_register(device, FIFO_CTRL_REG, 0b01010000);
   // 
   const double gyr_gain = 0.00875;
   for (uint32_t i=0; i<3; i++)
      gyr->gain.v[i] = gyr_gain;
   //
   temp->gain = 1.0f;
}


////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
// External facing code

int32_t l3g_setup(
      /* in out */       sensor_runtime_type *dev
      )
{
   if (enable_device(dev, HARDWARE_NAME))
      goto err;
   // l3g has gyro and temp
   init_sensor_gyro(&dev->gyro, NULL, 1.0);
   init_sensor_temp(&dev->temp, NULL, 1.0);
   dev->temp.celcius = 0.0f;
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


void l3g_update(
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
   //
   pull_gyro_data(dev);
   pull_temp_data(dev);
   // update timer
   timeadd(&dev->waketime, 0, UPDATE_INTERVAL_US * TIME_ADD_1US);
   *data_available = 1;
}

void l3g_shutdown(
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

