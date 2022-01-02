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

#include "lsm9ds0.h"
#include "lsm9ds0_registers.h"

#define HARDWARE_NAME   HARDWARE_NAME_LSM9DS0

#define UPDATE_INTERVAL_US    SAMPLING_RATE_BASE_US

#define DT_SEC       (1.0e-6 * ((double) UPDATE_INTERVAL_US))
#define DRIFT_TIME_CONST_SEC  900.0
#define DRIFT_TAU    (1.0 / (DRIFT_TIME_CONST_SEC / DT_SEC))

#define WARMUP_INTERVAL       WARMUP_INTERVAL_BASE

#define DRIFT_LOG_INTERVAL    (5.0 * 60.0)
static double log_timer_ = 0.0;


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
   uint8_t cmd = 0x80 | OUT_X_L_M;
   select_device(dev, dev->mag.mag_addr);
   int hw = dev->hw_device;
   if (i2c_smbus_read_i2c_block_data(hw, cmd, sizeof(raw), raw) < 0) {
      device_error(dev, __FILE__, __LINE__, cmd, SENSOR_FLAG_MAG);
   }
   data[0] = (int16_t) (raw[0] | raw[1] << 8);
   data[1] = (int16_t) (raw[2] | raw[3] << 8);
   data[2] = (int16_t) (raw[4] | raw[5] << 8);
}

//static void read_gyr_data(
//      /* in out */       sensor_runtime_type *dev,
//      /*    out */       int16_t data[3]
//      )
//{
//   uint8_t raw[6];
//   uint8_t cmd = 0x80 | OUT_X_L_G;
//   select_device(dev, dev->gyro.gyro_addr);
//   int hw = dev->hw_device;
//   if (i2c_smbus_read_i2c_block_data(hw, cmd, sizeof(raw), raw) < 0) {
//      device_error(dev, __FILE__, __LINE__, cmd, SENSOR_FLAG_GYRO);
//   }
//   data[0] = (int16_t) (raw[0] | raw[1] << 8);
//   data[1] = (int16_t) (raw[2] | raw[3] << 8);
//   data[2] = (int16_t) (raw[4] | raw[5] << 8);
//}

static void read_temp_data(
      /* in out */       sensor_runtime_type *dev,
      /*    out */       int16_t data[1]
      )
{
   uint8_t raw[2];
   uint8_t cmd = 0x80 | OUT_TEMP_L_XM;
   select_device(dev, dev->temp.temp_addr);
   int hw = dev->hw_device;
   if (i2c_smbus_read_i2c_block_data(hw, cmd, sizeof(raw), raw) < 0) {
      device_error(dev, __FILE__, __LINE__, cmd, SENSOR_FLAG_TEMP);
   }
   data[0] = (int16_t) (raw[0] | raw[1] << 8);
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
//      axis_dps->v[i] = ((float) accum[i]) * gain->v[i];
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
   uint8_t cmd = 0x80 | FIFO_SRC_REG_G;
   if (i2c_smbus_read_i2c_block_data(hw, cmd, 1, fifo) < 0) {
      device_error(dev, __FILE__, __LINE__, cmd, SENSOR_FLAG_GYRO);
   }
   const uint32_t cnt = (uint32_t) (fifo[0] & 0x0f);
//printf("GYRO FIFO has %d elements\n", cnt);
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
         if (i2c_smbus_read_i2c_block_data(hw, (0x80 | OUT_X_L_G),
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
//print_vec(&dev->gyro.axis_dps, "gyro");
   }
}


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
   uint8_t cmd = 0x80 | STATUS_REG_M;
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
   if (avail[0] & 0x08) {
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
   sensor_gyro_type *gyr = &device->gyro;
   sensor_accel_type *acc = &device->accel;
   sensor_mag_type *mag = &device->mag;
   sensor_temp_type *temp = &device->temp;
   /////////////////////////////////////////////////////////////////////
   // accelerometer
   // keep acc at lower rate than driver as high fidelity not required,
   // REG0 fifo, watermark and filters off
	write_acc_register(device, CTRL_REG0_XM, 0b00000000);
   // REG1 25 Hz update, continuous update, axes enabled
	write_acc_register(device, CTRL_REG1_XM, 0b01000111);
   // REG2 +/- 4G
	write_acc_register(device, CTRL_REG2_XM, 0b00001000);
   //
   const double acc_gain = 0.000122; // == 0.122 mg/LSB
   for (uint32_t i=0; i<3; i++)
      acc->gain.v[i] = acc_gain;
   // magnetometer
   // REG5 TEMP on, high res, 12.5Hz 
   write_mag_register(device, CTRL_REG5_XM, 0b11101000);
   // REG6 +/- 2 Gauss
   write_mag_register(device, CTRL_REG6_XM, 0b00000000);
   // REG7 filter off, mag on
   write_mag_register(device, CTRL_REG7_XM, 0b00000000);
   //
   const double mag_gain = 0.00008;
   for (uint32_t i=0; i<3; i++)
      mag->gain.v[i] = mag_gain;
   temp->gain = 1.0 / 8.0;
   /////////////////////////////////////////////////////////////////////
   // gyro
   //    stream (fifo) mode
   //    190Hz reads, cutoff=25
   //    245dps
   // 95/25; device on, all axes reporting
   write_gyr_register(device, CTRL_REG1_G, 0b00011111);
   // REG2 for filtering [disabled -- high pass filter in software]
   // REG3 misc setup (default)
   // REG4 245dps
   write_gyr_register(device, CTRL_REG4_G, 0b00000000);
   // REG5 fifo enable, hp-filter disable
   write_gyr_register(device, CTRL_REG5_G, 0b01000000);
   // FIFO_CTRL_REG stream mode, [unused] watermark=16
   write_gyr_register(device, FIFO_CTRL_REG_G, 0b01010000);
   // 
   const double gyr_gain = 0.00875;
   //const float gyr_gain = 0.00875f * DT_SEC;
   for (uint32_t i=0; i<3; i++)
      gyr->gain.v[i] = gyr_gain;
}


//// check WHO_AM_I registers on device to make sure we aren't loading
////    the wrong driver for a device
//static uint32_t check_correct_hardware(
//      /* in out */       sensor_runtime_type *dev
//      )
//{
//   uint32_t errs = 0;
//   uint8_t cmd;
//   int hw = dev->hw_device;
//   uint8_t who[1];
//   // check WHO_AM_I registers
//   // gyro
//   select_device(dev, dev->gyro.gyro_addr);
//   cmd = 0x80 | WHO_AM_I_G;
//   if (i2c_smbus_read_i2c_block_data(hw, cmd, sizeof(who), who) < 0) {
//      device_error(dev, __FILE__, __LINE__, cmd, SENSOR_FLAG_GYRO);
//   }
//   if (who[0] != 0xd4) {
//      fprintf(stderr, "Failed to identify gyro -- who_am_i=%02x\n", who[0]);
//      errs++;
//   }
//   // xm
//   select_device(dev, dev->accel.accel_addr);
//   cmd = 0x80 | WHO_AM_I_XM;
//   if (i2c_smbus_read_i2c_block_data(hw, cmd, sizeof(who), who) < 0) {
//      device_error(dev, __FILE__, __LINE__, cmd, SENSOR_FLAG_ACC_MAG);
//   }
//   if (who[0] != 0x49) {
//      fprintf(stderr, "Failed to identify acc/mag -- who_am_i=%02x\n", who[0]);
//      errs++;
//   }
//   return errs;
//}

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
// External facing code

int32_t lsm9ds0_setup(
      /* in out */       sensor_runtime_type *dev
      )
{
   if (enable_device(dev, HARDWARE_NAME))
      goto err;
//   if (check_correct_hardware(dev) != 0)
//      goto end;
   // lsm9ds0 has gyro, accel, mag and temp
   init_sensor_gyro(&dev->gyro, NULL, 1.0);
   init_sensor_accel(&dev->accel, NULL, 1.0);
   init_sensor_mag(&dev->mag, NULL, 1.0);
   // get config data (eg, offsets, i2c addresses, etc)
   fetch_gyro_config(dev);
   fetch_accel_config(dev);
   fetch_mag_config(dev);
   fetch_temp_config(dev);
   // on failure, initialize device will disable subsensors or
   //    will disable the entire device -> no need to check for errors
printf("initializing device\n");
   initialize_device(dev);
   // set flags last -- only if initialization successful
   dev->flags |= SENSOR_FLAGS_GYRO_ACC_MAG_TEMP;
   // set initial delay
   dev->waketime.tv_sec = 0;
   dev->waketime.tv_nsec = WARMUP_INTERVAL * TIME_ADD_1MS;
   return 0;
err:
   return -1;
}


void lsm9ds0_update(
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
   pull_acc_data(dev);
   pull_mag_temp_data(dev);
   // update timer
   timeadd(&dev->waketime, 0, UPDATE_INTERVAL_US * TIME_ADD_1US);
   *data_available = 1;
}


void lsm9ds0_shutdown(
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

