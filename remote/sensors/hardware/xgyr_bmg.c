#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <unistd.h>

#include "bmg160_registers.h"

#define WARMUP_INTERVAL_MSEC  (50)
#define UPDATE_INTERVAL_MSEC  (10)

#define DT_SEC       (1.0e-3f * ((float) UPDATE_INTERVAL_MSEC))
#define DRIFT_TIME_CONST_SEC  900.0f
#define DRIFT_TAU    (1.0f / (DRIFT_TIME_CONST_SEC / DT_SEC))

#define ADDR 0x68

#error "This is a work in progress. it mostly works, but still very much doesn't"

const float GYR_GAIN = 1.0f / 131.2f;
const float TEMP_GAIN = 0.5f;

int device_;
double celcius_;
double output_[3];

static void enable_device(void)
{
   // sanity check
   if ((device_ = open("/dev/i2c-1", O_RDWR)) < 0) {
      fprintf(stderr, "Failure accessing /dev/i2c-1\n");
      exit(1);
   }
   // select device
   if (ioctl(device_, I2C_SLAVE, ADDR) < 0) {
      perror("Failure selecting device");
      exit(1);
   }
}

int check_whoami(void)
{
   // check WHO_AM_I
   uint8_t cmd = 0;
   uint8_t expected = 0x0f;
   uint8_t data;
   if (i2c_smbus_read_i2c_block_data(device_, cmd, 1, &data) < 0) 
   {
      fprintf(stderr, "Error reading WHOAMI data\n");
      return -1;
   }
   if (data != expected) {
      fprintf(stderr, "WHO_AM_I check failed. Expected 0x%02x, got 0x%02x\n",
            expected, data);
      return -1;
   }
   return 0;
}

static void write_register(
      /* in     */ const uint8_t reg, 
      /* in     */ const uint8_t value
      )
{
   if (i2c_smbus_write_byte_data(device_, reg, value) < 0) {
      fprintf(stderr, "Write regiser error. reg=%d, value=%d\n", reg, value);
      exit(1);
   }
}

////////////////////////////////////////////////////////////////////////

static void initialize_device(void)
{
   // 250dps
   write_register(RANGE_ADDR, 0b00000011);
   // 200Hz, filter bandwidth=32Hz
   write_register(BW_ADDR, 0b10000100);
//   // high filter disable; MSB shadowing enabled
//   write_register(RATE_HBW_ADDR, 0b00000000);
   // start out in bypass mode. after first data read, switch to stream
   write_register(FIFO_CONFIG_1_ADDR, 0b00000000);
}

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////


static void read_temp_data(void)
{
   uint8_t raw;
   if (i2c_smbus_read_i2c_block_data(device_, TEMP_ADDR, 1, &raw) < 0) {
      fprintf(stderr, "Failure reading temp data\n");
      raw = 255;
   }
   celcius_ = 23.0 + TEMP_GAIN * (double) raw;
}


static uint32_t read_gyro_data(void)
{
   // fetch amount of data available
   uint8_t fifo;
   if (i2c_smbus_read_i2c_block_data(device_, FIFO_STAT_ADDR, 1, &fifo) < 0) {
      fprintf(stderr, "Failed to get fifo status\n");
      return 0;
   }
   uint32_t cnt = (uint32_t) (fifo & 0x7f);
   // process data if it's available
   if (cnt > 0) {
printf("Reading %d samples\n", cnt);
      // data transfered as bytes in little endian order. have
      //    data be written directly to int16 memory
      int8_t raw[6];
      int32_t accum[3] = { 0, 0, 0 };
      for (uint32_t i=0; i<cnt; i++) {
         if (i2c_smbus_read_i2c_block_data(device_, FIFO_DATA_ADDR,
                  sizeof(raw), raw) < 0) {
            fprintf(stderr, "Failed reading data sample %d\n", i);
            return 0;
         }
         int32_t output[3];
         for (uint32_t j=0; j<3; j++) {
            output[j] = (int32_t) (raw[2*j+1] | (raw[2*j] << 8));
            accum[j] += output[j];
printf("  %d ", output[j]);
         }
printf("\n");
      }
printf(" -> %d %d %d\n", accum[0], accum[1], accum[2]);
      for (uint32_t i=0; i<3; i++) {
         accum[i] /= (int32_t) cnt;
         output_[i] = (double) accum[i] * GYR_GAIN;
      }
   }
   write_register(FIFO_CONFIG_1_ADDR, 0b10000000);
   return cnt;
}

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

int main()
{
   enable_device();
   if (check_whoami() != 0) {
      return 1;
   }
   initialize_device();
   uint32_t i;
   for (i=0; i<50; i++) {
      printf("%3d   %7.3f   ", i, i * 0.001*UPDATE_INTERVAL_MSEC);
      if (read_gyro_data() != 0) {
         read_temp_data();
         printf("     %8.3f %8.3f %8.3f  t=%.3f",
               output_[0], output_[1], output_[2], celcius_);
      }
      printf("\n");
      usleep(1000 * UPDATE_INTERVAL_MSEC);
   }
   close(device_);
   return 0;
}

