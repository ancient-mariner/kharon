#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <errno.h>
#include <unistd.h>

////////////////////////////////////////////////////////////////////////

#define WHO_AM_I                 0x0F
#define WHO_AM_I_VALUE           0xD3

#define CTRL_REG1                0x20
#define CTRL_REG2                0x21
#define CTRL_REG3                0x22
#define CTRL_REG4                0x23
#define CTRL_REG5                0x24

#define REFERENCE                0x25
#define OUT_TEMP                 0x26

#define STATUS_REG               0x27
#define OUT_X_L                  0x28
#define OUT_X_H                  0x29
#define OUT_Y_L                  0x2A
#define OUT_Y_H                  0x2B
#define OUT_Z_L                  0x2C
#define OUT_Z_H                  0x2D

#define FIFO_CTRL_REG            0x2E
#define FIFO_SRC_REG             0x2F

////////////////////////////////////////////////////////////////////////

#define WARMUP_INTERVAL_USEC  (50 * 1000)
#define UPDATE_INTERVAL_USEC  (10 * 1000)
#define UPDATE_INTERVAL_SECONDS  ((double) UPDATE_INTERVAL_USEC / 1.0e6)

#define ADDR   0x68

const double SIG_GAIN = 0.00875;
const double TEMP_GAIN = 1.0 / 1.0;

double output_[3];
int16_t last_[3] = { 0, 0, 0 };
double celcius_;
int device_;

////////////////////////////////////////////////////////////////////////

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

void check_whoami(uint8_t addr, uint8_t expected)
{
   // check WHO_AM_I
   uint8_t cmd = 0x80 | addr;
   uint8_t data;
   if (i2c_smbus_read_i2c_block_data(device_, cmd, 1, &data) < 0) 
   {
      fprintf(stderr, "Error reading WHOAMI data\n");
      exit(1);
   }
   if (data != expected) {
      fprintf(stderr, "WHO_AM_I check failed. Expected 0x%02x, got 0x%02x\n",
            expected, data);
      exit(1);
   }
}

static void initialize_device(void)
{
   // 200Hz, 25Hz cutoff, xyz enabled
   write_register(CTRL_REG1, 0b01011111);
   // high pass filter 0.02Hz
   write_register(CTRL_REG2, 0b00101001);
   // REG3 -- no interrupts (default)
   // REG4 -- little endian, no self test (default)
   // fifo enable, high-pass filter enable, 
   write_register(CTRL_REG5, 0b01010001);
   // stream mode, watermark at 1
   write_register(FIFO_CTRL_REG, 0b01000001);
}

////////////////////////////////////////////////////////////////////////

static int32_t read_data(void)
{
   // if this procedure is called then there's data available
   // device stream/fifo read protocol
   // if data is available (check watermark)
   //    get size of fifo
   //    do a dummy read from 0x28 (increment bit 0)
   //    do a burst read (6 bytes) from 0x2a (Y low) to 29h (Y,Z,X)
   uint8_t cmd;
   uint8_t raw[6];
   int16_t data[3];
   int32_t accum[3] = { 0, 0, 0};
   uint32_t i;
   uint8_t status, n, n_samples;
   ///////////////////////////////////////////////////////////////
   // get fifo size
   cmd = 0x80 | FIFO_SRC_REG;
   if (i2c_smbus_read_i2c_block_data(device_, cmd, 1, &status) < 0) {
      fprintf(stderr, "Error reading source register\n");
      return -1;
   }
   n_samples = status & 0x1f;  // trim to lower 5 bits
   n = n_samples;
   if ((status & 0x80) == 0) {
      // watermark bit not set
      printf("Watermark not set 0x%02x  (n=%d)\n", status, n);
      goto no_data;
   }
   uint32_t cnt = 0;
   do {
      // dummy read
      cmd = OUT_X_L; // increment bit = 0
      if (i2c_smbus_read_i2c_block_data(device_, cmd, 2, raw) < 0) {
         fprintf(stderr, "Error during dummy read\n");
         goto no_data;
      }
      // burst read: Y, Z, X
      cmd = 0x80 | OUT_Y_L; 
      if (i2c_smbus_read_i2c_block_data(device_, cmd, 2, &raw[2]) < 0) {
         fprintf(stderr, "Error reading Y\n");
         goto no_data;
      }
      cmd = 0x80 | OUT_Z_L; 
      if (i2c_smbus_read_i2c_block_data(device_, cmd, 2, &raw[4]) < 0) {
         fprintf(stderr, "Error reading Z\n");
         goto no_data;
      }
      cmd = 0x80 | OUT_X_L; 
      if (i2c_smbus_read_i2c_block_data(device_, cmd, 2, &raw[0]) < 0) {
         fprintf(stderr, "Error reading X\n");
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
      // decrement n. if n < 0 then break (if n<0 for uint means is > than max)
   } while (--n < 0x1f);
   if (cnt == 0) {
      // this is an exceptional case -- only one sample (shouldn't be
      //    possible) and it matched the previous last sample. put 
      //    invalid data in last so we don't accidentally trigger again
      last_[0] = 0x8001;
      last_[1] = 0x7fff;
      last_[2] = 0x8001;
      goto no_data;
   }
   last_[0] = data[0];
   last_[1] = data[1];
   last_[2] = data[2];
   // apply gain and subtract drift
   for (i=0; i<3; i++) {
      // make average of signal (DPS) and return that
      output_[i] = (double) accum[i] * SIG_GAIN / ((double) cnt);
//      gyro->axis_dps.v[i] = (int32_t) (dtheta * gyro->gain.v[i] - 
//            gyro->drift_dps.v[i] * dt);
   }
   //
   cmd = 0x80 | OUT_TEMP;
   if (i2c_smbus_read_i2c_block_data(device_, cmd, 1, raw) < 0) {
      fprintf(stderr, "Error reading temp data\n");
      exit(1);
   }
   celcius_ = (double) raw[0] * TEMP_GAIN;
   //
   return 1;
no_data:
   return 0;
}

////////////////////////////////////////////////////////////////////////

int main()
{
   enable_device();
   check_whoami(WHO_AM_I, WHO_AM_I_VALUE);
   initialize_device();
   uint32_t i;
   for (i=0; i<50; i++) {
      printf("%3d   %7.3f   ", i, i * 0.000001*UPDATE_INTERVAL_USEC);
      if (read_data()) {
         printf("     %8.3f %8.3f %8.3f  t=%.3f",
               output_[0], output_[1], output_[2], celcius_);
      }
      printf("\n");
      usleep(UPDATE_INTERVAL_USEC);
   }
   close(device_);
   return 0;
}

