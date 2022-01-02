#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <errno.h>
#include <unistd.h>

////////////////////////////////////////////////////////////////////////
#define WHO_AM_I                    0x0F
#define WHO_AM_I_VALUE              0x3D

#define CTRL_REG1                   0x20
#define CTRL_REG2                   0x21
#define CTRL_REG3                   0x22
#define CTRL_REG4                   0x23
#define CTRL_REG5                   0x24

#define STATUS_REG                  0x27
#define OUT_X_L                     0x28
#define OUT_X_H                     0x29
#define OUT_Y_L                     0x2A
#define OUT_Y_H                     0x2B
#define OUT_Z_L                     0x2C
#define OUT_Z_H                     0x2D

#define TEMP_OUT_L                  0x2E
#define TEMP_OUT_H                  0x2F
////////////////////////////////////////////////////////////////////////

#define WARMUP_INTERVAL_USEC  (50 * 1000)
#define UPDATE_INTERVAL_USEC  (100 * 1000)

#define ADDR   0x1C

const float SIG_GAIN = 1.0f / 6842.0f;
const float TEMP_GAIN = 1.0f / 8.0f;

float output_[3];
float celcius_;
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
   // +/- 4 gauss
   write_register(CTRL_REG2, 0b00000000);
   // ultra-high-resolution mode xy; 20Hz; temp enabled
   write_register(CTRL_REG1, 0b11110100);
   // 'ultra-high-performance mode' z; little endian
   write_register(CTRL_REG4, 0b00001100);
   // continuous conversion mode
   write_register(CTRL_REG3, 0b00000000);
   // output registers not updated until MSb and LSb have been read
   write_register(CTRL_REG5, 0b01000000);
}

////////////////////////////////////////////////////////////////////////

static int32_t read_data(void)
{
   uint8_t cmd;
   uint8_t raw[6];
   ///////////////////////////////////////////////////////////////
   cmd = 0x80 | STATUS_REG;
   if (i2c_smbus_read_i2c_block_data(device_, cmd, 1, raw) < 0) 
   {
      fprintf(stderr, "Error checking if data available\n");
      exit(1);
   }
//printf("-- STATUS_REG: 0x%02x\n", raw[0]);
   if (raw[0] & 0x0f) {
      cmd = 0x80 | OUT_X_L;
      if (i2c_smbus_read_i2c_block_data(device_, cmd, sizeof(raw), raw) < 0) {
         fprintf(stderr, "Error reading acc data\n");
         exit(1);
      }
      output_[0] = (float) ((int16_t) (raw[0] | raw[1] << 8)) * SIG_GAIN;
      output_[1] = (float) ((int16_t) (raw[2] | raw[3] << 8)) * SIG_GAIN;
      output_[2] = (float) ((int16_t) (raw[4] | raw[5] << 8)) * SIG_GAIN;
   } else {
      goto no_data;
   }
   //
   cmd = 0x80 | TEMP_OUT_L;
   if (i2c_smbus_read_i2c_block_data(device_, cmd, 2, raw) < 0) {
      fprintf(stderr, "Error reading temp data\n");
      exit(1);
   }
   int16_t temp = (int16_t) (raw[0] | raw[1] << 8);
//printf("Temp: 0x%04x\n", temp);
   //celcius_ = (float) ((int16_t) (raw[0] | raw[1] << 8)) * TEMP_GAIN;
   celcius_ = 25.0f + (float) temp * TEMP_GAIN;
   //////////////////////
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
   usleep(WARMUP_INTERVAL_USEC);
   uint32_t i;
   double t = 0.0;
   for (i=0; i<30; i++) {
      if (read_data()) {
         //printf("%d     %.3f\n", i, i * 0.000001*UPDATE_INTERVAL_USEC);
         printf("%.3f  %.3f %.3f %.3f  t=%.3f\n", t,
               output_[0], output_[1], output_[2], celcius_);
      }
      usleep(UPDATE_INTERVAL_USEC);
      t += 0.000001 * UPDATE_INTERVAL_USEC;
   }
   printf("End time: %.3f\n", t);
   close(device_);
   return 0;
}

