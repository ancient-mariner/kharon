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
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <errno.h>
#include <unistd.h>

////////////////////////////////////////////////////////////////////////
#define STATUS_REG_AUX  0x07  // for temp
#define OUT_ADC3_L      0x0C  // for temp
#define OUT_ADC3_H      0x0D  // for temp

#define WHO_AM_I        0x0F
#define WHO_AM_I_VALUE  0x33

#define CTRL_REG0       0x1E
#define TEMP_CFG_REG    0x1F
#define CTRL_REG1       0x20
#define CTRL_REG2       0x21
#define CTRL_REG3       0x22
#define CTRL_REG4       0x23
#define CTRL_REG5       0x24
#define CTRL_REG6       0x25

#define REFERENCE       0x26
#define STATUS_REG      0x27
#define OUT_X_L         0x28
#define OUT_X_H         0x29
#define OUT_Y_L         0x2A
#define OUT_Y_H         0x2B
#define OUT_Z_L         0x2C
#define OUT_Z_H         0x2D

#define FIFO_CTRL_REG   0x2E
#define FIFO_SRC_REG    0x2F
////////////////////////////////////////////////////////////////////////

#define WARMUP_INTERVAL_USEC  (50 * 1000)
#define UPDATE_INTERVAL_USEC  (10 * 1000)

#define ADDR   0x18

// TODO this should be 1/1000, not 1/16k
const float SIG_GAIN = 1.0f / 16000.0f;
const float TEMP_GAIN = 1.0f / 0.0f;

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
   write_register(TEMP_CFG_REG, 0b01000000);
   // 25Hz, xyz enabled, normal/hi-res mode
   write_register(CTRL_REG1, 0b00110111);
   // no filters
   write_register(CTRL_REG2, 0b00000000);
   // registers not updated until read; +/-2G, high-res output mode
   write_register(CTRL_REG4, 0b10001000);
   // no FIFO
//   write_register(FIFO_CTRL_REG, 0b10000000);
}

////////////////////////////////////////////////////////////////////////

static int32_t read_data(void)
{
   uint8_t cmd;
   uint8_t raw[6];
   int32_t accum[3] = { 0, 0, 0};
   uint32_t i, j;
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
      goto done;
   } else {
      goto no_data;
   }

   cmd = 0x80 | FIFO_SRC_REG;
   if (i2c_smbus_read_i2c_block_data(device_, cmd, 1, raw) < 0)
   {
      fprintf(stderr, "Error checking if data available\n");
      exit(1);
   }
printf("-- FIFO SRC_REG: 0x%02x\n", raw[0]);
   uint32_t n = raw[0] & 0x1f;
   if (n == 0)
      goto no_data;
   cmd = 0x80 | OUT_X_L;
   for (i=0; i<n; i++) {
      if (i2c_smbus_read_i2c_block_data(device_, cmd, sizeof(raw), raw) < 0) {
         fprintf(stderr, "Error reading acc data\n");
         exit(1);
      }
for (j=0; j<6; j++) {
   printf(" 0x%02x", raw[j]);
}
printf("\n");
      accum[0] += (int16_t) (raw[0] | raw[1] << 8);
      accum[1] += (int16_t) (raw[2] | raw[3] << 8);
      accum[2] += (int16_t) (raw[4] | raw[5] << 8);
//printf("  %d    %d,%d,%d\n", i, accum[0], accum[1], accum[2]);
   }
   for (i=0; i<3; i++) {
      output_[i] = (float) (accum[i] / n) * SIG_GAIN;
      printf("  %.3f", output_[i]);
   }
   printf("\n");
   //
   cmd = 0x80 | OUT_ADC3_L;
   if (i2c_smbus_read_i2c_block_data(device_, cmd, 2, raw) < 0) {
      fprintf(stderr, "Error reading temp data\n");
      exit(1);
   }
printf("Raw temp: 0x%02x 0x%02x\n", raw[0], raw[1]);
   celcius_ = (float) ((int16_t) (raw[0] | raw[1] << 8)) * TEMP_GAIN;
done:
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
      printf("%d     %.3f\n", i, i * 0.000001*UPDATE_INTERVAL_USEC);
      uint8_t avail = 0;
      if (read_data()) {
         printf("%d  %.3f %.3f %.3f  t=%.3f\n", i,
               output_[0], output_[1], output_[2], celcius_);
      }
      usleep(UPDATE_INTERVAL_USEC);
   }
   close(device_);
}
