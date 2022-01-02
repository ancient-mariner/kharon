#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <errno.h>
#include <unistd.h>

////////////////////////////////////////////////////////////////////////
#define WHO_AM_I                    0x07
#define WHO_AM_I_VALUE              0xC4

#define DR_STATUS                   0x00
#define OUT_X_MSB                   0x01
#define SYS_MOD                     0x08

#define DIE_TEMP                    0x0F
#define CTRL_REG1                   0x10
#define CTRL_REG2                   0x11
////////////////////////////////////////////////////////////////////////

#define WARMUP_INTERVAL_USEC  (50 * 1000)
#define UPDATE_INTERVAL_USEC  (20 * 1000)

#define ADDR   0x0E

const float SIG_GAIN = 1.0e-5f;
const float TEMP_GAIN = 1.0f;

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
   uint8_t cmd = addr;
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
   // 20Hz, 16x oversample, normal mode, active
   write_register(CTRL_REG1, 0b01000001);
   // no automatic correction, automatic reset
   write_register(CTRL_REG2, 0b10100000);
}

////////////////////////////////////////////////////////////////////////

static void data_available(
      /*    out */       uint8_t *data,
      /* in     */ const uint8_t status_reg
      )
{
   //uint8_t cmd = 0x08 | status_reg;
   uint8_t cmd = status_reg;
   if (i2c_smbus_read_i2c_block_data(device_, cmd, 1, data) < 0) 
   {
      fprintf(stderr, "Error checking if data available\n");
      exit(1);
   }
}

static void read_data(void)
{
   uint8_t raw[6];
   //uint8_t cmd = 0x80 | OUT_X_MSB;
   uint8_t cmd = OUT_X_MSB;
   if (i2c_smbus_read_i2c_block_data(device_, cmd, sizeof(raw), raw) < 0) {
      fprintf(stderr, "Error reading data\n");
      exit(1);
   }
//   printf(" %02x %02x   %02x %02x   %02x %02x\n", raw[0], raw[1], raw[2], raw[3], raw[4], raw[5]);
   // mag data order X Y Z Temp, big endian
   output_[0] = (float) (raw[0] | raw[1] << 8) * SIG_GAIN;
   output_[1] = (float) (raw[2] | raw[3] << 8) * SIG_GAIN;
   output_[2] = (float) (raw[4] | raw[5] << 8) * SIG_GAIN;
   //output_[0] = (float) (raw[1] | raw[0] << 8) * SIG_GAIN;
   //output_[1] = (float) (raw[3] | raw[2] << 8) * SIG_GAIN;
   //output_[2] = (float) (raw[5] | raw[4] << 8) * SIG_GAIN;
}

static int32_t read_temp_data(void)
{
   uint8_t temp;
   uint8_t cmd = 0x80 | DIE_TEMP;
   if (i2c_smbus_read_i2c_block_data(device_, cmd, 1, &temp) < 0) {
      fprintf(stderr, "Error reading temperature\n");
      exit(1);
   }
//printf("TEMP  %0x02x (%d)\n", temp);
   celcius_ = (float) (temp);
   return 0;
}


////////////////////////////////////////////////////////////////////////

int main()
{
   enable_device();
   check_whoami(WHO_AM_I, WHO_AM_I_VALUE);
   initialize_device();
   uint32_t i;
   for (i=0; i<500; i++) {
      uint8_t avail = 0;
      data_available(&avail, DR_STATUS);
      if (avail & 0x08) {
         read_data();
         read_temp_data();
         printf("%d  %.3f %.3f %.3f  t=%.3f\n", i, 
               output_[0], output_[1], output_[2], celcius_);
//      } else {
//         printf("%d  no data available\n", i);
      }
      usleep(UPDATE_INTERVAL_USEC);
   }
   close(device_);
   return 0;
}
