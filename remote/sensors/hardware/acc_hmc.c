#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <errno.h>
#include <unistd.h>

#include "hmc6343_registers.h"

#define HARDWARE_NAME   HARDWARE_NAME_HMC6343

#define ADDR 	0x19

#define UPDATE_INTERVAL_USEC  (10 * 1000)

// there's a 5ms lag between reqeusting data and getting it
// data asked for twice (once for acc, 2nd for mag)
// there's also a 5ms lag between getting acc and asking for mag

// NOTE: HMC 6343 requires at least 500ms to initialize

static double output_[6] = { 0 };
static float celcius_;
static int device_;

static const float acc_gain_ = 1.0f / 1044.0f; // estimated
static const float mag_gain_ = 1.0f / 3100.0f; // estimated

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

void get_serial_number(void)
{
   // check WHO_AM_I
   uint8_t data;
   if (i2c_smbus_read_i2c_block_data(device_, 0x06, 1, &data) < 0) 
   {
      fprintf(stderr, "Error reading WHOAMI data\n");
      exit(1);
   }
printf("S/N LSB: %d\n", data);
   if (i2c_smbus_read_i2c_block_data(device_, 0x07, 1, &data) < 0) 
   {
      fprintf(stderr, "Error reading WHOAMI data\n");
      exit(1);
   }
printf("S/N MSB: %d\n", data);
//   if (data != expected) {
//      fprintf(stderr, "WHO_AM_I check failed. Expected 0x%02x, got 0x%02x\n",
//            expected, data);
//      exit(1);
//   }
}

void check_whoami()
{
   // check WHO_AM_I
   //uint8_t cmd = 0x08 | addr;
   uint8_t data;
   if (i2c_smbus_read_i2c_block_data(device_, WHO_AM_I, 1, &data) < 0) 
   {
      fprintf(stderr, "Error reading WHOAMI data\n");
      exit(1);
   }
printf("Who-am-i: %d\n", data);
//   if (data != expected) {
//      fprintf(stderr, "WHO_AM_I check failed. Expected 0x%02x, got 0x%02x\n",
//            expected, data);
//      exit(1);
//   }
}

static int32_t write_register(
      /* in     */ const uint8_t reg, 
      /* in     */ const uint8_t value
      )
{
   if (i2c_smbus_write_byte_data(device_, reg, value) < 0) {
      fprintf(stderr, "%s: write_register() error. reg=%d, value=%d\n", 
            __FILE__, reg, value);
      exit(1);
   }
   return 0;
}

////////////////////////////////////////////////////////////////////////

static int32_t read_acc_data(void)
{
   uint8_t raw[6];
   int16_t data[3];
   uint8_t cmd = 0x33;
   if (i2c_smbus_read_i2c_block_data(device_, cmd, sizeof(raw), raw) < 0) {
      fprintf(stderr, "Error reading acc data\n");
      exit(1);
   }
   // acc ordered X, Y, Z big endian
   data[0] = (int16_t) (raw[1] | raw[0] << 8);
   data[1] = (int16_t) (raw[3] | raw[2] << 8);
   // left-hand system. invert Z to make right hand
   data[2] = (int16_t) (-(raw[5] | raw[4] << 8));
   //
   for (uint32_t i=0; i<3; i++) {
      output_[i] = acc_gain_ * data[i];
   }
   printf(" ACC: %d %d %d\n", data[0], data[1], data[2]);
   return 0;
}


static int32_t read_mag_data(void)
{
   uint8_t raw[6];
   int16_t data[3];
   uint8_t cmd = 0x33;
   if (i2c_smbus_read_i2c_block_data(device_, cmd, sizeof(raw), raw) < 0) {
      fprintf(stderr, "Error reading mag data\n");
      exit(1);
   }
   // mag ordered X, Y, Z big endian
   // inverting signal to match polarity of existing sensors
   data[0] = (int16_t) (-(raw[1] | raw[0] << 8));
   data[1] = (int16_t) (-(raw[3] | raw[2] << 8));
   data[2] = (int16_t) (-(raw[5] | raw[4] << 8));
   //
   for (uint32_t i=0; i<3; i++) {
      output_[i+3] = mag_gain_ * data[i];
   }
   printf(" MAG: %d %d %d\n", data[0], data[1], data[2]);
   return 0;
}

static void init_device(void)
{
}


////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

void hmc6343_update(
      /*    out */       int32_t *data_available
      )
{
   static int state = 0;
   // 2 states
   //    0  send request for attitude, wait 5ms
   //    1  read acc, wait 5
   //    2  request mag, wait 5
   //    3  read mag, wait for next update
   switch (state) {
      case 0:
         state = 1;
         *data_available = 0;
         // request attitude data
         if (write_register(0x32, ACC_XYZ_BIG) != 0)
            fprintf(stderr, "error requesting accelerometer\n");
         break;
      case 1:
         state = 2;
         *data_available = 0;
         if (read_acc_data() != 0)
            fprintf(stderr, "error reading accelerometer\n");
         break;
      case 2:
         state = 3;
         *data_available = 0;
         // request mag data
         if (write_register(0x32, MAG_XYZ_BIG) != 0)
            fprintf(stderr, "error requesting magnetometer\n");
         break;
      case 3:
         state = 0;
         if (read_mag_data() != 0)
            fprintf(stderr, "error reading magnetometer\n");
         *data_available = 1;
         break;
      default:
         fprintf(stderr, "Unknown state in hmc6343: %d\n", state);
         exit(1);
   };
}

////////////////////////////////////////////////////////////////////////
//

int write_byte(uint8_t cmd)
{
   if (i2c_smbus_write_byte_data(device_, 0x32, cmd) < 0) {
      fprintf(stderr, "write error. cmd=0x%02x\n", cmd);
      return -1;
   }
   return 0;
}

int read_byte(uint8_t cmd)
{
   uint8_t raw[1];
   if (i2c_smbus_read_i2c_block_data(device_, cmd, sizeof(raw), raw) < 0) {
      fprintf(stderr, "Foo error\n");
      return -1;
   }
   printf("Value: %d  (0x%2x)\n", raw[0], raw[0]);
   return 0;
}

int processor_reset()
{
   return write_byte(0x82);
}


void foo()
{
   write_register(OP_MODE2, 0b00000010);
   check_whoami();
   //initialize_device();
   usleep(1000000);
   for (uint32_t i=0; i<50; i++) {
      int32_t avail = 0;
      hmc6343_update(&avail);
      if (avail != 0) {
         printf("%d  %.3f %.3f %.3f    %.3f, %.3f, %.3f\n", i, 
               output_[0], output_[1], output_[2],
               output_[3], output_[4], output_[5]);
      }
      usleep(UPDATE_INTERVAL_USEC);
   }
}

int main()
{
   enable_device();
//   usleep(1000);
//   get_serial_number();
   //check_whoami();
   if (processor_reset()) {
      goto end;
   }
end:
   close(device_);
   return 0;
}


