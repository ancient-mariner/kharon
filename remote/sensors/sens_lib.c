#include "sens_lib.h"
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include "lin_alg.h"
#include "pinet.h"
#include "softiron.h"

// when fast drift detection enabled, the time constant for the 
//    high-pass filter is increased by the below multiplier, greatly
//    reducing the time necessary to reach equilibrium. this is
//    meant to be used in initial calibration
// sample time constant is 900 seconds. 900/45=20
static int gyro_fast_drift_detection_enabled_ = 0;
#define TAU_MULTIPLIER     45

void enable_fast_gyro_drift_detection(void)
{
   printf("Gyro drift detection enabled\n");
   gyro_fast_drift_detection_enabled_ = 1;
}

int is_fast_drift_detection_enabled(void)  
{ 
   return gyro_fast_drift_detection_enabled_;
}

////////////////////////////////////////////////////////////////////////

#if defined(INTEL)

int i2c_smbus_write_byte_data(int device, uint8_t cmd, uint8_t val)
{
   (void) device;
   (void) cmd;
   (void) val;
   fprintf(stderr, "Compiled with INTEL flag -- no valid i2c library\n");
   hard_exit("sens_lib.c::i2c_smbus_write_byte_data", 1);
   return -1;
}

int i2c_smbus_read_i2c_block_data(int device, uint8_t cmd, size_t sz, void *raw)
{
   (void) device;
   (void) cmd;
   (void) sz;
   (void) raw;
   fprintf(stderr, "Compiled with INTEL flag -- no valid i2c library\n");
   hard_exit("sens_lib.c::i2c_smbus_read_i2c_block_data", 1);
   return -1;
}
#endif   // INTEL

// if device failure fails, the subsequent attempt to use the device
//    will fail. 
void select_device(
      /* in     */ const sensor_runtime_type *dev,
      /* in     */ const int addr
      )
{
   if (ioctl(dev->hw_device, I2C_SLAVE, addr) < 0) {
      perror("Failure selecting device");
      fprintf(stderr, "device address: 0x%02x\n", addr);
   }
}


int32_t enable_device(
      /* in out */       sensor_runtime_type *dev,
      /* in     */ const char *hw_name
      )
{
   // sanity check
   if (strcmp(dev->type_name, hw_name) != 0) {
      fprintf(stderr, "Wrong device detected in driver - '%s'\n",
            dev->type_name);
      goto err;
   }
   if ((dev->hw_device = open(dev->device_addr, O_RDWR)) < 0) {
      fprintf(stderr, "Failure accessing %s\n", hw_name);
      fprintf(stderr, "    %s\n", dev->device_addr);
      fprintf(stderr, "    %s\n", dev->type_name);
      goto err;
   }
   return 0;
err:
   dev->flags |= SENSOR_FLAG_DISABLED;
   return -1;
}

// handle device error
// reason for error is unknown, as is severity. disable the modality
//    where the error occurred, and inactivate the sensor if there
//    are no active modalities. the sensor driver will shutdown 
//    when it has insufficient sensors running. perhaps an app
//    restart will fix the problem
// TODO develop protocol to induce hard-reboot of device. that may be
//    necessary to fix some problems
void device_error(
      /* in out */       sensor_runtime_type *dev,
      /* in     */ const char *src_file,
      /* in     */ const int line_num,
      /* in     */ const int reg,
      /* in     */ const uint32_t flag
      )
{
   // report problem
   fprintf(stderr, "WARNING Sensor communication error: %s:%d  reg=0x%02x\n", 
         src_file, line_num, reg);
   // disable device
   fprintf(stderr, "        Disabling channel 0x%04x from %s : %s\n", 
         (flag & SENSOR_FLAG_ANY_SENSOR), dev->device_addr, dev->type_name);
   dev->flags &= ~flag;
   if ((dev->flags & SENSOR_FLAG_ANY_SENSOR) == 0) {
      // no active devices running on this sensor -- deactivate it
      fprintf(stderr, "        Disabling entire device\n");
      dev->flags |= SENSOR_FLAG_DISABLED;
   }
}


////////////////////////////////////////////////////////////////////////
// sensor block initialization


////////////////////////////////////////////////////////////////
// gyro
void init_sensor_gyro(
      /*    out */       sensor_gyro_type *gyro,
      /* in     */ const vector_type *gain,
      /* in     */ const double confidence
      )
{
   if (gain != NULL)
      copy_vector(gain, &gyro->gain);
   gyro->confidence = confidence;
   SET_VEC(&gyro->axis_dps, 0.0);
   identity_matrix(&gyro->axis_alignment);
}

void init_sensor_gyro_null(
      /*    out */       sensor_gyro_type *gyro
      )
{
   gyro->confidence = 0.0;
   SET_VEC(&gyro->gain, 0.0);
   SET_VEC(&gyro->drift_dps, 0.0);
   SET_VEC(&gyro->axis_dps, 0.0);
   identity_matrix(&gyro->axis_alignment);
}

void update_gyro_drift(
      /* in out */       sensor_runtime_type *dev,
      /* in     */ const int32_t accum[3],
      /* in     */ const double base_tau
      )
{
   vector_type *drift_dps = &dev->gyro.drift_dps;
   vector_type *axis_dps = &dev->gyro.axis_dps;
   vector_type *gain = &dev->gyro.gain;
   // store rotation in axis, converting acquired signal to deg-per-sec
   for (uint32_t i=0; i<3; i++) {
      axis_dps->v[i] = ((double) accum[i]) * gain->v[i];
   }
   // increase time constant with high rates of rotation, as these 
   //    usually due motion and not drift
   double dps = vector_len(axis_dps);
   double turn_scale = dps > 1.0 ? 1.0 / dps : 1.0;
   const double tau = turn_scale * (gyro_fast_drift_detection_enabled_ ? 
         TAU_MULTIPLIER * base_tau : base_tau);
   // drift estimate calcualted by taking long-term average of sensor output
   for (uint32_t i=0; i<3; i++) {
      drift_dps->v[i] = (1.0 - tau) * drift_dps->v[i] + tau * axis_dps->v[i];
   }
   // subtract drift estimate from signal
   for (uint32_t i=0; i<3; i++) {
      axis_dps->v[i] -= drift_dps->v[i];
   }
}


////////////////////////////////////////////////////////////////
// accel & mag
void init_sensor_accel(
      /*    out */       sensor_accel_type *accel,
      /* in     */ const vector_type *gain,
      /* in     */ const double confidence
      )
{
   accel->confidence = confidence;
   if (gain != NULL)
      copy_vector(gain, &accel->gain);
   SET_VEC(&accel->up, 0.0f);
   SET_VEC(&accel->scale, 1.0f);
   identity_matrix(&accel->axis_alignment);
}

void init_sensor_mag(
      /*    out */       sensor_mag_type *mag,
      /* in     */ const vector_type *gain,
      /* in     */ const double confidence
      )
{
   mag->confidence = confidence;
   if (gain != NULL)
      copy_vector(gain, &mag->gain);
   SET_VEC(&mag->mag, 0.0);
   SET_VEC(&mag->scale, 1.0);
   identity_matrix(&mag->axis_alignment);
}

void init_sensor_accel_null(
      /*    out */       sensor_accel_type *accel
      )
{
   accel->confidence = 0.0;
   SET_VEC(&accel->up, 0.0);
   SET_VEC(&accel->gain, 0.0);
   SET_VEC(&accel->offset, 0.0);
   SET_VEC(&accel->scale, 1.0);
   identity_matrix(&accel->axis_alignment);
}

void init_sensor_mag_null(
      /*    out */       sensor_mag_type *mag
      )
{
   mag->confidence = 0.0;
   SET_VEC(&mag->mag, 0.0);
   SET_VEC(&mag->gain, 0.0);
   SET_VEC(&mag->offset, 0.0);
   SET_VEC(&mag->scale, 1.0);
   identity_matrix(&mag->softiron);
   identity_matrix(&mag->axis_alignment);
}


////////////////////////////////////////////////////////////////
// temp
void init_sensor_temp(
      /*    out */       sensor_temp_type *temp,
      /* in     */ const double *gain,
      /* in     */ const double confidence
      )
{
   temp->confidence = confidence;
   if (gain != NULL)
      temp->gain = *gain;
   temp->celcius = 0.0;
}

void init_sensor_temp_null(
      /*    out */       sensor_temp_type *temp
      )
{
   temp->confidence = 0.0;
   temp->gain = 0.0;
   temp->celcius = 0.0;
}

////////////////////////////////////////////////////////////////
// baro
void init_sensor_baro(
      /*    out */       sensor_baro_type *baro,
      /* in     */ const double *gain,
      /* in     */ const double confidence
      )
{
   baro->confidence = confidence;
   if (gain != NULL)
      baro->gain = *gain;
   baro->mbar = 1013.0;
}

void init_sensor_baro_null(
      /*    out */       sensor_baro_type *baro
      )
{
   baro->confidence = 0.0;
   baro->gain = 0.0;
   baro->mbar = 0.0;
}

////////////////////////////////////////////////////////////////
// gps
void init_sensor_gps(
      /*    out */       sensor_gps_type *gps,
      /* in     */ const double confidence
      )
{
   gps->confidence = confidence;
   SET_VEC(&gps->latlon, 0.0);
}

void init_sensor_gps_null(
      /*    out */       sensor_gps_type *gps
      )
{
   gps->confidence = 0.0;
   SET_VEC(&gps->latlon, 0.0);
}

// sensor block initialization
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
// sensor runtime

//void write_acc_data(
//      /* in     */ const int16_t data[3],
//      /* in out */       sensor_runtime_type *sensor
//      )
//{
//   // TODO apply correction data and store in sensor struct
//}
//
//void write_mag_data(
//      /* in     */ const int16_t data[3],
//      /* in out */       sensor_runtime_type *sensor
//      )
//{
//   // TODO apply correction data and store in sensor struct
//}
//
//void write_gyr_data(
//      /* in     */ const int16_t data[3],
//      /* in out */       sensor_runtime_type *sensor
//      )
//{
//   // TODO apply correction data and store in sensor struct
//}

// sensor runtime
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
// combining sensors

// applies offset and scale 
static void apply_acc_corrections(
      /* in out */       sensor_runtime_type *sensor,
      /*    out */       vector_type *value
      )
{
   const vector_type *offset = &sensor->accel.offset;
   const vector_type *scale = &sensor->accel.scale;
   for (uint32_t i=0; i<3; i++) {
      // subtract offset and multiply by scale
      value->v[i] = (value->v[i] - offset->v[i]) * scale->v[i];
   }
//print_vec(corrected, "corrected");
}

// make weighted average of most recent signal from all acc sensors
void update_acc(
      /* in out */       sensor_runtime_type *sensor_stack,
      /*    out */       consensus_sensor_type *consensus
      )
{
   double confidence = 0.0;
   vector_type up_accum;
   zero_vector(&up_accum);
   // sum weighted signals from all accel sensors
   for (uint32_t i=0; i<MAX_SENSORS; i++) {
      sensor_runtime_type *sensor = &sensor_stack[i];
      const uint32_t flags = sensor->flags;
      if (flags & SENSOR_FLAG_INACTIVE)
         continue;
      if (flags & SENSOR_FLAG_ACC) {
         const double conf = sensor->accel.confidence;
         confidence += conf;
         // realign axes of input data and copy to accumulator
         vector_type vec;
         mult_matrix_vector(&sensor->accel.axis_alignment, 
               &sensor->accel.up, &vec);
         // apply offset and scale corrections
         apply_acc_corrections(sensor, &vec);
         //
         up_accum.v[0] += conf * vec.v[0];
         up_accum.v[1] += conf * vec.v[1];
         up_accum.v[2] += conf * vec.v[2];
         if ((sensor->log_data[0] != 0) && (consensus->log_data[0] == 0)) {
            // if this sensor has data to log, and log slot is open
            //    in consensus object, send it
            strcpy(consensus->log_data, sensor->log_data);
            sensor->log_data[0] = 0;
         }
         consensus->state.avail[IMU_ACC] = 1;
      }
   }
   // store weighted average in accel slot of consensus
   confidence = 1.0 / confidence;
   consensus->acc.v[0] = up_accum.v[0] * confidence;
   consensus->acc.v[1] = up_accum.v[1] * confidence;
   consensus->acc.v[2] = up_accum.v[2] * confidence;
}

// applies hard-iron and soft-iron corrections
static void apply_mag_corrections(
      /* in out */       sensor_runtime_type *sensor,
      /*    out */       vector_type *value
      )
{
   vector_type tmp;
   copy_vector(value, &tmp);
   // 'softiron' also handles hard iron offset
   apply_softiron_correction(&sensor->mag.softiron, &sensor->mag.scale, 
         &sensor->mag.offset, &tmp, value);
}

// make weighted average of most recent signal from all mag sensors
void update_mag(
      /* in out */       sensor_runtime_type *sensor_stack,
      /*    out */       consensus_sensor_type *consensus
      )
{
   double confidence = 0.0;
   vector_type mag_accum;
   zero_vector(&mag_accum);
   // sum weighted signals from all mag sensors
   for (uint32_t i=0; i<MAX_SENSORS; i++) {
      sensor_runtime_type *sensor = &sensor_stack[i];
      const uint32_t flags = sensor->flags;
      if (flags & SENSOR_FLAG_INACTIVE)
         continue;
      if (flags & SENSOR_FLAG_MAG) {
         const double conf = sensor->mag.confidence;
         confidence += conf;
         // realign axes of input data and copy to accumulator
         vector_type vec;
         mult_matrix_vector(&sensor->mag.axis_alignment,
               &sensor->mag.mag, &vec);
         // apply hard and soft iron corrections
         apply_mag_corrections(sensor, &vec);
         // 
         mag_accum.v[0] += conf * vec.v[0];
         mag_accum.v[1] += conf * vec.v[1];
         mag_accum.v[2] += conf * vec.v[2];
         if ((sensor->log_data[0] != 0) && (consensus->log_data[0] == 0)) {
            // if this sensor has data to log, and log slot is open
            //    in consensus object, send it
            strcpy(consensus->log_data, sensor->log_data);
            sensor->log_data[0] = 0;
         }
         consensus->state.avail[IMU_MAG] = 1;
      }
   }
   // store weighted average in mag slot of consensus
   confidence = 1.0 / confidence;
   consensus->mag.v[0] = mag_accum.v[0] * confidence;
   consensus->mag.v[1] = mag_accum.v[1] * confidence;
   consensus->mag.v[2] = mag_accum.v[2] * confidence;
}


// make weighted average of most recent signal from all acc/mag sensors
void update_temp(
      /* in out */       sensor_runtime_type *sensor_stack,
      /*    out */       consensus_sensor_type *consensus
      )
{
   double confidence = 0.0;
   double temp = 0.0;
   // sum weighted signals from all temp sensors
   for (uint32_t i=0; i<MAX_SENSORS; i++) {
      sensor_runtime_type *sensor = &sensor_stack[i];
      const uint32_t flags = sensor->flags;
      if (flags & SENSOR_FLAG_INACTIVE)
         continue;
      if (flags & SENSOR_FLAG_TEMP) {
         const double conf = sensor->temp.confidence;
         confidence += conf;
         temp += conf * sensor->temp.celcius;
         if ((sensor->log_data[0] != 0) && (consensus->log_data[0] == 0)) {
            // if this sensor has data to log, and log slot is open
            //    in consensus object, send it
            strcpy(consensus->log_data, sensor->log_data);
            sensor->log_data[0] = 0;
         }
         consensus->state.avail[IMU_TEMP] = 1;
      }
   }
   if (confidence > 0.0) {
      // compute weighted average
      temp /= confidence;
   }
   consensus->temp = temp;
}


// make weighted average of most recent signal from all gyros
void update_gyr(
      /* in out */       sensor_runtime_type *sensor_stack,
      /*    out */       consensus_sensor_type *consensus
      )
{
   double confidence = 0.0;
   vector_type axis_dps;
   zero_vector(&axis_dps);
   // sum weighted signals from all accel sensors
   for (uint32_t i=0; i<MAX_SENSORS; i++) {
      sensor_runtime_type *sensor = &sensor_stack[i];
      const uint32_t flags = sensor->flags;
      if (flags & SENSOR_FLAG_INACTIVE)
         continue;
      if (flags & SENSOR_FLAG_GYRO) {
         const sensor_gyro_type *gyro = &sensor->gyro;
         const double conf = gyro->confidence;
         confidence += conf;
         // realign axes of input data and copy to accumulator
         vector_type vec;
         mult_matrix_vector(&gyro->axis_alignment, 
               &gyro->axis_dps, &vec);
         axis_dps.v[0] += conf * vec.v[0];
         axis_dps.v[1] += conf * vec.v[1];
         axis_dps.v[2] += conf * vec.v[2];
         if ((sensor->log_data[0] != 0) && (consensus->log_data[0] == 0)) {
            // if this sensor has data to log, and log slot is open
            //    in consensus object, send it
            strcpy(consensus->log_data, sensor->log_data);
            sensor->log_data[0] = 0;
         }
         consensus->state.avail[IMU_GYR] = 1;
      }
   }
   // store weighted average in gyro slot of consensus
   confidence = 1.0 / confidence;
   consensus->gyr_axis.v[0] = axis_dps.v[0] * confidence;
   consensus->gyr_axis.v[1] = axis_dps.v[1] * confidence;
   consensus->gyr_axis.v[2] = axis_dps.v[2] * confidence;
}

// combining sensors
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
// utility functions

// applies gain, offset and scale (for acc amd mag)
void apply_gain(
      /* in     */ const int16_t raw[3],
      /* in     */ const vector_type *gain,
      /*    out */       vector_type *corrected
      )
{
   for (uint32_t i=0; i<3; i++) {
      corrected->v[i] = gain->v[i] * ((double) raw[i]);
   }
//print_vec(corrected, "corrected");
}

//// applies gain, offset and scale (for acc amd mag)
//void apply_gain_scale_offset(
//      /* in     */ const int16_t raw[3],
//      /* in     */ const vector_type *gain,
//      /* in     */ const vector_type *scale,
//      /* in     */ const vector_type *offset,
//      /*    out */       vector_type *corrected
//      )
//{
//   for (uint32_t i=0; i<3; i++) {
//      // apply gain, then subtract offset and multiply by scale
//      const double sig = gain->v[i] * ((double) raw[i]);
//      corrected->v[i] = (sig - offset->v[i]) * scale->v[i];
//   }
////print_vec(corrected, "corrected");
//}

