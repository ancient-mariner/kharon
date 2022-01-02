#if !defined(SENS_LIB_H)
#define  SENS_LIB_H
#include "pin_types.h"
#include "s2.h"

// gyro drift detection
// this is disabled by default, with detection moved centrally
// the functionality persists here to allow a device's drift to
//    be measured to allow a base correction (dev/.../drift_dps)
//    to be applied on acquisition
//#define DRIFT_TIME_CONST_SEC  30.0f

void enable_fast_gyro_drift_detection(void);
int is_fast_drift_detection_enabled(void);

////////////////////////////////////////////////////////////////////////

#if defined(INTEL)
// to allow compilation of ARM code on INTEL
int i2c_smbus_write_byte_data(int device, uint8_t cmd, uint8_t val);
int i2c_smbus_read_i2c_block_data(int device, uint8_t cmd, size_t sz, void *raw);
#endif   // INTEL

// prepares device for subsequent command
void select_device(
      /* in     */ const sensor_runtime_type *dev,
      /* in     */ const int addr
      );

// opens pipe with device
int32_t enable_device(
      /* in out */       sensor_runtime_type *dev,
      /* in     */ const char *hw_name
      );

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
      /* in     */ const int cmd,
      /* in     */ const uint32_t flag
      );


////////////////////////////////////////////////////////////////////////
// initialize sensor blocks

// gyro initializers (data and null)
// if gain is to be set separately, supply NULL
void init_sensor_gyro(
      /*    out */       sensor_gyro_type *gyro,
      /* in     */ const vector_type *gain,
      /* in     */ const double confidence
      );
void init_sensor_gyro_null(
      /*    out */       sensor_gyro_type *gyro
      );

// accel and mag initializers
// if gains are to be set separately, supply NULL
void init_sensor_accel(
      /*    out */       sensor_accel_type *accel,
      /* in     */ const vector_type *acc_gain,
      /* in     */ const double confidence
      );

void init_sensor_mag(
      /*    out */       sensor_mag_type *mag,
      /* in     */ const vector_type *mag_gain,
      /* in     */ const double confidence
      );

void init_sensor_accel_null(
      /*    out */       sensor_accel_type *accel
      );

void init_sensor_mag_null(
      /*    out */       sensor_mag_type *mag
      );

// temp initializers
// if gain is to be set separately, supply NULL
void init_sensor_temp(
      /*    out */       sensor_temp_type *temp,
      /* in     */ const double *gain,
      /* in     */ const double confidence
      );
void init_sensor_temp_null(
      /*    out */       sensor_temp_type *temp
      );

// barometer initializers
// if gain is to be set separately, supply NULL
void init_sensor_baro(
      /*    out */       sensor_baro_type *baro,
      /* in     */ const double *gain,
      /* in     */ const double confidence
      );
void init_sensor_baro_null(
      /*    out */       sensor_baro_type *baro
      );

// GPS initialiers
void init_sensor_gps(
      /*    out */       sensor_gps_type *gps,
      /* in     */ const double confidence
      );
void init_sensor_gps_null(
      /*    out */       sensor_gps_type *gps
      );

////////////////////////////////////////////////////////////////////////
// writing raw data

//// take raw sensor data and write it to sensor block
//void write_acc_data(
//      /* in     */ const int16_t data[3],
//      /* in out */       sensor_runtime_type *sensor
//      );
//
//void write_mag_data(
//      /* in     */ const int16_t data[3],
//      /* in out */       sensor_runtime_type *sensor
//      );
//
//void write_gyr_data(
//      /* in     */ const int16_t data[3],
//      /* in out */       sensor_runtime_type *sensor
//      );

////////////////////////////////////////////////////////////////////////
// combining sensor data as consensus data

// make weighted average of most recent signal from all acc sensors
void update_acc(
      /* in out */       sensor_runtime_type *sensor_stack,
      /*    out */       consensus_sensor_type *consensus
      );

// make weighted average of most recent signal from all mag sensors
void update_mag(
      /* in out */       sensor_runtime_type *sensor_stack,
      /*    out */       consensus_sensor_type *consensus
      );

// make weighted average of most recent signal from all gyros
void update_gyr(
      /* in out */       sensor_runtime_type *sensor_stack,
      /*    out */       consensus_sensor_type *consensus
      );
 
// 
void update_temp(
      /* in out */       sensor_runtime_type *sensor_stack,
      /*    out */       consensus_sensor_type *consensus
      );
 
//void update_attitude_by_gyro(
//      /*    out */       consensus_sensor_type *consensus,
//      /* in     */ const double gyro_init
//      );


void update_gyro_drift(
      /* in out */       sensor_runtime_type *dev,
      /* in     */ const int32_t accum[3],
      /* in     */ const double tau
      );

////////////////////////////////////////////////////////////////////////
// utility functions

// applies gain 
void apply_gain(
      /* in     */ const int16_t raw[3],
      /* in     */ const vector_type *gain,
      /*    out */       vector_type *corrected
      );

//// applies gain, offset and scale (for acc amd mag)
//void apply_gain_scale_offset(
//      /* in     */ const int16_t raw[3],
//      /* in     */ const vector_type *gain,
//      /* in     */ const vector_type *scale,
//      /* in     */ const vector_type *offset,
//      /*    out */       vector_type *corrected
//      );

#endif   // SENS_LIB_H

