#if! defined(S2_H)
#define S2_H
#include "pin_types.h"
#include <time.h>

#define SET_VEC(z, a) do { (z)->v[0]=a; (z)->v[1]=a; (z)->v[2]=a; } while (0)

// sensor descriptors
// common fields
//    <name>_addr: i2c address (eg, 0x1D). name included in name to
//          lower probability of copy-paste errors
//
//    confidence: confidence in sensor's accuracy, relative to others.
//          a noisy sensor may have a weight of 20 an a low-noise sensor
//          have a weight of 500. output signal is a weighted average
//          of individual signals. a very low-weight sensor contributes
//          little to the signal, but if the high-weight signal goes
//          offline, the low-weight sensor(s) will become dominant
//          NOTE: for reference, have initial set of pinet sensors
//          have confidence 1.0f
//
//    offset: distance from centroid of sensor's signal from where
//          it should be, on all axes. for gyro, this is 'drift'.
//          NOTE: offset is applied before scale
//
//    gain: multiplicative constant to get raw signal from sensor 
//          into desired units of computation. this is a fixed 
//          constant based on the settings/registers of the sensor.
//          it is hard-coded in each sensor's driver
//
//    scale: multiplicative constant to adjust signal to correct 
//          unit based on characteristic of sensor (eg, scale=1.05
//          if sensor reads 5% low on a given axis). this can 
//          change with time and temperature
//         
 
// delay (in milliseconds) before sensors start streaming data
// have all sensors share this delay so their output can be synchronized
#define WARMUP_INTERVAL_BASE  600

#define SAMPLING_RATE_BASE_US    12500

////////////////////////////////////////////////////////////////
// gyro 


struct sensor_gyro {
   // relative confidence of this sensor relative to others
   double confidence;
   //
   vector_type gain;
   vector_type drift_dps;
   matrix_type axis_alignment;
   //
//   double dt;  // interval between samples
   vector_type axis_dps; // gyro signal in axis-angle representation
   //
   uint8_t gyro_addr;  // i2c address
};
typedef struct sensor_gyro sensor_gyro_type;


////////////////////////////////////////////////////////////////
// accelerometer and magnetometer

struct sensor_accel {
   double confidence;  // relative confidence
   //
   vector_type gain;
   vector_type offset;
   vector_type scale;
   matrix_type axis_alignment;
   //
   vector_type up;      // acc signal (perhaps better called 'down')
   //
   uint8_t accel_addr;  // i2c address for acc
};
typedef struct sensor_accel sensor_accel_type;

struct sensor_mag {
   double confidence;  // relative confidence
   //
   vector_type gain;
   vector_type offset;
   vector_type scale;
   matrix_type softiron;
   matrix_type axis_alignment;
   //
   vector_type mag;   // mag signal (actual signal, not orthogonalanlized)
   //
   uint8_t mag_addr;  // i2c address for mag
};
typedef struct sensor_mag sensor_mag_type;


////////////////////////////////////////////////////////////////
// temperature
struct sensor_temp {
   double confidence;  // relative confidence
   //
   double gain;
//   double offset;
   //
   double celcius;
   //
   uint8_t temp_addr;  // i2c address
};
typedef struct sensor_temp sensor_temp_type;


////////////////////////////////////////////////////////////////
// barometer

struct sensor_baro {
   double confidence;  // relative confidence
   //
   double gain;
//   double offset;
   //
   double mbar;
   //
   uint8_t baro_addr;  // i2c address
};
typedef struct sensor_baro sensor_baro_type;


////////////////////////////////////////////////////////////////
// GPS
struct sensor_gps {
   // TODO FINISH ME
   double confidence;  // relative confidence
   vector_type latlon;
   //
   uint8_t gps_addr;  // i2c address
};
typedef struct sensor_gps sensor_gps_type;

////////////////////////////////////////////////////////////////

// max number of independent sensors in acquisition loop
#define MAX_SENSORS  16u

#define SENSOR_NAME_LEN    32u
#define SENSOR_PATH_LEN    128u


#define SENSOR_FLAG_GYRO         0x00000001
#define SENSOR_FLAG_ACC          0x00000002
#define SENSOR_FLAG_MAG          0x00000004
#define SENSOR_FLAG_TEMP         0x00000008
#define SENSOR_FLAG_GPS          0x00000010 
#define SENSOR_FLAG_BARO         0x00000020
// sensor types can go up to 0x0000ffff
#define SENSOR_FLAG_ANY_SENSOR   0x0000ffff
// no device installed (or active) on this descriptor
#define SENSOR_FLAG_DISABLED     0x40000000
#define SENSOR_FLAG_INACTIVE     0x80000000



#define SENSOR_FLAGS_GYRO_ACC_MAG_TEMP \
      (SENSOR_FLAG_GYRO |     \
      SENSOR_FLAG_ACC |       \
      SENSOR_FLAG_MAG |       \
      SENSOR_FLAG_TEMP)

#define SENSOR_FLAGS_GYRO_ACC_MAG \
      (SENSOR_FLAG_GYRO |     \
      SENSOR_FLAG_ACC |       \
      SENSOR_FLAG_MAG)

#define SENSOR_FLAGS_ACC_MAG_TEMP \
      (SENSOR_FLAG_ACC |      \
      SENSOR_FLAG_MAG |       \
      SENSOR_FLAG_TEMP)

#define SENSOR_FLAGS_MAG_TEMP  (SENSOR_FLAG_MAG | SENSOR_FLAG_TEMP)
#define SENSOR_FLAGS_ACC_TEMP  (SENSOR_FLAG_ACC | SENSOR_FLAG_TEMP)
#define SENSOR_FLAGS_GYRO_TEMP (SENSOR_FLAG_GYRO | SENSOR_FLAG_TEMP)


#define  HARDWARE_NAME_LSM9DS0      "lsm9ds0"
#define  HARDWARE_NAME_LSM303       "lsm303"
#define  HARDWARE_NAME_L3G          "l3g"
#define  HARDWARE_NAME_BMG160       "bmg160"
#define  HARDWARE_NAME_HMC6343      "hmc6343"
#define  HARDWARE_NAME_LIS3MDL      "lis3mdl"   // mag
#define  HARDWARE_NAME_LIS3DH       "lis3dh"    // acc
#define  HARDWARE_NAME_A3G4250D     "a3g4250d"  // gyr
#define  HARDWARE_NAME_MAG3110      "mag3110"

// 
extern const char *HARDWARE_LIST[];


struct sensor_runtime;

// function that's called to initialize a sensor
// initial value of waketime should be the desired delay after
//    s2's acquisition starts before the sensor gets its first
//    update. this will often be 0 or DT
typedef int32_t (*sensor_setup_callback) (
      /* in out */       struct sensor_runtime *sensor
      );

// self-check called after initialization and before runtime
typedef int32_t (*sensor_whoami_callback) (
      /* in out */       struct sensor_runtime *sensor
      );


// function that's called by main data acquisition loop
// sensor indicates when it's ready to be next awoken by setting
//    the 'waketime' value, and it will be called sometime shortly
//    after that time
// there are 2 probable scenarios for setting waketime. one is 
//    incrementing the value by a set amount (eg, 50ms), the other
//    is getting the present time and incrementing it (eg, by 1ms)
// note: a error will be (should be) thrown if the requested wake time is
//    in the past
// to change the time, do something like:
//    struct timespec t;
//    clock_gettime(CLOCK_MONOTONIC, &t)
//    t.tv_sec += 2;    // wait 2 seconds to be woken up again
//    memcpy(when_ready, &t, sizeof t);
// note: if advancing tv_nsec instead (eg, tv_nsec += xxx), make
//    sure to account for situation where nsec is > 1e9
typedef void (*sensor_update_callback) (
      /* in out */       struct sensor_runtime *sensor,
      /*    out */       int32_t *data_available  // 1=yes, 0=no
      );

// function that's called during system shutdown
typedef void (*sensor_shutdown_callback) (
      /* in out */       struct sensor_runtime *sensor
      );

////////////////////////////////////////////////////////////////////////
// interface for sensor. when clock exceeds waketime, the embedded
//    function is called and the code for managing this sensor is
//    executed
struct sensor_runtime {
   struct timespec   waketime;
   struct timespec   last_update;
   int hw_device;
   // callback functions for setup and regular updates
   sensor_update_callback update;
   sensor_whoami_callback self_test;   // nullified after passing
   sensor_setup_callback setup;
   sensor_shutdown_callback shutdown;
   // 
   uint32_t flags;   // control flags for managing hardware
   uint32_t state;   // flags for managing transitions
   ////////////////////////////////////////////
   // don't implement inheritance -- keep data from all sensor types here
   // keep data compartmentalized so that the same object can be
   //    merged with itself without special logic
   sensor_gyro_type     gyro;
   sensor_accel_type    accel;
   sensor_mag_type      mag;
   sensor_temp_type     temp;
   sensor_baro_type     baro;
   sensor_gps_type      gps;
   //
   // data to log in kernel. string cleared when data is sent
   char log_data[SENSOR_PACKET_LOG_DATA];   
   char name[SENSOR_NAME_LEN];   // unique name of sensor on the device
   char type_name[SENSOR_NAME_LEN];    // e.g., lsm9ds0
   char device_addr[SENSOR_NAME_LEN];  // eg, /dev/i2c-1
   //
   // path to config directory. eg, /pinet/dev/<node>/sensors/i2c/<name>/
   char config_root[SENSOR_PATH_LEN];  
};
typedef struct sensor_runtime sensor_runtime_type;


////////////////////////////////////////////////////////////////////////
// stores the merged/fused outputs of multiple sensors (and types)
struct consensus_sensor {
   // merged signals per modality
   vector_type acc;
   vector_type mag;
   vector_type gyr_axis;   // degrees per second
   //
   double temp;
   double baro;
   vector_type latlon;  // eg, from gps
   imu_modality_state_type state;
   //
   char log_data[SENSOR_PACKET_LOG_DATA];
};
typedef struct consensus_sensor consensus_sensor_type;


#endif   // S2_H
