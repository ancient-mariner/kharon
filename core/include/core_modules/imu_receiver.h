#if !defined(IMU_RECEIVER_H)
#define IMU_RECEIVER_H
#if !defined(_GNU_SOURCE)
#define _GNU_SOURCE
#endif   // _GNU_SOURCE
#include "pinet.h"
#include <stdio.h>
#include "datap.h"
#include "time_lib.h"
#include "logger.h"
#include "sensor_packet.h"

// receives and distributes information from primary gyro-acc-mag source
// publishes gyr, acc and mag data in ship space (z forward (bow), y up, 
//    x left (port))
// log data is in same form as was reported from sensor, so it can
//    be re-used as emulated input (ie, to replay an event)
//
// gyro drift (low-pass) filter implemented at device level
// interploated sensor values reported -- data is not otherwise 
//    filtered or fused
// 

/**
IMU receiver logs data as is reported from sensor, so it can be re-used as
emulated input (e.g., to replay data). Some sensor modalities (eg, acc, mag)
don't report every sample. To simplify logging output, repeat previously
reported values for each sample. The log won't be a literal repeat of
what was delivered, but it should represent what was used and, other
than during data outages, it should accurately represent what was delivered

Published gyro data is upsampled 10ms intervals. 

TODO see imu_streams.h for comment about merging att and imu resampling

Time is stored as a double, for
backward compatibility reasons, but to generate the tmestamp an intermediate
value is a cast integer. This means that times can be compared for equality
even though they're floats, but only so long as no operations have
been performed on them (e.g., add 10ms (i.e., +0.01 sec))
**/


#define IMU_PRODUCER_INTERVAL_US      10000  // 10ms

// 512 is ~5 seconds at 100hz
#define IMU_QUEUE_LEN   512

#define IMU_CLASS_NAME  "IMU_receiver"

#define I2C_DEV_TO_SHIP_CONFIG   "i2c_dev2ship"

#define IMU_LOG_LEVEL      LOG_LEVEL_DEFAULT

// pinet.h defines IMU_ACC, IMU_MAG, IMU_GYR, NUM_IMU_CHANNELS

struct imu_output {
   // acc, mag and gyr
   vector_type modality[NUM_IMU_CHANNELS];
   // state is 1 (valid data) or 0 (invalid)
   imu_modality_state_type state;
};
typedef struct imu_output imu_output_type;


// different modalities report at different rates, some notable slower
//    than the producer publication interval. for simplicity of
//    data consumers, previously received data is republished until
//    another sample from the same modality is received. e.g., if a MAG
//    only reports every 50ms, yet data is published every 10ms, the
//    received MAG signal is repeated every 10ms until another MAG signal
//    is received. this has an implicit ability to hide data source 
//    failures, so a timer is needed to let data from a modality disappear 
//    when the sensor is down
// duration that an old sample is reused
// acc and mag aren't time critical as they're primarily used as a slow drift
//    correction for the gyro. don't recycle too long though as 
//    attitude calculator needs to know when that data is stale
#define ACC_RECYCLE_DURATION_USEC      150000
#define MAG_RECYCLE_DURATION_USEC      150000


/**
priority rules
   1  producer data is always used when available
   2  producer data used when available, with 1/2 weight of P1
   3  producer data used when P1 is not available, with same weight as P2
   4  do-not-use
**/
enum { IMU_PRI_1=0, IMU_PRI_2, IMU_PRI_3, IMU_PRI_NULL };

struct imu_class {
   int sockfd, connfd;
   char device_name[MAX_NAME_LEN];
   FILE *logfile;
   log_info_type *log;
   // rotation matrix to/from IMU reference frame to body frame
   matrix_type acc_dev2ship;
   matrix_type mag_dev2ship;
   matrix_type gyr_dev2ship;
   // mag offsets for compass correction (to correct for installation bias)
   // TODO elaborate on concept -- presently bias is assumed to be 
   //    circular but that's not a reliable assumption
   double x_mag_bias;
   double z_mag_bias;
   // the priority for each modality in the IMU stream
   int8_t priority[NUM_IMU_CHANNELS];
   // most recently received data. this is used for building next
   //    output sample (gyr) or for recycling (acc, mag) during upsample
   vector_type recycle_value[NUM_IMU_CHANNELS];
   // duration before acc or mag go stale
   int32_t recycle_timer_usec[NUM_IMU_CHANNELS];  
   microsecond_timestamp_type prev_gyr_data_t;
   // time of previously published sample
   microsecond_timestamp_type prev_publish_t;
};
typedef struct imu_class imu_class_type;

struct imu_setup {
   uint32_t logging;
   char device_name[MAX_NAME_LEN];
   uint8_t priority;
};
typedef struct imu_setup imu_setup_type;

// thread entry point
void * imu_class_init(void *);

//void imu_set_rotation_info(datap_desc_type *dp, const char *cam_name);
//
//const matrix_type * get_alignment_matrix(
//      /*    out */ const imu_class_type *imu
//      );

void module_config_load_imu_to_ship(
      /* in     */ const char *module_name,
      /*    out */       imu_class_type *imu
      );

void set_imu_priority(
      /* in out */       datap_desc_type *imu_dp,
      /* in     */ const uint32_t acc_pri,
      /* in     */ const uint32_t gyr_pri,
      /* in     */ const uint32_t mag_pri
      );

//static microsecond_timestamp_type prev_imu_publish_time(
//      /* in     */ const uint64_t usec
//      );
//
//static microsecond_timestamp_type next_imu_publish_time(
//      /* in     */ const uint64_t usec
//      );

#endif   // IMU_RECEIVER_H

