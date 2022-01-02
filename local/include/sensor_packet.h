#if !defined(SENSOR_PACKET_H)
#define SENSOR_PACKET_H
#include "pinet.h"
#include <stdint.h>
#include "lin_alg.h"

// 16 characters per float value -- 15 signal + 1 NULL
//    15 bytes from use of %.7e:
//       1 for sign
//       2 for "n."
//       7 for digits
//       4 for +/-eNN
//    -> 14 bytes (reserve one extra byte for padding)
// %.7e -> +1.1234567e+100
//         123456789012345
#define FLOAT_SERIAL_BYTES 20 // 16 is necessary, 20 is safer
// 16 values:
//    3 gyr
//    3 acc
//    3 mag
//    3 gps
//    1 temp
//    1 baro
#define SP_SERIAL_LENGTH   (14 * FLOAT_SERIAL_BYTES)

struct imu_data {
   vector_type gyro, acc, mag;
   double temp;
};
typedef struct imu_data imu_data_type;

// print sensor data to stdout
void print_sensor_data(
      /* in     */ const struct imu_sensor_packet *s
      );
void print_sensor_data2(
      /* in     */ const struct imu_sensor_packet *s
      ); // higher precision output

// converts sensor packet into text, for network transport
void serialize_sensor_packet(
      /* in     */ const struct imu_sensor_packet *s, 
      /*    out */       char serial[SP_SERIAL_LENGTH]
      );

// convert sensor packet from network representation to internal
void restore_sensor_packet(
      /* in     */ const char serial[SP_SERIAL_LENGTH],
      /*    out */       struct imu_sensor_packet *s
      );

// convert sensor packet from network representation to internal,
//    storing representation in specified locations
// if data is not desired for a particular channel, provide NULL
void restore_sensor_packet_individual(
      /* in     */ const char serial[SP_SERIAL_LENGTH],
      /*    out */       struct vector_type *gyr, 
      /*    out */       struct vector_type *acc, 
      /*    out */       struct vector_type *mag,
      /*    out */       struct vector_type *gps,
      /*    out */       double * temp,
      /*    out */       double * baro,
      /*    out */       imu_modality_state_type *state

      );

int get_latest_gyro_drift(const char *device, struct vector_type *drift);

int save_gyro_drift(const char* device, const struct vector_type *drift);

// calculates the discrepancy, in degrees, of the gyro signal in curr
//    with the ACC and MAG vector change between prev and curr.
//    The difference, how many degrees the gyro signal is greater than
//    the computed rotation, is stored in err[]
void estimate_gyro_error(const struct imu_sensor_packet *prev,
            const struct imu_sensor_packet *curr, struct vector_type *err);

#endif   // SENSOR_PACKET_H
