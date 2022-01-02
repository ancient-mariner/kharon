#include "sensor_packet.h"
#include <stdio.h>
#include <assert.h>
#include <string.h>
#include <stdlib.h>
#include <sys/time.h>


void print_sensor_data(
      /* in     */ const struct imu_sensor_packet *s
      )
{
   printf("%6.2f,%6.2f,%6.2f,  ", (double) (s->gyr.v[0]), 
            (double) (s->gyr.v[1]), (double) (s->gyr.v[2]));
   printf("%7.3f,%7.3f,%7.3f,  ", (double) (s->acc.v[0]), 
            (double) (s->acc.v[1]), (double) (s->acc.v[2]));
   printf("%6.2f,%6.2f,%6.2f,  ", (double) (s->mag.v[0]), 
            (double) (s->mag.v[1]), (double) (s->mag.v[2]));
   printf("%6.2f,%6.2f,%6.2f,  ", (double) (s->gps.v[0]), 
            (double) (s->gps.v[1]), (double) (s->gps.v[2]));
   printf("%6.2f\n", (double) (s->temp));
   printf("%6.2f\n", (double) (s->baro));
}

void print_sensor_data2(
      /* in     */ const struct imu_sensor_packet *s
      )
{
   printf("%9.4f,%9.4f,%9.4f,  ", s->gyr.v[0], s->gyr.v[1], s->gyr.v[2]);
   printf("%9.4f,%9.4f,%9.4f,  ", s->acc.v[0], s->acc.v[1], s->acc.v[2]);
   printf("%9.4f,%9.4f,%9.4f,  ", s->mag.v[0], s->mag.v[1], s->mag.v[2]);
   printf("%9.4f,%9.4f,%9.4f,  ", s->gps.v[0], s->gps.v[1], s->gps.v[2]);
   printf("%6.2f\n", s->temp);
   printf("%6.2f\n", s->baro);
}

//void print_vector(double vec[3])
//{
//   printf("%7.3f, %7.3f, %7.3f\n", vec[0], vec[1], vec[2]);
//}

void serialize_sensor_packet(
      /* in     */ const struct imu_sensor_packet *s, 
      /*    out */       char serial[SP_SERIAL_LENGTH]
      )
{
   memset(serial, 0, SP_SERIAL_LENGTH);
   uint32_t idx = 0;
   // %.7e -> +1.1234567e+100
   //         123456789012345
#if FLOAT_SERIAL_BYTES < 16
#error "Must have at least 16 bytes for float-text conversion using %.7e"
#endif   // FLOAT_SERIAL_BYTES
   ///////////////////
   // Gyro
   if (s->state.avail[IMU_GYR]) {
      for (uint32_t i=0; i<3; i++) {
         snprintf(&serial[idx], FLOAT_SERIAL_BYTES-1, "%.7e", 
               (double)(s->gyr.v[i]));
         idx += FLOAT_SERIAL_BYTES;
      }
   } else {
      idx += 3 * FLOAT_SERIAL_BYTES;
   }
   ///////////////////
   // Acc
   if (s->state.avail[IMU_ACC]) {
      for (uint32_t i=0; i<3; i++) {
         snprintf(&serial[idx], FLOAT_SERIAL_BYTES-1, "%.7e", 
               (double) (s->acc.v[i]));
         idx += FLOAT_SERIAL_BYTES;
      }
   } else {
      idx += 3 * FLOAT_SERIAL_BYTES;
   }
   ///////////////////
   // Mag
   if (s->state.avail[IMU_MAG]) {
      for (uint32_t i=0; i<3; i++) {
         snprintf(&serial[idx], FLOAT_SERIAL_BYTES-1, "%.7e", 
               (double) (s->mag.v[i]));
         idx += FLOAT_SERIAL_BYTES;
      }
   } else {
      idx += 3 * FLOAT_SERIAL_BYTES;
   }
   ///////////////////
   // GPS
   if (s->state.avail[IMU_GPS]) {
      for (uint32_t i=0; i<3; i++) {
         snprintf(&serial[idx], FLOAT_SERIAL_BYTES-1, "%.7e", 
               (double) (s->gps.v[i]));
         idx += FLOAT_SERIAL_BYTES;
      }
   } else {
      idx += 3 * FLOAT_SERIAL_BYTES;
   }
   ///////////////////
   // temp
   if (s->state.avail[IMU_TEMP]) {
      snprintf(&serial[idx], FLOAT_SERIAL_BYTES-1, "%.7e", (double) (s->temp));
   }
   idx += FLOAT_SERIAL_BYTES;
   ///////////////////
   // baro
   if (s->state.avail[IMU_BARO]) {
      snprintf(&serial[idx], FLOAT_SERIAL_BYTES-1, "%.7e", (double) (s->baro));
   }
   idx += FLOAT_SERIAL_BYTES;
   //
   assert(idx <= SP_SERIAL_LENGTH);
}


void restore_sensor_packet_individual(
      /* in     */ const char serial[SP_SERIAL_LENGTH],
      /*    out */       struct vector_type *gyr, 
      /*    out */       struct vector_type *acc, 
      /*    out */       struct vector_type *mag,
      /*    out */       struct vector_type *gps,
      /*    out */       double * temp,
      /*    out */       double * baro,
      /*    out */       imu_modality_state_type *state
      )
{
   uint32_t idx = 0;
   uint32_t VEC_LEN = 3;
   state->flags = 0; // set availability flags to zero
   //
   if ((gyr) && (serial[idx] != 0)) {
      for (uint32_t i=0; i<VEC_LEN; i++) {
         gyr->v[i] = atof(&serial[idx]);
         idx += FLOAT_SERIAL_BYTES;
      }
      state->avail[IMU_GYR] = 1;
   } else {
      if (gyr) {
         zero_vector(gyr);
      }
      idx += VEC_LEN * FLOAT_SERIAL_BYTES;
   }
   //
   if (acc && serial[idx]) {
      for (uint32_t i=0; i<VEC_LEN; i++) {
         acc->v[i] = atof(&serial[idx]);
         idx += FLOAT_SERIAL_BYTES;
      }
      state->avail[IMU_ACC] = 1;
   } else {
      if (acc) {
         zero_vector(acc);
      }
      idx += VEC_LEN * FLOAT_SERIAL_BYTES;
   }
   //
   if (mag && serial[idx]) {
      for (uint32_t i=0; i<VEC_LEN; i++) {
         mag->v[i] = atof(&serial[idx]);
         idx += FLOAT_SERIAL_BYTES;
      }
      state->avail[IMU_MAG] = 1;
   } else {
      if (mag) {
         zero_vector(mag);
      }
      idx += VEC_LEN * FLOAT_SERIAL_BYTES;
   }
   //
   if (gps && serial[idx]) {
      for (uint32_t i=0; i<VEC_LEN; i++) {
         gps->v[i] = atof(&serial[idx]);
         idx += FLOAT_SERIAL_BYTES;
      }
      state->avail[IMU_GPS] = 1;
   } else {
      if (gps) {
         zero_vector(gps);
      }
      idx += VEC_LEN * FLOAT_SERIAL_BYTES;
   }
   //
   if (temp && serial[idx]) {
      *temp = atof(&serial[idx]);
      idx += FLOAT_SERIAL_BYTES;
      state->avail[IMU_TEMP] = 1;
   } else {
      if (temp) {
         *temp = 0.0f;
      }
      idx += FLOAT_SERIAL_BYTES;
   }
   //
   if (baro && serial[idx]) {
      *baro = atof(&serial[idx]);
      idx += FLOAT_SERIAL_BYTES;
      state->avail[IMU_BARO] = 1;
   } else {
      if (baro) {
         *baro = 0.0f;
      }
      idx += FLOAT_SERIAL_BYTES;
   }
}

void restore_sensor_packet(
      /* in     */ const char serial[SP_SERIAL_LENGTH],
      /*    out */       struct imu_sensor_packet *s
      )
{
   restore_sensor_packet_individual(serial, 
         &s->gyr, &s->acc, &s->mag, &s->gps, &s->temp, &s->baro,
         &s->state);
}

