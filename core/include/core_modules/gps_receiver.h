#if !defined(GPS_RECEIVER_H)
#define GPS_RECEIVER_H
#if !defined(_GNU_SOURCE)
#define _GNU_SOURCE
#endif   // _GNU_SOURCE
#include "pinet.h"
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include "datap.h"
#include "time_lib.h"
#include "logger.h"
#include "sensor_packet.h"

// receives and distributes information from networked GPS source
// GPS data received from network is expected to be NMEA sentences

// GPS receiver logs data as is reported from sensor so it can be 
//    re-used as emulated input (e.g., to replay data). just like IMU data

// GPS input is fed on the order of one or a few packets per second
// we don't need a lot of data, but err on the side of too much, esp.
//    as these packets are pretty small. once more data is available
//    on actual feed rates this can be adjusted (or not)
#define GPS_RECEIVER_QUEUE_LEN   128

#define GPS_RECEIVER_CLASS_NAME  "gps_receiver"

#define GPS_RECEIVER_LOG_LEVEL      LOG_LEVEL_DEFAULT


// size of data block sent by GPS process, storing the NMEA sentence(s).
// if content is less than block size then the remaining bytes are zero
//#define GPS_BLOCK_SIZE     256 -> moved to pinet.h
// format for GPS block
//       char[]    timestamp
//       <one or more spaces>
//       char[]    1 NMEA sentence


// TODO publish data using gps reader's output struct, of vice versa
struct gps_receiver_output {
   world_coordinate_type pos;
   degree_type heading;    // should be used only if attitude unavailable
   meter_per_second_type speed;
   // availability flags
   uint32_t available;
#define GPS_REC_AVAILABLE_LATITUDE      0x0001
#define GPS_REC_AVAILABLE_LONGITUDE     0x0002
#define GPS_REC_AVAILABLE_TRACK         0x0004
#define GPS_REC_AVAILABLE_SPEED         0x0008
#define GPS_REC_AVAILABLE_TIME          0x0010
#define GPS_REC_AVAILABLE_DATE          0x0020
   // date and time are store in semi-raw form
   // date is ddmmyy
   // time is hhmmss.sss
   uint32_t zulu_date;
   double zulu_time;
   // nmea message type
   char message_type[8];
};
typedef struct gps_receiver_output gps_receiver_output_type;

#define GPS_REC_MIN_DATA_FOR_PUBLISH      (  \
      GPS_REC_AVAILABLE_LATITUDE    | \
      GPS_REC_AVAILABLE_LONGITUDE   | \
      GPS_REC_AVAILABLE_TIME)

#define GPS_REC_AVAILABLE_LATLON_MASK     (  \
      GPS_REC_AVAILABLE_LATITUDE | GPS_REC_AVAILABLE_LONGITUDE)


struct gps_receiver_class {
   int sockfd, connfd;
   char device_name[MAX_NAME_LEN];
   FILE *logfile;
   log_info_type *log;
};
typedef struct gps_receiver_class gps_receiver_class_type;


struct gps_receiver_setup {
   uint32_t logging;
   char device_name[MAX_NAME_LEN];
   uint8_t priority;
};
typedef struct gps_receiver_setup gps_receiver_setup_type;

// thread entry point
void * gps_receiver_init(void *);

////////////////////////////////////////////////////////////////////////

void print_nmea_message(
      /* in     */ const gps_receiver_output_type *out
      );

#endif   // GPS_RECEIVER_H

