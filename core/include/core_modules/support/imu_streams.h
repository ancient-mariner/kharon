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
#if !defined(IMU_STREAMS_H)
#define IMU_STREAMS_H
#include "pin_types.h"
#include "time_lib.h"
/**
Takes as input a vector value and timestamp, such as from an IMU stream,
   and resamples to 100Hz on exact millisecond boundaries. values are
   cached. simple stream only keeps most recently reported data.
Stream samples are assumed to contain the data values as measured since
   the previous sample
**/

// IMU publishes data at 100hz and att resamples to the same rate
// TODO move resample to IMU level and have attitude stick to IMU output
//    unless there's a clear reason for not doing that (eg, an auxiliary
//    attitude data source), in which case clearly document why it's
//    necessary/appropriate for multiple resample levels (that's too
//    complex and complex means more error prone)

// stream to be resampled at 100hz, but keep the option open to change that
//    by keeping that as a variable
#define SAMPLE_FREQ_HZ        100
#define SAMPLE_DUR_USEC       (1000000 / SAMPLE_FREQ_HZ)
#define SAMPLE_DUR_SEC        ((double) SAMPLE_DUR_USEC * 1.0e-6)

// number of samples cached
// this must be a power of 2. if not, change bitwise AND code to
//    be integral modulus
#define RESAMPLE_QUEUE_LEN 2048

#define SIMPLE_VECTOR_TIMEOUT_SEC

// NOTE vector streams are not thread safe. they are designed to be
//    manipulated and accessed from the same thread

// for storing the most recent ACC or MAG value from a single IMU
struct simple_vector_stream {
   microsecond_type timestamp;
   vector_type sample;
   int8_t priority;
};
typedef struct simple_vector_stream simple_vector_stream_type;


// gyro stream is resampled to regular intervals. complementary filter
//    applied to gyro stream from data in acc/mag streams, which are
//    simple_vector_stream
struct resampled_vector_stream {
   // data stored in queue
   vector_type resampled[RESAMPLE_QUEUE_LEN];
   // indices of read position in queue, storing next available
   //    sample, and of write position, where next sample is
   //    being assembled
   uint32_t read_queue_idx;
   uint32_t write_queue_idx;
   // write position duration is the amount of data copied to write
   //    buffer, in unit of seconds. min value is 0 and when dur
   //    reaches SAMPLE_DUR the sample is complete and it can be
   //    published
   double write_pos_dur;
   // to keep timestamps on even millisecond boundaries, store sample
   //    times as ints (in usec) and convert those to float
   // times reported are at end of sample
   microsecond_type read_sample_time;
   microsecond_type write_sample_time;
   double read_sample_sec;
   double write_sample_sec;   // <0 when initialized but before data added
   //////////////////
   int8_t priority;
};
typedef struct resampled_vector_stream resampled_vector_stream_type;

#endif   // IMU_STREAMS_H

