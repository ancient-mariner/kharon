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
#if !defined(TIME_LIB_H)
#define  TIME_LIB_H
#include "pin_types.h"
#include <time.h>

////////////////////////////////////////////////////////////////////////
// microsecond-based time

// timstamps stored as microseconds to avoid round-off errors of using
//    base 10 in math versus storage of base 2
// TODO migrate to microsecond_timestamp for storage from double
//    and timespec
struct microsecond {
      uint64_t usec;
};
typedef struct microsecond microsecond_type;
typedef struct microsecond microsecond_timestamp_type;

struct millisecond {
      uint64_t msec;
};
typedef struct millisecond millisecond_type;
typedef struct millisecond_timestamp millisecond_timestamp_type;

microsecond_timestamp_type timestamp_from_real(
      /* in     */ const double t
      );

double real_from_timestamp(
      /* in     */ const microsecond_timestamp_type t
      );

// microsecond-based time
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
// original time representation

#define TIME_ADD_1MS       (1000 * 1000)
#define TIME_ADD_1US       (1000)

// sensor sampling rate is 12.5ms
#define TIME_ADD_12_5MS    (12500 * 1000)


// print time to stdout
void print_time(
      /* in     */ const struct timespec *t,
      /* in     */ const char *label
      );

////////////////////////////////////////////////////////////////////////
// time manipulation functions assume that timespec always represents
//    a positive value


// returns 1 if a is later than b, -1 if b is later than a,
//    and 0 if they're equal
int32_t timecmp(
      /* in     */ const struct timespec *a,
      /* in     */ const struct timespec *b
      );

// adds specified interval to timespec
void timeadd(
      /* in out */       struct timespec *t,
      /* in     */ const time_t sec,
      /* in     */ const long nsec
      );

// adds specified interval to timespec
void timeadd_f(
      /* in out */       struct timespec *t,
      /* in     */ const double sec
      );

// copy data from src into dest
void timecpy(
      /*    out */       struct timespec *dest,
      /* in     */ const struct timespec *src
      );


// returns difference in time between a and b, as a real, in seconds
double time_delta_seconds(
      /*    out */       struct timespec *a,
      /* in     */ const struct timespec *b
      );


void double_to_timespec(
      /* in     */ const double seconds,
      /*    out */       struct timespec *ts
      );


double timespec_to_double(
      /* in     */ const struct timespec *ts
      );

// sleep until time stored in waketime, using
//    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, ...)
// sleep is aborted if interrupted and value pointed to by quit_flag
//    is non-zero. if quit_flag is NULL then it is ignored.
//
// sleep until time stored in waketime, using
//    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, ...)
// sleep is aborted if interrupted and value pointed to by quit_flag
//    is non-zero. if quit_flag is NULL then it is ignored.
//
// returns 0 on normal waking
// returns -1 on error
// returns -2 on if exiting on quit
int32_t sleep_until(
      /* in     */ const struct timespec *waketime,
      /* in     */ volatile int32_t *quit_flag
      );

#endif   // TIME_LIB_H
