#if !defined(TIMEKEEPER_H)
#define  TIMEKEEPER_H
#if !defined(_GNU_SOURCE)
#define _GNU_SOURCE
#endif   // _GNU_SOURCE

#include <time.h>

// each network node operates based on its own system clock
//
// the timekeeper provides a reference time that is common
//    between all cooperating nodes. time is reported in
//    seconds (double precision). the epoch for this reference
//    time is typically the process start time on the master
//    node, but can be whatever the master determines

void init_timekeeper(void);
void shutdown_timekeeper(void);

// wrapper for clock_gettime(MONOTONIC)
double system_now(void);

// on the master node, this is system time. on slave nodes synchronized
//    to master, this is system time corrected for drift that should
//    be very close (+/- 1ms) to master
double now(void);

// this should be called whenever a reference clock time is available
void timekeeper_set_time(
      /* in     */ const char * new_time_str
      );

// this should be called whenever a reference clock time is available
void timekeeper_set_time_f(
      /* in     */ const double master_clock
      );

////////////////////////////////////////////////////////////////////////
// utility functions

// initialize timespec to specified time
void init_timespec_f(
      /*    out */       struct timespec *t,
      /* in     */ const double seconds
      );

// increment the time stored in timespec by the specified values
// NOTE: decrement is not supported and behavior is undefined if time
//    delta is negative
void increment_time(
      /* in out */       struct timespec *t, 
      /* in     */ const time_t sec, 
      /* in     */ const long nsec
      );
void increment_timef(
      /* in out */       struct timespec *ts, 
      /* in     */ const double seconds
      );

// print contents of a timespec to stdout (# seconds, as float)
void print_timespec(
      /* in     */ const struct timespec t, 
      /* in     */ const char *label
      );

#endif  // TIMEKEEPER_H
