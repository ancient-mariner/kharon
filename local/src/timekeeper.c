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
#include "timekeeper.h" // include this first as it defines _GNU_SOURCE
#include <stdio.h>
#include <pthread.h>
#include <stdlib.h>
#include "logger.h"

static double local_to_master_dt_ = 0.0;
// timekeeper-time = sys-time + dt

// to force use of a mutex for protecting the shared master_to_local_dt value,
//    either uncomment the following line or compile with
//    -DTIMEKEEPER_USE_MUTEX
//#define TIMEKEEPER_USE_MUTEX    0

#define TEN_E9    1000000000

#if defined(TIMEKEEPER_USE_MUTEX)
static pthread_mutex_t s_mutex;
#endif   // USE_MUTEX
static int s_init = 0;


void init_timekeeper(void)
{
   if (s_init != 0)
      return;
   s_init = 1;
#if defined(TIMEKEEPER_USE_MUTEX)
   pthread_mutex_init(&s_mutex, NULL);
#else
   // sanity check -- make sure double read/write are considered atomic
   if (__atomic_always_lock_free(sizeof(double), 0) == 0) {
      fprintf(stderr, "Access to double-precision values is not atomic\n");
      fprintf(stderr, "Need to implement mutex for access to shared "
            "variables on this platform\n");
      fprintf(stderr, "Recompile with -DTIMEKEEPER_USE_MUTEX\n");
      fprintf(stderr, "Source: timekeeper.c\n");
      exit(1);
   }
#endif   // USE_MUTEX
}


void shutdown_timekeeper(void)
{
#if defined(TIMEKEEPER_USE_MUTEX)
   pthread_mutex_destroy(&s_mutex);
#endif   // USE_MUTEX
}


double system_now(void)
{
   struct timespec ts;
   clock_gettime(CLOCK_MONOTONIC, &ts);
   return (double) ts.tv_sec + 1.0e-9 * ((double) (ts.tv_nsec));
}

//
double now(void)
{
#if defined(TIMEKEEPER_USE_MUTEX)
   // grab mutex
   pthread_mutex_lock(&s_mutex);
#endif   // USE_MUTEX
   // get time delta
   double dt = local_to_master_dt_;
#if defined(TIMEKEEPER_USE_MUTEX)
   // release mutex
   pthread_mutex_unlock(&s_mutex);
#endif   // USE_MUTEX
   // clock_gettime
   return system_now() + dt;
}


// input is double-precision value in text form (value in seconds since epoch)
void timekeeper_set_time(
      /* in     */ const char * new_time_str
      )
{
   // atof
   double master_clock = strtod(new_time_str, NULL);
   timekeeper_set_time_f(master_clock);
}


// input is seconds since epoch
void timekeeper_set_time_f(
      /* in     */ const double master_clock
      )
{
   // clock_gettime
   // get time of local system clock
   struct timespec ts;
   clock_gettime(CLOCK_MONOTONIC, &ts);
   double local_clock = (double) ts.tv_sec + 1.0e-9 * ((double) ts.tv_nsec);
   // delta is the time difference between master clock and local system
//   double old_dt = local_to_master_dt_;
//
//printf("Timekeeper @ %.4f (sys=%.4f)", now(), local_clock);
#if defined(TIMEKEEPER_USE_MUTEX)
   // grab mutex
   pthread_mutex_lock(&s_mutex);
#endif   // USE_MUTEX
   // update delta
   local_to_master_dt_ = master_clock - local_clock;
#if defined(TIMEKEEPER_USE_MUTEX)
   // release mutex
   pthread_mutex_unlock(&s_mutex);
#endif   // USE_MUTEX
//
//printf(": delta change = %.4f (%.4f -> %.4f)\n",
//      (local_to_master_dt_ - old_dt), old_dt, local_to_master_dt_);
}


////////////////////////////////////////////////////////////////////////
// utility functions

void init_timespec_f(
      /*    out */       struct timespec *t,
      /* in     */ const double seconds
      )
{
   if (c_assert(seconds >= 0.0) != 0) {
      t->tv_sec = 0;
      t->tv_nsec = 0;
      return;
   }
   const time_t sec = (time_t) seconds;
   t->tv_sec = sec;
   t->tv_nsec = (long) (TEN_E9 * (seconds - (double) sec) + 0.5);
}


void increment_time(
      /* in out */       struct timespec *t,
      /* in     */ const time_t sec,
      /* in     */ const long nsec
      )
{
   t->tv_sec += sec;
   t->tv_nsec += nsec;
   if (t->tv_nsec >= TEN_E9) {
      t->tv_sec += 1;
      t->tv_nsec -= TEN_E9;
   }
}

void increment_timef(
      /* in out */       struct timespec *ts,
      /* in     */ const double seconds
      )
{
//log_info("Increment by %.4f seconds", sec);
   if (c_assert(seconds >= 0.0) != 0) {
      ts->tv_sec = 0;
      ts->tv_nsec = 0;
      return;
   }
   struct timespec dt;
   init_timespec_f(&dt, seconds);
   increment_time(ts, dt.tv_sec, dt.tv_nsec);
//printf(" -> %ld\t%ld\n", ts->tv_sec, ts->tv_nsec);
}

void print_timespec(
      /* in     */ const struct timespec t,
      /* in     */ const char *label
      )
{
   double tf = (double) t.tv_sec + 1.0e-9 * ((double) t.tv_nsec);
   printf("%.6f  %s\n", tf, label?label:"");
}

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

#if defined(TEST_TIMEKEEPER)

static uint32_t test_init_timespec_f(void)
{
   uint32_t errs = 0;
   printf("Testing init_timespec_f\n");
   struct timespec t;
   double d = 3.12345678901;
   init_timespec_f(&t, d);
   if ((t.tv_sec != 3) || (t.tv_nsec != 123456789)) {
      fprintf(stderr, "Double time %f != %ld.%09ld\n", d, t.tv_sec, t.tv_nsec);
      errs++;
   }
   //
   if (errs == 0) {
      printf("    passed\n");
   } else {
      printf("    %d errors\n", errs);
   }
   return errs;
}

static uint32_t test_increment_time(void)
{
   uint32_t errs = 0;
   printf("Testing increment_time\n");
   struct timespec t = { .tv_sec = 4, .tv_nsec = 333444555 };
   increment_time(&t, 1, 222111000);
   if ((t.tv_sec != 5) || (t.tv_nsec != 555555555)) {
      fprintf(stderr, "Incorrect value. Expected %d.%09d. Got %ld.%09ld\n",
            5, 555555555, t.tv_sec, t.tv_nsec);
      errs++;
   }
   //
   if (errs == 0) {
      printf("    passed\n");
   } else {
      printf("    %d errors\n", errs);
   }
   return errs;
}

int main(int argc, char **argv)
{
   (void) argc;
   (void) argv;
   uint32_t errs = 0;
   errs += test_init_timespec_f();
   errs += test_increment_time();
   //
   if (errs == 0) {
      printf("--------------------\n");
      printf("--  Tests passed  --\n");
      printf("--------------------\n");
   } else {
      printf("---------------------------------\n");
      printf("***  One or more tests failed ***\n");
   }
   return (int) errs;
}
#endif   // TEST_TIMEKEEPER
