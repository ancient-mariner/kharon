#include "time_lib.h"
#include <stdio.h>
#include <assert.h>
#include <errno.h>
#include <math.h>

microsecond_timestamp_type timestamp_from_real(
      /* in     */ const double t_dbl
      )
{
   // simply casting an int64 to float and back can provide the wrong
   //    number, due rounding error. adding a small value before the cast
   //    will compensate for that problem. a rounding factor = 4.99e-7 has 
   //    been tested round-trip for timestamps for 1ms resolution
   //    timestamps representing time out to 50 years (i.e., 1.6e9 seconds)
   const microsecond_timestamp_type t = 
         { .usec = (uint64_t) ((t_dbl + 4.99e-7) * 1.0e6) };
   return t;
}

double real_from_timestamp(
      /* in     */ const microsecond_timestamp_type t
      )
{
   return (double) t.usec * 1.0e-6;
}


////////////////////////////////////////////////////////////////////////

void print_time(
      /* in     */ const struct timespec *t,
      /* in     */ const char *label
      )
{
   printf("%20s\t%6ld.%09ld\n", label, t->tv_sec, t->tv_nsec);
}

////////////////////////////////////////////////////////////////////////

void double_to_timespec(
      /* in     */ const double seconds,
      /*    out */       struct timespec *ts
      )
{
   assert(seconds >= 0.0);
   ts->tv_sec = (time_t) (seconds);
   ts->tv_nsec = (long) 
         (1.0e9 * (seconds - (double) ((time_t) (seconds))) + 0.5);
}

double timespec_to_double(
      /* in     */ const struct timespec *ts
      )
{
   assert(ts->tv_sec >= 0);
   double sec = (double) ts->tv_sec + 1.0e-9 * ((double) ts->tv_nsec);
   return sec;
}


double time_delta_seconds(
      /*    out */       struct timespec *a,
      /* in     */ const struct timespec *b
      )
{
   double t = (double) (b->tv_sec - a->tv_sec) + 
         1.0e-9 * ((double) (b->tv_nsec - a->tv_nsec));
   return fabs(t);
}

// copy data from src into dest
void timecpy(
      /*    out */       struct timespec *dest,
      /* in     */ const struct timespec *src
      )
{
   dest->tv_sec = src->tv_sec;
   dest->tv_nsec = src->tv_nsec;
}

void timeadd(
      /* in out */       struct timespec *t,
      /* in     */ const time_t sec,
      /* in     */ const long nsec
      )
{
   const int32_t NSECS = 1000 * 1000 * 1000;
   t->tv_sec += sec;
   t->tv_nsec += nsec;
   if (t->tv_nsec > NSECS) {
      t->tv_nsec -= NSECS;
      t->tv_sec += 1;
   } else if (t->tv_nsec < 0) {
      t->tv_nsec += NSECS;
      t->tv_sec -= 1;
   }
}

void timeadd_f(
      /* in out */       struct timespec *t,
      /* in     */ const double sec
      )
{
   const int32_t NSECS = 1000 * 1000 * 1000;
   time_t sec_i = (time_t) sec;
   double remainder = sec - (double) sec_i;
   long nsec_i = (long) (remainder * NSECS);
   timeadd(t, sec_i, nsec_i);
}


// returns 1 if a is later than b, -1 if b is later than a, 
//    and 0 if they're equal
int32_t timecmp(
      /* in     */ const struct timespec *a,
      /* in     */ const struct timespec *b
      )
{
   if (a->tv_sec > b->tv_sec)
      return 1;
   else if (a->tv_sec < b->tv_sec)
      return -1;
   if (a->tv_nsec > b->tv_nsec)
      return 1;
   else if (a->tv_nsec < b->tv_nsec)
      return -1;
   return 0;
}

// sleep until time stored in waketime, using
//    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, ...)
// sleep is aborted if interrupted and value pointed to by quit_flag
//    is non-zero. if quit_flag is NULL then it is ignored.
//
// returns 0 on normal waking
// returns errno on error
// returns EINTR on if exiting on quit
int32_t sleep_until(
      /* in     */ const struct timespec *waketime,
      /* in     */ volatile int32_t *quit_flag
      )
{
   int32_t quit_override = 0;
   volatile int32_t *quit = (quit_flag == NULL ? &quit_override : quit_flag);
   while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, waketime,
            NULL) != 0) {
      // check for quit signal -- in case sleeping forever
      if (*quit != 0)   
         return EINTR;
      if ((errno != 0) && (errno != EINTR)) {
         return errno;
      }
   }
   return 0;
}


////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

#if defined(TEST_TIME_LIB)

static uint32_t test_double_ts_roundtrip(
      /* in     */ const double s
      )
{
   uint32_t errs = 0;
   struct timespec ts;
   double_to_timespec(s, &ts);
   double s2 = timespec_to_double(&ts);
   if (fabs(s - s2) > 1.0e-12) {
      fprintf(stderr, "Double-timespec roundtrip error: %e and %e differ "
            "by %e\n", s, s2, s-s2);
      errs++;
   }
   return errs;
}

static uint32_t test_time_delta(
      /*    out */       struct timespec *a,
      /* in     */ const struct timespec *b,
      /* in     */ const double dt
      )
{
   uint32_t errs = 0;
   double dt2 = time_delta_seconds(a, b);
   if (fabs(dt2 - dt) > 1.0e-12) {
      fprintf(stderr, "Time-delta error: |%ld.%09ld - %ld.%09ld| = %f. "
            "Got %f\n", a->tv_sec, a->tv_nsec, b->tv_sec, b->tv_nsec, dt, dt2);
      errs++;
   }
   return errs;
}

static uint32_t test_time_compare(
      /*    out */       struct timespec *a,
      /* in     */ const struct timespec *b,
      /* in     */ const int32_t expected
      )
{
   uint32_t errs = 0;
   int32_t val = timecmp(a, b);
   if (val != expected) {
      fprintf(stderr, "Time compare error: %f > %f. Got %d. Expected %d\n",
            timespec_to_double(a), timespec_to_double(b), val, expected);
      errs++;
   }
   return errs;
}

static uint32_t test_time_add(
      /* in     */ const struct timespec *a,
      /* in     */ const struct timespec *b,
      /* in     */ const double t
      )
{
   uint32_t errs = 0;
   struct timespec ts;
   timecpy(&ts, a);
   timeadd(&ts, b->tv_sec, b->tv_nsec);
   double t2 = timespec_to_double(&ts);
   if (fabs(t - t2) > 1.0e-12) {
      fprintf(stderr, "Time add error. "
            "%ld.%09ld + %ld.%09ld != %.9f. Got %.9f (delta=%e)\n",
            a->tv_sec, a->tv_nsec, b->tv_sec, b->tv_nsec, t, t2, t-t2);
      errs++;
   }
   return errs;
}

static uint32_t test_times(void)
{
   uint32_t errs = 0;
   printf("Testing time functions\n");
   struct timespec ta, tb, tc, td;
   double a = 1.2345;
   double b = 3.5;
   double c = 4.9998;
   double d = 5.997;
   ///////////////////////////
   // double-to-timespec and back
   errs += test_double_ts_roundtrip(a);
   errs += test_double_ts_roundtrip(b);
   errs += test_double_ts_roundtrip(c);
   errs += test_double_ts_roundtrip(d);
   ///////////////////////////
   double_to_timespec(a, &ta);
   double_to_timespec(b, &tb);
   double_to_timespec(c, &tc);
   double_to_timespec(d, &td);
   // time delta
   errs += test_time_delta(&tb, &ta, b-a);
   errs += test_time_delta(&tc, &ta, c-a);
   errs += test_time_delta(&td, &ta, d-a);
   errs += test_time_delta(&tc, &tb, c-b);
   errs += test_time_delta(&td, &tb, d-b);
   errs += test_time_delta(&td, &tc, d-c);
   ///////////////////////////
   // time compare
   errs += test_time_compare(&ta, &tb, -1);
   errs += test_time_compare(&ta, &ta,  0);
   errs += test_time_compare(&tb, &ta,  1);
   errs += test_time_compare(&ta, &tc, -1);
   errs += test_time_compare(&tc, &ta,  1);
   errs += test_time_compare(&ta, &td, -1);
   errs += test_time_compare(&td, &ta,  1);
   //
   errs += test_time_compare(&tb, &tc, -1);
   errs += test_time_compare(&tb, &tb,  0);
   errs += test_time_compare(&tc, &tb,  1);
   errs += test_time_compare(&tb, &td, -1);
   errs += test_time_compare(&td, &tb,  1);
   //
   errs += test_time_compare(&tc, &td, -1);
   errs += test_time_compare(&tc, &tc,  0);
   errs += test_time_compare(&td, &td,  0);
   errs += test_time_compare(&td, &tc,  1);
   ///////////////////////////
   // time add
   errs += test_time_add(&ta, &tb, a+b);
   errs += test_time_add(&ta, &tc, a+c);
   errs += test_time_add(&ta, &td, a+d);
   errs += test_time_add(&tb, &tc, b+c);
   errs += test_time_add(&tb, &td, b+d);
   errs += test_time_add(&tc, &td, c+d);
   //
   return errs;
}

int main(int argc, char** argv) 
{
   (void) argc;
   (void) argv;
   uint32_t errs = 0;
   errs += test_times();
   //////////////////
   printf("\n");
   if (errs == 0) {
      printf("--------------------\n");
      printf("--  Tests passed  --\n");
      printf("--------------------\n");
   } else {
      printf("**********************************\n");
      printf("**** ONE OR MORE TESTS FAILED ****\n");
      printf("**********************************\n");
      fprintf(stderr, "%s failed\n", argv[0]);
   }
   return (int) errs;
}

#endif   // TEST_TIME_LIB

