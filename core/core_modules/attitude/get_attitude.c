#include "pinet.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <errno.h>
#include <dirent.h>
#include "datap.h"
#include "sensor_packet.h"
#include "logger.h"

#include "core_modules/attitude.h"

static void make_weighted_average(
      /* in     */ const datap_desc_type *dp, 
      /* in out */       log_info_type *log,
      /* in     */ const double t, 
      /* in     */ const uint32_t idx_before,
      /* in     */ const uint32_t idx_after,
      /*    out */       attitude_output_type *rot
      )
{
   const double t0 = dp->ts[idx_before];
   const double t1 = dp->ts[idx_after];
   const double dt = t1 - t0;
   log_info(log, "Interpolating %.3f (idx=%ld) on interval [%0.3f, %0.3f]", 
         t, idx_before, t0, t1);
   if (c_assert(dt > 0.0) != 0) {
      // this is a recoverable error, but it indicates a problem 
      //    somewhere (ie, this should never happen)
      log_err(log, "Internal attitude error. Time between successive "
            "timestamps is not increasing");
      log_err(log, "Idx %d has t=%f", idx_before, t0);
      log_err(log, "Idx %d has t=%f", idx_after, t1);
      // treat as a fatal error
      hard_exit(__func__, 1);
   }
   const double w1 = (double) ((t - t0) / dt);
   assert(w1 <= 1.0);
   assert(w1 >= 0.0);
   const double w0 = (double) (1.0 - w1);
   const matrix_type *low = dp_get_object_at(dp, idx_before);
   const matrix_type *high = dp_get_object_at(dp, idx_after);
   for (uint32_t i=0; i<9; i++) {
      rot->ship2world.m[i] = w0 * low->m[i] + w1 * high->m[i];
   }
}

// TODO evaluate this, esp. regarding new 100Hz publication approach
//    (probably OK, but need to check)
// called by attitude consumer (e.g., optical_up)
void get_attitude(
      /* in     */ const datap_desc_type *dp, 
      /* in out */       log_info_type *log,
      /* in     */ const double t, 
      /*    out */       enum attitude_query_state *out_status,
      /*    out */       attitude_output_type *rot,
      /* in out */       uint64_t *prev_idx
      )
{
   const uint32_t queue_len = dp->queue_length;
   // search for attitude data for a given time
   enum attitude_query_state status = FOUND; // ever the optimist
   // start search at prev_idx or 1/2 queue length behind head of queue
   // reason for not reading from end of queue is to avoid possibility
   //    (soft-avoid) of having attitude thread get ahead of reader
   //    and overwrite part of buffer being read from
   // early start is earliest position in queue that we can start
   //    looking
   const uint64_t produced = dp->elements_produced;
   const uint64_t early_start = (produced > queue_len/2) ?
         (produced - queue_len/2) : 0;
   // index in queue to start search
   const uint64_t prev = *prev_idx; // local copy of previous index
   uint64_t idx = prev > early_start ? prev : early_start;
   if (c_assert(idx <= produced) != 0) {
      log_err(log, "Internal error (get_attitude): search index %d is "
            "past end of array (n=%ld)\n", idx, produced);
      hard_exit(__func__, __LINE__);
   }
   // search forward until a sample is found after desired time
   // look for interval that surrounds desired time and interpolate
   //    between samples to get desired approximation
   uint32_t idx_new = (uint32_t) (idx % queue_len);
   uint32_t idx_old = idx_new;   // start w/ new = old
   for (; idx<produced; idx++) {
      if (dp->ts[idx_new] > t) {
         // found a time sample after desired time
         if (idx_new == idx_old) {
            // the first sample tested is in the future
            //    so we don't have a valid interval
            log_info(log, "First sample (%.3f) is past now (%.3f)",
                  dp->ts[idx_new], t);
            status = MISSING;
            goto end;
         }
         // make weighted average of attitude matrix from before and after
         //    samples
         make_weighted_average(dp, log, t, idx_old, idx_new, rot);
         // store index of sample immediately preceding desired time
         *prev_idx = idx_old;
         // all done
         goto end;
      }
      idx_old = idx_new;
      // advance index. check for wraparound at end of queue
      if (++idx_new >= queue_len) {
         idx_new = 0;
      }
   }
   // if we made it here then we didn't find a future timestamp
   // ie, requested time is later than last sample. cannot provide answer
   //    right now
   status = PENDING;
end:
   // single exit point
   *out_status = status;
}

