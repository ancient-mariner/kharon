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
#include "core_modules/support/imu_streams.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "lin_alg.h"

////////////////////////////////////////////////////////////////////////
// init

static void simple_stream_init(
      /*    out */       simple_vector_stream_type *stream,
      /* in     */ const int8_t priority
      )
{
   stream->timestamp.usec = 0;
   zero_vector(&stream->sample);
   stream->priority = priority;
}


static void resample_stream_init(
      /*    out */       resampled_vector_stream_type *stream,
      /* in     */ const int8_t priority
      )
{
   memset(stream, 0, sizeof *stream);
   stream->write_sample_sec = -1.0;
   stream->priority = priority;
}

static void add_first_sample(
      /* in out */       resampled_vector_stream_type *stream,
      /* in     */ const vector_type *val,
      /* in     */ const double t   // sample time, in seconds
      )
{
   // find publish time before and after t. add val to sample
   //    defined by these bounds, setting 'weight' as t-start_t
   uint64_t sample_start_time = (uint64_t) (t * 1.0e6);
   // align to 10ms boundary
   sample_start_time /= SAMPLE_DUR_USEC;
   sample_start_time *= SAMPLE_DUR_USEC;
   double sample_start_sec = (double) sample_start_time * 1.0e-6;
   // set read and write times
   uint64_t sample_end_time = sample_start_time + SAMPLE_DUR_USEC;
   double sample_end_sec = (double) sample_end_time * 1.0e-6;
   stream->read_sample_time.usec = sample_end_time;
   stream->read_sample_sec = sample_end_sec;
   stream->write_sample_time.usec = sample_end_time;
   stream->write_sample_sec = sample_end_sec;
   // push sample values to queue
   // even if T is at end_t, don't publish. let next call to
   //    add_sample() handle publication
   double dt = t - sample_start_sec;
   double wt = dt / SAMPLE_DUR_SEC;
   stream->read_queue_idx = 0;
   stream->write_queue_idx = 0;
   vector_type *data = &stream->resampled[stream->write_queue_idx];
   for (uint32_t i=0; i<3; i++) {
      data->v[i] = val->v[i] * wt;
   }
   stream->write_pos_dur = dt;
}

// init
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
// main interface -- simple stream

static void update_stream_data(
      /* in out */       simple_vector_stream_type *stream,
      /* in     */ const vector_type *val,
      /* in     */ const double t   // sample time, in seconds
      )
{
   stream->timestamp.usec = (uint64_t) (t * 1.0e6);
   for (uint32_t i=0; i<3; i++) {
      stream->sample.v[i] = val->v[i];
   }
}

// main interface -- simple stream
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
// main interface -- resample

// move read position to next position in queue
// WARNING there are no checks -- call this only if it's known that
//    the read position can be safely advanced
static void advance_read_position(
      /* in out */       resampled_vector_stream_type *stream
      )
{
   stream->read_queue_idx =
         (stream->read_queue_idx + 1) & (RESAMPLE_QUEUE_LEN - 1);
   stream->read_sample_time.usec += SAMPLE_DUR_USEC;
   stream->read_sample_sec = (double) stream->read_sample_time.usec * 1.0e-6;
}


// advances the write index, updating other values as appropriate
// if queue fully wraps, read index is advanced too
static void publish_sample(
      /* in out */       resampled_vector_stream_type *stream
      )
{
   // update index and sample times of write data
   uint32_t idx = stream->write_queue_idx + 1;
   idx &= (RESAMPLE_QUEUE_LEN - 1); // wrap index around end of queue
   stream->write_queue_idx = idx;
   stream->write_sample_time.usec += SAMPLE_DUR_USEC;
   stream->write_sample_sec = (double) stream->write_sample_time.usec * 1.0e-6;
   // advance read if necessary
   if (stream->write_queue_idx == stream->read_queue_idx) {
      // queue is full. advance read point
      advance_read_position(stream);
   }
   // clear next write vector
   vector_type *vec = &stream->resampled[idx];
   for (uint32_t i=0; i<3; i++) {
      vec->v[i] = 0.0f;
   }
   stream->write_pos_dur = 0.0;
}

static void add_sample(
      /* in out */       resampled_vector_stream_type *stream,
      /* in     */ const vector_type *val,
      /* in     */ const double t   // sample time, in seconds
      )
{
   if (stream->write_sample_sec < 0.0) {
      add_first_sample(stream, val, t);
      return;
   }
   while (t >= stream->write_sample_sec) {
      vector_type *data = &stream->resampled[stream->write_queue_idx];
      if (stream->write_pos_dur > 0.0) {
         // write data is partially filled from previous sample that
         //    partially overlapped with it. fill it the rest of the
         //    way and publish
         double dt = SAMPLE_DUR_SEC - stream->write_pos_dur;
         double remainder = dt / SAMPLE_DUR_SEC;
         for (uint32_t i=0; i<3; i++) {
            data->v[i] += val->v[i] * remainder;
         }
      } else {
         // the supplied value completely covers the next sample. fill
         //    sample and publish
         for (uint32_t i=0; i<3; i++) {
            data->v[i] = val->v[i];
         }
      }
      // sets write_pos_dur to 0.0 and increments write_sample_sec
      publish_sample(stream);
   }
   // all complete resampled values are filled. use what's left to
   //    partially fill the write sample
   vector_type *data = &stream->resampled[stream->write_queue_idx];
   // get dt between sample start and data timestamp (write_sample_sec
   //    is at end of sample)
   double dt = SAMPLE_DUR_SEC - (stream->write_sample_sec - t);
   double wt = dt / SAMPLE_DUR_SEC; // weight is mult of dT
   for (uint32_t i=0; i<3; i++) {
      data->v[i] = val->v[i] * wt;
   }
   stream->write_pos_dur = dt;
}

// returns microsecond time of next sample if data is available. if
//    no data available, timestamp is zero.
// next_sample_time stores beginning of sample. return time at end
//    of sample
static microsecond_type is_sample_available(
      /* in out */       resampled_vector_stream_type *stream
      )
{
   if (stream->write_queue_idx == stream->read_queue_idx) {
      microsecond_type zero = { .usec = 0 };
      return zero;
   }
   return stream->read_sample_time;
}


// fetches sample in read location and returns time of that sample
// if sample not available, returns -1.0
static double get_next_sample(
      /* in out */       resampled_vector_stream_type *stream,
      /*    out */       vector_type *val
      )
{
   if (stream->write_queue_idx == stream->read_queue_idx) {
      return -1.0;
   }
   vector_type *data = &stream->resampled[stream->read_queue_idx];
   for (uint32_t i=0; i<3; i++) {
      val->v[i] = data->v[i];
   }
   double t = stream->read_sample_sec;
   advance_read_position(stream);
   return t;
}

//
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
// unit tests

#if defined(IMU_STREAMS_TEST)

static uint32_t test_init(void)
{
   uint32_t errs = 0;
   printf("Testing init\n");
   /////////////////////////////////////////////////////////////////////
   // init stream, add single value
   // make sure stream is initialized and nothing is published
   resampled_vector_stream_type stream;
   resample_stream_init(&stream, 1);
   if (stream.write_sample_sec >= 0.0) {
      printf("  Freshly initialized stream doesn't indicate no data\n");
      errs++;
   }
   vector_type a = { .v = { 0.1f, 0.2f, 0.3f } };
   double t = 100.516;
   add_sample(&stream, &a, t);
   double expected_start = 100.520;
   double expected_end = 100.520;
   if (fabs(expected_start - stream.read_sample_sec) > 0.0001) {
      printf("  Incorrect read sample time. Got %.3f, expected %.3f\n",
            stream.read_sample_sec, expected_start);
      errs++;
   }
   if (fabs(expected_end - stream.write_sample_sec) > 0.0001) {
      printf("  Incorrect write sample time. Got %.3f, expected %.3f\n",
            stream.write_sample_sec, expected_end);
      errs++;
   }
   // check vector value
   double x = a.v[0] * 0.6;
   double y = a.v[1] * 0.6;
   double z = a.v[2] * 0.6;
   vector_type *data = &stream.resampled[0];
   if ((fabs(x - data->v[0]) > 0.0001) ||
         (fabs(y - data->v[1]) > 0.0001) ||
         (fabs(z - data->v[2]) > 0.0001)) {
      printf("  Stored vector is of incorrect magnitude. Expected "
            "%.2f,%.2f,%.2f, got %.2f,%.2f,%.2f\n", x, y, z,
            data->v[0], data->v[1], data->v[2]);
      errs++;
   }
   /////////////////////////////////////////////////////////////////////
   if (errs == 0) {
      printf("    passed\n");
   } else {
      printf("    %d errors\n", errs);
   }
   return errs;
}

static uint32_t test_add_sample(void)
{
   uint32_t errs = 0;
   printf("Testing add_sample\n");
   /////////////////////////////////////////////////////////////////////
   resampled_vector_stream_type stream;
   resample_stream_init(&stream, 1);
   vector_type a = { .v = { 0.1, 0.2, 0.3 } };
   vector_type b = { .v = { 0.5, 0.6, 0.7 } };
   // ab is 1/4 the way between a and b
   vector_type ab = { .v = { 0.2, 0.3, 0.4 } };
   // start sample 3/4 the way into the resampled interval
   double t = 100.5175;
   add_sample(&stream, &a, t);
   t += SAMPLE_DUR_SEC;
   add_sample(&stream, &a, t);
   // make sure data is published
   microsecond_type pub_time = is_sample_available(&stream);
   if (pub_time.usec == 0) {
      printf("  First sample not published\n");
      errs++;
   }
   uint64_t expected_time = 100520000l;
   if (pub_time.usec != expected_time) {
      printf("  First sample publish time is off. Got %ld, expected %ld\n",
            pub_time.usec, expected_time);
      errs++;
   }
   vector_type z = { .v = { 0.0, 0.0, 0.0 } };
   double sec = get_next_sample(&stream, &z);
   double expected_sec = (double) expected_time * 1.0e-6;
   if (fabs(sec - expected_sec) > 0.000001) {
      printf("  Sample time is off. Got %.3f, expected %.3f\n", sec,
            expected_sec);
      errs++;
   }
   // check vector value
   if ((fabs(a.v[0] - z.v[0]) > 0.0001) ||
         (fabs(a.v[1] - z.v[1]) > 0.0001) ||
         (fabs(a.v[2] - z.v[2]) > 0.0001)) {
      printf("  Sample value is incorrect. Expected "
            "%.2f,%.2f,%.2f, got %.2f,%.2f,%.2f\n",
            a.v[0], a.v[1], a.v[2], z.v[0], z.v[1], z.v[2]);
      errs++;
   }
   /////////////////////////////////////////////////////////////////////
   // add 2 samples worth of data in one chunk
   t += 2.0 * SAMPLE_DUR_SEC;
   add_sample(&stream, &b, t);
   // get and check first output sample
   sec = get_next_sample(&stream, &z);
   expected_sec += SAMPLE_DUR_SEC;
   if (fabs(sec - expected_sec) > 0.000001) {
      printf("  Sample 2 time is off. Got %.3f, expected %.3f\n", sec,
            expected_sec);
      errs++;
   }
   // check vector value
   if ((fabs(ab.v[0] - z.v[0]) > 0.0001) ||
         (fabs(ab.v[1] - z.v[1]) > 0.0001) ||
         (fabs(ab.v[2] - z.v[2]) > 0.0001)) {
      printf("  Sample 2 value is incorrect. Expected "
            "%.2f,%.2f,%.2f, got %.2f,%.2f,%.2f\n",
            ab.v[0], ab.v[1], ab.v[2], z.v[0], z.v[1], z.v[2]);
      errs++;
   }
   // get and check second output sample
   sec = get_next_sample(&stream, &z);
   expected_sec += SAMPLE_DUR_SEC;
   if (fabs(sec - expected_sec) > 0.000001) {
      printf("  Sample 3 time is off. Got %.3f, expected %.3f\n", sec,
            expected_sec);
      errs++;
   }
   // check vector value
   if ((fabs(b.v[0] - z.v[0]) > 0.0001) ||
         (fabs(b.v[1] - z.v[1]) > 0.0001) ||
         (fabs(b.v[2] - z.v[2]) > 0.0001)) {
      printf("  Sample 3 value is incorrect. Expected "
            "%.2f,%.2f,%.2f, got %.2f,%.2f,%.2f\n",
            b.v[0], b.v[1], b.v[2], z.v[0], z.v[1], z.v[2]);
      errs++;
   }
   /////////////////////////////////////////////////////////////////////
   if (errs == 0) {
      printf("    passed\n");
   } else {
      printf("    %d errors\n", errs);
   }
   return errs;
}

static uint32_t test_simple_stream(void)
{
   uint32_t errs = 0;
   printf("Testing simple_stream\n");
   /////////////////////////////////////////////////////////////////////
   simple_vector_stream_type stream;
   vector_type a = { .v = { 0.1, 0.2, 0.3 } };
   vector_type b = { .v = { 0.2, 0.3, 0.4 } };
   simple_stream_init(&stream, 1);
   update_stream_data(&stream, &a, 100.3456789);
   if (stream.sample.v[0] != a.v[0]) {
      printf("  Stream value 1 incorrect. Found %.3f, expected %.3f\n",
            (double) stream.sample.v[0], (double) a.v[0]);
      errs++;
   }
   microsecond_type expected = { .usec = 100345678 };
   if (stream.timestamp.usec != expected.usec) {
      printf("  Stream timestamp incorrect. Found %ld, expected %ld\n",
            stream.timestamp.usec, expected.usec);
      errs++;
   }
   update_stream_data(&stream, &b, 101.0000000001);
   if (stream.sample.v[0] != b.v[0]) {
      printf("  Stream value 2 incorrect. Found %.3f, expected %.3f\n",
            (double) stream.sample.v[0], (double) b.v[0]);
      errs++;
   }
   /////////////////////////////////////////////////////////////////////
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
   //
   errs += test_init();
   errs += test_add_sample();
   errs += test_simple_stream();
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

#endif   // IMU_STREAMS_TEST

