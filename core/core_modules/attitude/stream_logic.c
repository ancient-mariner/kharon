
#include "device_align.c"

// pulls data from all producers with new content and updates vector streams
static void update_vector_streams(
      /* in out */       datap_desc_type *self,
      /* in out */       attitude_class_type *att
      )
{
   for (uint32_t i=0; i<self->num_attached_producers; i++) {
      if ((self->run_state & DP_STATE_DONE) != 0) {
         break;
      }
      producer_record_type *pr = &self->producer_list[i];
      datap_desc_type *producer = pr->producer;
      while (pr->consumed_elements < producer->elements_produced) {
         // initialization hook. set first publication time based on
         //    when data starts arriving
         if (att->next_publish_time.usec == 0) {
            microsecond_type t = timestamp_from_real(now());
            t.usec /= SAMPLE_DUR_USEC;
            t.usec *= SAMPLE_DUR_USEC;
            att->next_publish_time = t;
         }
         ///////////////////////////////
         // get data source
         const uint32_t p_idx = (uint32_t)
               pr->consumed_elements % producer->queue_length;
         imu_output_type *imu_out = (imu_output_type*)
               dp_get_object_at(producer, p_idx);
         const double t = producer->ts[p_idx];
         ///////////////////////////////
         // send data to each stream
         if ((att->acc_stream[i] != NULL) && (imu_out->state.avail[IMU_ACC])) {
            update_stream_data(att->acc_stream[i], 
                  &imu_out->modality[IMU_ACC], t);
//log_info(att->log, "ACC at %.3f from %s", t, producer->td->obj_name);
         }
         if ((att->mag_stream[i] != NULL) && (imu_out->state.avail[IMU_MAG])) {
            update_stream_data(att->mag_stream[i], 
                  &imu_out->modality[IMU_MAG], t);
//log_info(att->log, "MAG at %.3f from %s", t, producer->td->obj_name);
         }
         if ((att->gyr_stream[i] != NULL) && (imu_out->state.avail[IMU_GYR])) {
            add_sample(att->gyr_stream[i], &imu_out->modality[IMU_GYR], t);
//log_info(att->log, "GYR at %.3f from %s", t, producer->td->obj_name);
         }
         pr->consumed_elements++;
      }
   }
}


////////////////////////////////////////////////////////////////////////
// get data from streams

////////////////////////////////////////////////
// resampled streams

// pulls content of all available P1 and P2 streams into output vector
// output vector is set as weighted average of all inputs
static void pull_p12_data_resample(
      /* in out */       attitude_class_type *att,
      /* in out */       resampled_vector_stream_type **streams,
      /*    out */       vector_type *vec
      )
{
//printf("  p12-gyr\n");
   zero_vector(vec); // clear output vec
   double wt = 0.0;  // keep track of weight added to output vec
   microsecond_type t;
   vector_type tmp_vec;
   for (uint32_t i=0; i<MAX_ATTACHED_PRODUCERS; i++) {
      resampled_vector_stream_type *stream = streams[i];
      if (stream == NULL) {
         continue;
      }
      // add stream data to output vec as function of priority
      switch (stream->priority) {
         case IMU_PRI_1:
            t = is_sample_available(stream);
            if ((t.usec > 0) && (t.usec == att->next_publish_time.usec)) {
               get_next_sample(stream, &tmp_vec);
               add_weighted_vector(vec, &tmp_vec, 1.0);
               wt += 1.0;
            }
            break;
         case IMU_PRI_2:
            t = is_sample_available(stream);
            if ((t.usec > 0) && (t.usec == att->next_publish_time.usec)) {
               get_next_sample(stream, &tmp_vec);
               add_weighted_vector(vec, &tmp_vec, 0.5);
               wt += 0.5;
            }
            break;
         default:
            ;  // do nothing
      };
   }
   if (c_assert(wt > 0.0)) {
      log_err(att->log, "Internal error -- No content pulled from streams");
      hard_exit(__func__, __LINE__);
   }
   // normalize vector by added weight, making it a weighted average of
   //    values in all contributing streams
   mult_vector_scalar(vec, 1.0/wt);
}

// pulls content of all available P2 and P3 streams into output vector
// output vector is set as weighted average of all inputs
static void pull_p23_data_resample(
      /* in out */       attitude_class_type *att,
      /* in out */       resampled_vector_stream_type **streams,
      /*    out */       vector_type *vec
      )
{
//printf("  p23-gyr\n");
   zero_vector(vec); // clear output vec
   double wt = 0.0;  // keep track of weight added to output vec
   microsecond_type t;
   vector_type tmp_vec;
   for (uint32_t i=0; i<MAX_ATTACHED_PRODUCERS; i++) {
      resampled_vector_stream_type *stream = streams[i];
      if (stream == NULL) {
         continue;
      }
      // add stream data to output vec as function of priority
      switch (stream->priority) {
         case IMU_PRI_2:
            t = is_sample_available(stream);
            if ((t.usec > 0) && (t.usec == att->next_publish_time.usec)) {
               get_next_sample(stream, &tmp_vec);
               add_weighted_vector(vec, &tmp_vec, 1.0);
               wt += 1.0;
            }
            break;
         case IMU_PRI_3:
            t = is_sample_available(stream);
            if ((t.usec > 0) && (t.usec == att->next_publish_time.usec)) {
               get_next_sample(stream, &tmp_vec);
               add_weighted_vector(vec, &tmp_vec, 1.0);
               wt += 1.0;
            }
            break;
         default:
            ;  // do nothing
      };
   }
   if (c_assert(wt > 0.0)) {
      log_err(att->log, "Internal error -- No content pulled from streams");
      hard_exit(__func__, __LINE__);
   }
   // normalize vector by added weight, making it a weighted average of
   //    values in all contributing streams
   mult_vector_scalar(vec, 1.0/wt);
}

////////////////////////////////////////////////
// get data from simple streams


// pulls content of all available P1 and P2 streams into output vector
// output vector is set as weighted average of all inputs
static void pull_p12_data(
      /* in out */       attitude_class_type *att,
      /* in out */       simple_vector_stream_type **streams,
      /* in     */ const microsecond_type timeout,
      /*    out */       vector_type *vec
      )
{
//printf("  p12-am\n");
   zero_vector(vec); // clear output vec
   double wt = 0.0;  // keep track of weight added to output vec
   for (uint32_t i=0; i<MAX_ATTACHED_PRODUCERS; i++) {
      const simple_vector_stream_type *stream = streams[i];
      if (stream == NULL) {
         continue;
      }
      // add stream data to output vec as function of priority
      switch (stream->priority) {
         case IMU_PRI_1:
            if (stream->timestamp.usec > timeout.usec) {
               add_weighted_vector(vec, &stream->sample, 1.0);
               wt += 1.0;
            }
            break;
         case IMU_PRI_2:
            if (stream->timestamp.usec > timeout.usec) {
               add_weighted_vector(vec, &stream->sample, 0.5);
               wt += 0.5;
            }
            break;
         default:
            ;  // do nothing
      };
   }
   if (c_assert(wt > 0.0)) {
      log_err(att->log, "Internal error -- No content pulled from streams");
      hard_exit(__func__, __LINE__);
   }
   // normalize vector by added weight, making it a weighted average of
   //    values in all contributing streams
   mult_vector_scalar(vec, 1.0/wt);
}


// pulls content of all available P2 and P3 streams into output vector
// output vector is set as weighted average of all inputs
static void pull_p23_data(
      /* in out */       attitude_class_type *att,
      /* in out */       simple_vector_stream_type **streams,
      /* in     */ const microsecond_type timeout,
      /*    out */       vector_type *vec
      )
{
//printf("  p23-am\n");
   zero_vector(vec); // clear output vec
   double wt = 0.0;  // keep track of weight added to output vec
   for (uint32_t i=0; i<MAX_ATTACHED_PRODUCERS; i++) {
      const simple_vector_stream_type *stream = streams[i];
      if (stream == NULL) {
         continue;
      }
      // add stream data to output vec as function of priority
      switch (stream->priority) {
         case IMU_PRI_2:
            if (stream->timestamp.usec > timeout.usec) {
               add_weighted_vector(vec, &stream->sample, 1.0);
               wt += 1.0;
            }
            break;
         case IMU_PRI_3:
            if (stream->timestamp.usec > timeout.usec) {
               add_weighted_vector(vec, &stream->sample, 1.0);
               wt += 1.0;
            }
            break;
         default:
            ;  // do nothing
      };
   }
   if (c_assert(wt > 0.0)) {
      log_err(att->log, "Internal error -- No content pulled from streams");
      hard_exit(__func__, __LINE__);
   }
   // normalize vector by added weight, making it a weighted average of
   //    values in all contributing streams
   mult_vector_scalar(vec, 1.0/wt);
}


// pull mag data from most recently updated stream
// this is an emergency fallback procedure and should only be used when
//    all live streams have failed. the logic here is that it's assumed
//    that the most recent sample, even if old, is semi-accurate. 
//    that should be largely
//    true if the vessel doesn't turn. if it assumes a new heading, the
//    gyro will drift back to assume the heading from when the mag sensors
//    went offline
// priority is ignored here
static void pull_most_recent_mag_data(
      /* in out */       attitude_class_type *att,
      /*    out */       vector_type *vec
      )
{
   // assume north for bootstrap failure condition where MAG signal was
   //    never reported
   vec->v[0] = 0.0;
   vec->v[1] = 0.0;
   vec->v[2] = 1.0;
   // find most recently updated MAG sensor
   simple_vector_stream_type *recent_stream = NULL;
   microsecond_type t = { .usec = 0 };
   for (uint32_t i=0; i<MAX_ATTACHED_PRODUCERS; i++) {
      simple_vector_stream_type *stream = att->mag_stream[i];
      if (stream == NULL) {
         continue;
      }
      if ((t.usec == 0) || (t.usec < stream->timestamp.usec)) {
         t.usec = stream->timestamp.usec;
         recent_stream = stream;
      }
   }
   if (recent_stream != NULL) {
      copy_vector(&recent_stream->sample, vec);
   }
}


// get data from streams
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////


// sum the number of channels that are 'current' for each priority, with
//    current meaning that content has been received within the timeout
//    window for the channel (e.g., w/in the last 500ms)
static void count_avail_chans_by_priority(
      /* in     */       simple_vector_stream_type **stream,
      /* in     */ const microsecond_type timeout,
      /*    out */       uint32_t cnt[3] // [0] for P1, [1] for P2, [2] for P3
      )
{
   for (uint32_t i=0; i<3; i++) {
      cnt[i] = 0;
   }
   for (uint32_t i=0; i<MAX_ATTACHED_PRODUCERS; i++) {
      if (stream[i] == NULL) {
         continue;
      }
      if (stream[i]->timestamp.usec > timeout.usec) {
         switch (stream[i]->priority) {
            case IMU_PRI_1:
               cnt[0]++;
               break;
            case IMU_PRI_2:
               cnt[1]++;
               break;
            case IMU_PRI_3:
               cnt[2]++;
               break;
         }
      }
   }
}

// count number of sources with data available for publication
//    among list of resample streams. stores time of earliest sample so
//    calling function can determine if next available data is in future
static void count_avail_chans_by_priority_resample(
      /* in out */       attitude_class_type *att,
      /* in out */       resampled_vector_stream_type **stream,
      /*    out */       microsecond_type *earliest_sample,
      /*    out */       uint32_t cnt[3] // [0] for P1, [1] for P2, [2] for P3
      )
{
   for (uint32_t i=0; i<3; i++) {
      cnt[i] = 0;
   }
   earliest_sample->usec = 0;
   // count chans with  data that's as current as publication time
   //    and discard content from before publication time
   for (uint32_t i=0; i<MAX_ATTACHED_PRODUCERS; i++) {
      if (stream[i] == NULL) {
         continue;   // no data of desired modality from this source
      }
      microsecond_type t = is_sample_available(stream[i]);
      while ((t.usec > 0) && (t.usec < att->next_publish_time.usec)) {
         // data is stale (ie, it's older than desired publication time)
         // get sample and discard 
         vector_type ignore;
         get_next_sample(stream[i], &ignore);
         t = is_sample_available(stream[i]);
      }
      if (t.usec == 0) {
         continue;
      }
      // if data still available, update priority count
      if (t.usec == att->next_publish_time.usec) {
         switch (stream[i]->priority) {
            case IMU_PRI_1:
               cnt[0]++;
               break;
            case IMU_PRI_2:
               cnt[1]++;
               break;
            case IMU_PRI_3:
               cnt[2]++;
               break;
         }
      } 
      // keep track of earliest available sample time
      // TODO remove first conditional -- it should be redundant w/ 2nd
      //    but that appears to not be the case
      if (earliest_sample->usec == 0) {
         earliest_sample->usec = t.usec;
      } else if ((t.usec > 0) && (t.usec < earliest_sample->usec)) {
         earliest_sample->usec = t.usec;
      }
   }
}


// check to see that a quorum of P1 sources is available for all modalities
//    and if so, publish next sample
// returns 1 if data published and 0 if not
static int publish_if_available(
      /* in out */       datap_desc_type *self,
      /* in out */       attitude_class_type *att,
      /* in     */ const microsecond_type timeout
      )
{
//log_info(att->log, "Standard publish");
   int rc = 0;
   /////////////////////////////
   // check number of P1 channels available
   uint32_t cnt[3];
   microsecond_type earliest_sample;   // ignored here
   count_avail_chans_by_priority_resample(att, att->gyr_stream, 
         &earliest_sample, cnt);
   if (cnt[0] < att->num_p1_gyr) {
      goto end;
   }
   count_avail_chans_by_priority(att->acc_stream, timeout, cnt);
   if (cnt[0] < att->num_p1_acc) {
      goto end;
   }
   count_avail_chans_by_priority(att->mag_stream, timeout, cnt);
   if (cnt[0] < att->num_p1_mag) {
      goto end;
   }
   /////////////////////////////
   // if we made it to here then there's sufficient content available on
   //    all modalities to publish. do so
   vector_type gyr, acc, mag;
   // first, peek into streams to check alignment (otherwise the streams 
   //    are modified when data is pulled)
   perform_gyro_alignment(self);
   pull_p12_data_resample(att, att->gyr_stream, &gyr);
   pull_p12_data(att, att->acc_stream, timeout, &acc);
   pull_p12_data(att, att->mag_stream, timeout, &mag);
   /////////////////////////////
   // apply filter to data
   dt_second_type dt = { .dt_sec = SAMPLE_DUR_SEC };
   apply_filter(att, &gyr, &acc, &mag, dt);
   /////////////////////////////
   // publish
log_info(att->log, "Standard publish %.3f", (double) att->next_publish_time.usec * 1.0e-6);
   publish_data(att, self);
   rc = 1;
end:
   return rc;  
}


// publish next attitude sample based on whatever data is available (unless
//    no gyro available)
// returns 1 if data published, 0 otherwise
static int32_t force_publish_next_sample(
      /* in out */       datap_desc_type *self,
      /* in out */       attitude_class_type *att,
      /* in     */ const microsecond_type timeout
      )
{
   int rc = 0;
log_warn(att->log, "\nForce publish %.3f", (double) att->next_publish_time.usec * 1.0e-6);
   /////////////////////////////
   // get best gyro data. if only gyro data is in future, publication time
   //    is moved forward to when data is available and the init timer is
   //    incremented
   uint32_t gyr_cnt[3];   // stores number of streams w/ data avail for each pri
   vector_type gyr;
   microsecond_type earliest_sample;
   count_avail_chans_by_priority_resample(att, att->gyr_stream, 
         &earliest_sample, gyr_cnt);
   // cases to consider 
   //    A) present P1 data available
   //    B) present P2/P3 data available (no present P1)
   //    C) no present data but future P1 data
   //    D) no present data but future P2/P3 data (no P1)
   //    E) no data
   dt_second_type dt = { .dt_sec = SAMPLE_DUR_SEC };
   if (earliest_sample.usec > att->next_publish_time.usec) {
log_info(log_, "CD Earliest gyro %.3f  Next %.3f\n", (double) earliest_sample.usec * 1.0e-6, (double) att->next_publish_time.usec * 1.0e-6);
//printf("Earliest sample: %ld\n", earliest_sample.usec);
//printf("Next publish: %ld\n", att->next_publish_time.usec);
      // Case C or D. sample time advanced to convert to case A/B
      // next available data is in the future. advance init timer and 
      //    next publication time (we can do that here because ACC and MAG
      //    streams haven't been read yet
      att->init_timer.seconds += 1.0e-6 * (double) 
            (earliest_sample.usec - att->next_publish_time.usec);
      if (att->init_timer.seconds > BOOTSTRAP_INTERVAL_SEC) {
         att->init_timer.seconds = BOOTSTRAP_INTERVAL_SEC;
      }
//printf("init timer: %f\n", att->init_timer.seconds);
      // we're advancing publish time so need to keep track of how much
      dt.dt_sec += (double) 
            (earliest_sample.usec - att->next_publish_time.usec) * 1.0e-6;
printf("DT: %.3f\n", dt.dt_sec);
      // if dT is too large things can blow up -- this is most observable
      //    on startup when dt_sec=1.0 and there's a high degree of rotation.
      //    problem was observed as segfault in optical_up.
      // this needs to be investigated and fixed FIXME. for now, post a warning
      //    and maybe it'll be noticed
      log_warn(log_, "Force dT=%.3f seconds", dt.dt_sec);
      att->next_publish_time.usec = earliest_sample.usec;
      // see what's available at the new publication time
      count_avail_chans_by_priority_resample(att, att->gyr_stream, 
            &earliest_sample, gyr_cnt);
//printf("Earliest sample: %ld\n", earliest_sample.usec);
//printf("Next publish: %ld\n", att->next_publish_time.usec);
      if (c_assert(earliest_sample.usec == att->next_publish_time.usec)) {
         log_err(att->log, "Internal error -- failed to advance publication "
               "time correctly");
         hard_exit(__func__, __LINE__);
      }
//printf("Done Case CD\n");
   } else if (earliest_sample.usec == 0) {
log_info(log_, "Case E");
      // Case E -- no data available for gyro
      // don't alter publication time as whatever we select is arbitrary.
      // wait until 'future' data (ie, data arriving after next_publish_time).
      //    that will update init_timer and restart the stream
      goto end;
   }
   // Cases A and B are signaled by count values > 0
   if (gyr_cnt[0] > 0) {
log_info(log_, "Case A\n");
      pull_p12_data_resample(att, att->gyr_stream, &gyr);     // A
   } else if ((gyr_cnt[1] + gyr_cnt[2]) > 0) {
log_info(log_, "Case B\n");
      pull_p23_data_resample(att, att->gyr_stream, &gyr);     // B
   } else {
      log_err(att->log, "Internal error -- failed to fetch gyro data");
      hard_exit(__func__, __LINE__);
   }
   /////////////////////////////
   // get best mag and acc
   // if there's content on p1, pull from active p1 and p2
   // else if no p1, pull from p2 and p3 if they exist
   // otherwise (ie, no p1,p2,p3) then pull from p1 or p2 sensor that 
   //    had data most recently
   // ACC
   uint32_t acc_cnt[3];
   vector_type acc;
   count_avail_chans_by_priority(att->acc_stream, timeout, acc_cnt);
   if (acc_cnt[0] > 0) {
log_info(log_, " Acc p12");
      pull_p12_data(att, att->acc_stream, timeout, &acc);
   } else if ((acc_cnt[1] + acc_cnt[2]) > 0) {
log_info(log_, " Acc p23");
      pull_p23_data(att, att->acc_stream, timeout, &acc);
   } else {
log_info(log_, " Acc fallback");
      // no current ACC data
      // it's potentially dangerous to pull from stale sensor for ACC
      //    as this may induce a false list if there was motion during
      //    last data read. instead put ACC as the +Y axis
      acc.v[0] = 0.0f;
      acc.v[1] = 1.0f;
      acc.v[2] = 0.0f;
   }
   // MAG
   uint32_t mag_cnt[3];
   vector_type mag;
   count_avail_chans_by_priority(att->mag_stream, timeout, mag_cnt);
   if (mag_cnt[0] > 0) {
log_info(log_, " Mag p12");
      pull_p12_data(att, att->mag_stream, timeout, &mag);
   } else if ((mag_cnt[1] + mag_cnt[2]) > 0) {
log_info(log_, " Mag p32");
      pull_p23_data(att, att->mag_stream, timeout, &mag);
   } else {
log_info(log_, " Mag fallback");
      pull_most_recent_mag_data(att, &mag);
   }
   /////////////////////////////
   // apply filter to data
   apply_filter(att, &gyr, &acc, &mag, dt);
   /////////////////////////////
   // publish
   publish_data(att, self);
   rc = 1;
end:
//printf("done force publish\n");
   return rc;  
}

