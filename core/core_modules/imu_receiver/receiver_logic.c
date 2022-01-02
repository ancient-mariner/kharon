// this file is included and not compiled on its own. use the parent
//    file's #include <...> and don't repeat here

static void start_network(
      /* in out */       datap_desc_type *dp
      )
{
   struct imu_class *imu = (struct imu_class*) dp->local;
   while ((dp->run_state & DP_STATE_DONE) == 0) {
      if ((imu->connfd = 
            wait_for_connection(imu->sockfd, dp->td->obj_name)) >= 0)
         break; // have connection
      if ((dp->run_state & DP_STATE_DONE) != 0)
         break; // quit flag set
      log_err(imu->log, "%s failed to initialize server. Waiting to "
            "try again", dp->td->obj_name);
      sleep(5);
   }
}


static void load_dev2ship(
      /* in out */       imu_class_type *imu,
      /* in     */ const char *module_name,
      /* in     */ const char *dev_name,     // eg, 'bmg160'
      /* in     */ const char *config_name,  // eg, 'gyr_dev2ship'
      /*    out */       matrix_type *mat
      )
{
   // initialize alignment matrix to identity in case problems occur below
   identity_matrix(mat);
   //
   FILE *fp = open_config_file_ro2(get_environment(), dev_name, "sensors", 
         config_name, NULL);
   if (!fp) {
      log_err(imu->log, "Unable to open %s for '%s'", config_name, module_name);
      goto end;
   }
   if (config_read_matrix(fp, mat) != 0) {
      log_err(imu->log, "Unable to read alignment %s for '%s'", 
            config_name, module_name);
      goto end;
   }
end:
   if (fp)
      fclose(fp);
}

void module_config_load_imu_to_ship(
      /* in     */ const char *module_name,
      /*    out */       imu_class_type *imu
      )
{
   load_dev2ship(imu, module_name, imu->device_name, "acc_dev2ship",
         &imu->acc_dev2ship);
   load_dev2ship(imu, module_name, imu->device_name, "mag_dev2ship",
         &imu->mag_dev2ship);
   load_dev2ship(imu, module_name, imu->device_name, "gyr_dev2ship",
         &imu->gyr_dev2ship);
//printf("Module: %s\n", module_name);
//print_mat(&imu->acc_dev2ship, "Acc");
//print_mat(&imu->mag_dev2ship, "Mag");
//print_mat(&imu->gyr_dev2ship, "Gyro");
}

static void unpack_data(
      /* in     */ const double timestamp,
      /* in     */ const char *serial,
      /*    out */       imu_sensor_packet_type *data,
      /* in out */       imu_class_type *imu
      )
{
   restore_sensor_packet_individual(serial, &data->gyr, &data->acc, 
            &data->mag, NULL, &data->temp, NULL, &data->state);
//log_info(imu->log, "flags: %08x  g:%d  a%d  m%d", data->state.flags, data->state.avail[IMU_GYR], data->state.avail[IMU_ACC], data->state.avail[IMU_MAG]);
   data->timestamp = timestamp;
   // recycle acc and mag as necessary
   // acc
   if (data->state.avail[IMU_ACC] == 0) {
      if (imu->recycle_timer_usec[IMU_ACC] > 0) {
         // re-use old value
         copy_vector(&imu->recycle_value[IMU_ACC], &data->acc);
         // don't subtract -- that's done when value is accessed
      }
   } else {
      // cache new value for future use
      copy_vector(&data->acc, &imu->recycle_value[IMU_ACC]);
      imu->recycle_timer_usec[IMU_ACC] = ACC_RECYCLE_DURATION_USEC;
   }
   // mag
   if (data->state.avail[IMU_MAG] == 0) {
      if (imu->recycle_timer_usec[IMU_MAG] > 0) {
         copy_vector(&imu->recycle_value[IMU_MAG], &data->mag);
         // don't subtract -- that's done when value is accessed
      }
   } else {
      // cache new value for future use
      copy_vector(&data->mag, &imu->recycle_value[IMU_MAG]);
      imu->recycle_timer_usec[IMU_MAG] = MAG_RECYCLE_DURATION_USEC;
   }
}


static void rotate_and_log(
      /* in out */       imu_sensor_packet_type *data,
      /* in     */ const imu_class_type *imu
      )
{
   // make copy of data as below matrix operations cannot be done
   //    with the same input/output data structures
   vector_type acc, mag, gyr;
   copy_vector(&data->acc, &acc);
   copy_vector(&data->mag, &mag);
   copy_vector(&data->gyr, &gyr);
   // log data if logging enabled
   if (imu->logfile) {
      fprintf(imu->logfile,
            "%.3f  "
            "%7.4f %7.4f %7.4f  "
            "%7.4f %7.4f %7.4f  "
            "%7.4f %7.4f %7.4f  "
            "%5.1f\n", 
            data->timestamp,
            (double) gyr.v[0], (double) gyr.v[1], (double) gyr.v[2],
            (double) acc.v[0], (double) acc.v[1], (double) acc.v[2],
            (double) mag.v[0], (double) mag.v[1], (double) mag.v[2],
            (double) data->temp);
   }
   // transform IMU data to ship-space
   mult_matrix_vector(&imu->gyr_dev2ship, &gyr, &data->gyr);
   mult_matrix_vector(&imu->acc_dev2ship, &acc, &data->acc);
   mult_matrix_vector(&imu->mag_dev2ship, &mag, &data->mag);
   // apply compass correction
   data->mag.v[0] -= imu->x_mag_bias;
   data->mag.v[2] -= imu->z_mag_bias;
//if (imu->logfile) {
//   fprintf(imu->logfile,
//         "%.3f  "
//         "%6.3f %6.3f %6.3f  "
//         "%7.3f %7.3f %7.3f  "
//         "%6.3f %6.3f %6.3f  "
//         "%6.1f\n", 
//         data->timestamp,
//         (double) data->gyr.v[0], (double) data->gyr.v[1], (double) data->gyr.v[2],
//         (double) data->acc.v[0], (double) data->acc.v[1], (double) data->acc.v[2],
//         (double) data->mag.v[0], (double) data->mag.v[1], (double) data->mag.v[2],
//         (double) data->temp);
//}
}

////////////////////////////////////////////////////////////////////////
// timestamp conversion

static microsecond_timestamp_type prev_imu_publish_time(
      /* in     */ const microsecond_timestamp_type sample
      )
{
   // round down, aligned to multiple of sample interval
   const uint64_t ts = sample.usec / SAMPLE_INTERVAL_US;
   const microsecond_timestamp_type t = { .usec = ts * SAMPLE_INTERVAL_US };
//printf("PREV    %ld -> %ld\n", sample.usec, t.usec);
   return t;
}

static microsecond_timestamp_type next_imu_publish_time(
      /* in     */ const microsecond_timestamp_type sample
      )
{
   // round up, aligned to multiple of sample interval
   const uint64_t ts = (sample.usec + SAMPLE_INTERVAL_US) / SAMPLE_INTERVAL_US;
   const microsecond_timestamp_type t = { .usec = ts * SAMPLE_INTERVAL_US };
//printf("NEXT    %ld -> %ld\n", sample.usec, t.usec);
   return t;
}


// timestamp conversion
////////////////////////////////////////////////////////////////////////

// publish IMU data (ACC and MAG only -- no gyro flagged as present)
static void publish_upsample_no_gyro(
      /* in out */       datap_desc_type *dp,
      /* in out */       imu_class_type *imu,
      /* in     */ const imu_sensor_packet_type *packet,
      /* in     */ const microsecond_timestamp_type data_t
      )
{
   // next publication time
   microsecond_timestamp_type next_t = 
         next_imu_publish_time(imu->prev_publish_t);
   // while next publication time is ahead of packet timestamp, create
   //    a record and publish
   uint32_t num_published = 0;
   // only publish if it's time. the most recently received ACC and MAG 
   //    values are published regardless of when they were received
   //    (assuming they're not stale) so no need to interpolate. that
   //    keeps logic much easier than w/ gyro
   while (next_t.usec <= data_t.usec) {
      // publish next sample to queue
      uint32_t idx = (uint32_t) (dp->elements_produced % dp->queue_length);
      imu_output_type out;
      out.state.flags = 0; // mark all modalities as invalid
      // copy acc and mag data from 'recycle' buffer when it's available
      // stop copying when data is too old so we don't push stale data
      //    to subscribers. they must detect breakage in flow and act
      //    accordingly
      if (imu->recycle_timer_usec[IMU_ACC] > 0) {
         copy_vector(&packet->acc, &out.modality[IMU_ACC]);
         out.state.avail[IMU_ACC] = 1;
         imu->recycle_timer_usec[IMU_ACC] -= SAMPLE_INTERVAL_US;
      }
      if (imu->recycle_timer_usec[IMU_MAG] > 0) {
         copy_vector(&packet->mag, &out.modality[IMU_MAG]);
         out.state.avail[IMU_MAG] = 1;
         imu->recycle_timer_usec[IMU_MAG] -= SAMPLE_INTERVAL_US;
      }
      // publish sample but don't bother to tell subscribers yet
      dp->ts[idx] = real_from_timestamp(next_t);
      imu_output_type *out_buf = dp_get_object_at(dp, idx);
//printf("no-gyr %.3f  %d %d %d  gx:%.3f  ax:%.3f  mx:%.3f  %s\n", dp->ts[idx], out.state.avail[IMU_GYR], out.state.avail[IMU_ACC], out.state.avail[IMU_MAG], (double) out.modality[IMU_GYR].v[0], (double) out.modality[IMU_ACC].v[0], (double) out.modality[IMU_MAG].v[0], dp->td->obj_name);
      // copy data to output buffer
      memcpy(out_buf, &out, sizeof *out_buf);
      // mark data as published but don't signal yet
      dp->elements_produced++;
      num_published++;
      // advance times
      imu->prev_publish_t = next_t;
      next_t.usec += IMU_PRODUCER_INTERVAL_US;
   }
   // if a sample was pushed into the queue, notify subscribers
   if (num_published > 0) {
      dp_signal_data_available(dp);
   }
}

// publish sample(s) with gyro data
// keep track of total rotation and publish accumulated rotation
//    since last published sample and the next
// NOTE: there is a variable delay between when data is acquired and
//    timestampled and when it is published, with the publish timestamp
//    being on the order of 10ms delayed relative to the acquired data.
//    6.25ms is from the driver, which publishes the previous 12.5ms 
//    at the end of the interval, and ~5ms delay from pushing to next 
//    upsampled publication time. for now this can be compensated for 
//    by adjusting the image-acquisition delay (whose acquisition delay
//    error is at least as large)
// TODO publish with minimal delay by adjusting publication
//    timestamp to accurately reflect when data was acquired
// NOTE: upsample manually checked by rotating device through 360 degrees
//    in opposite directions and summed reported rotation for raw and
//    upsampled values. values checked out to less than a 1/50 degree
//    (start/stop samples don't align, possibly explaining part of error)
//    version: 308, jan 2020
static void publish_upsample_gyro(
      /* in out */       datap_desc_type *dp,
      /* in out */       imu_class_type *imu,
      /* in     */ const imu_sensor_packet_type *packet,
      /* in     */ const microsecond_timestamp_type data_t
      )
{
   // next publication time
   microsecond_timestamp_type next_t = 
         next_imu_publish_time(imu->prev_publish_t);
//printf("-------------------------------\n");
//printf("data at %.3f, next publish at %.3f, last sample at %.3f, last gyro at %.3f\n", (double) data_t.usec * 1.0e-6, (double) next_t.usec * 1.0e-6, (double) imu->prev_publish_t.usec * 1.0e-6, (double) imu->prev_gyr_data_t.usec * 1.0e-6);
   if (data_t.usec < imu->prev_gyr_data_t.usec) {
      // out-of-order packet received. this happened before after a
      //    large synchronization adjustment moved the PIs clock back
      //    more than a sample interval, and a fix should prevent that
      //    from recurring. however, in case of a repeat, move the
      //    record of the timestamp of the previous sample back in
      //    time to match the new data sample, so there's not a 
      //    negative dT (this should have the effect of dropping
      //    the present value)
      // log the event first
      log_err(imu->log, "Out-of-order packets detected. Previous was at "
            "%.3f and present is at %.3f", 
            (double) imu->prev_gyr_data_t.usec * 1.0e-6, 
            (double) data_t.usec * 1.0e-6);
      imu->prev_gyr_data_t = data_t;
   }
   // put a copy of gyro data on the stack
   vector_type curr_gyr;
	copy_vector(&packet->gyr, &curr_gyr);
//print_vec(&curr_gyr, "current gyro");
   // keep track of how much rotation occurred, and publish as much
   //    as possible
   uint32_t num_published = 0;
   while (next_t.usec < data_t.usec) {
      // data to publish -- do so
      uint32_t idx = (uint32_t) (dp->elements_produced % dp->queue_length);
      imu_output_type out;
      out.state.flags = packet->state.flags;
      // copy acc and mag data from 'recycle' buffer when it's available
      // stop copying when data is too old so we don't push stale data
      //    to subscribers. they must detect breakage in flow and act
      //    accordingly
      if (imu->recycle_timer_usec[IMU_ACC] > 0) {
         copy_vector(&packet->acc, &out.modality[IMU_ACC]);
         out.state.avail[IMU_ACC] = 1;
         imu->recycle_timer_usec[IMU_ACC] -= SAMPLE_INTERVAL_US;
      } 
      if (imu->recycle_timer_usec[IMU_MAG] > 0) {
         copy_vector(&packet->mag, &out.modality[IMU_MAG]);
         out.state.avail[IMU_MAG] = 1;
         imu->recycle_timer_usec[IMU_MAG] -= SAMPLE_INTERVAL_US;
      }
      // add rotation from this sample to the 'recycle' gyro value
      //    (horrible name -- refactor later). rotation should be
      //    added from the previous gyro data time and the sample
      // then publish
      double k = (double) (next_t.usec - imu->prev_gyr_data_t.usec) 
            / (double) IMU_PRODUCER_INTERVAL_US;
//print_vec(&imu->recycle_value[IMU_GYR], "previous rotation");
//printf("k = %f (%ld - %ld)\n", (double) k, next_t.usec, imu->prev_gyr_data_t.usec);
      for (uint32_t i=0; i<3; i++) {
         imu->recycle_value[IMU_GYR].v[i] += k * curr_gyr.v[i];
      }
      copy_vector(&imu->recycle_value[IMU_GYR], &out.modality[IMU_GYR]);
      // publish sample but don't bother to tell subscribers yet
      dp->ts[idx] = real_from_timestamp(next_t);
      imu_output_type *out_buf = dp_get_object_at(dp, idx);
//printf("gyr  %.3f  flags: %08lx  gx:%.3f  ax:%.3f  mx:%.3f  %s\n", dp->ts[idx], out.state.flags, (double) out.modality[IMU_GYR].v[0], (double) out.modality[IMU_ACC].v[0], (double) out.modality[IMU_MAG].v[0], dp->td->obj_name);
      // copy data to output buffer
      memcpy(out_buf, &out, sizeof *out_buf);
//printf("%s publish, %.3f, %.3f, %.3f, %.3f\n", dp->td->obj_name, dp->ts[idx], (double) out_buf->modality[IMU_GYR].v[0], (double) out_buf->modality[IMU_GYR].v[1], (double) out_buf->modality[IMU_GYR].v[2]);
      // mark data as published but don't signal yet
      dp->elements_produced++;
      num_published++;
      // advance times
      imu->prev_publish_t = next_t;
      imu->prev_gyr_data_t = next_t;
      next_t.usec += IMU_PRODUCER_INTERVAL_US;
      // reset 'recycle' value -- it's used up so put it to zero
      zero_vector(&imu->recycle_value[IMU_GYR]);
   };
   /////////////////////////////
   // present data is before the next publication time, update
   //    the 'recycle' bucket with the amount of rotation between
   //    the gyro sample and the present timestamp
   double k = (double) (data_t.usec - imu->prev_gyr_data_t.usec) 
         / (double) IMU_PRODUCER_INTERVAL_US;
//printf("last k = %f (%ld - %ld)\n", (double) k, data_t.usec, imu->prev_gyr_data_t.usec);
   for (uint32_t i=0; i<3; i++) {
      imu->recycle_value[IMU_GYR].v[i] += k * curr_gyr.v[i];
   }
//print_vec(&imu->recycle_value[IMU_GYR], "leftover rotation");
   // update gyro data timestamp
   imu->prev_gyr_data_t = data_t;
   // if a sample was pushed into the queue, notify subscribers
   if (num_published > 0) {
      dp_signal_data_available(dp);
   }
}


// add sample to queue
// data from network is expected to be in packet, and if that's not 
//    available then recycled data should be copied to there
// for publication, fill in data for all previous data samples, whether 
//    from recycling (acc, mag) or interpolation (gyr)
// if gyro is prioritized as being available, don't publish until
//    new gyro data is presented
static void publish_upsample(
      /* in out */       datap_desc_type *dp,
      /* in out */       imu_class_type *imu,
      /* in     */ const imu_sensor_packet_type *packet
      )
{
   // convert sample acquisition time to timestamp
   microsecond_timestamp_type data_t = timestamp_from_real(packet->timestamp);
//printf("acquisition time %.3f  (%ld)\n", packet->timestamp, data_t.usec);
   if (imu->prev_publish_t.usec == 0) {
      // first sample. advance timer to what would be previous publication
      //    time for this sample
      imu->prev_publish_t = prev_imu_publish_time(data_t);
      imu->prev_gyr_data_t = data_t;
//printf("** initialize clocks\n");
   }
   if (imu->priority[IMU_GYR] == IMU_PRI_NULL) {
      publish_upsample_no_gyro(dp, imu, packet, data_t);
   } else {
      if (packet->state.avail[IMU_GYR] == 1) {
         // gyro data available -- process it
         publish_upsample_gyro(dp, imu, packet, data_t);
      } else {
         printf("No gyro data available\n");
      }
   }
}

////////////////////////////////////////////////////////////////////////
// set priority

// interface to set priorities for modalities from imu producer
// modality priorities are on 1-4, with 4 meaning modality is absent for
//    this producer
void set_imu_priority(
      /* in out */       datap_desc_type *imu_dp,
      /* in     */ const uint32_t acc_pri,
      /* in     */ const uint32_t gyr_pri,
      /* in     */ const uint32_t mag_pri
      )
{
   // sanity check
   if (imu_dp == NULL) {
      fprintf(stderr, "NULL source provided to set_imu_priority\n");
      hard_exit(__func__, __LINE__);
   } 
   /////////////////////////////////////////////////////////////////////
   imu_class_type *imu = (imu_class_type*) imu_dp->local;
   if (strcmp(imu_dp->td->class_name, IMU_CLASS_NAME) != 0) {
      fprintf(stderr, "Priority must be set on IMU module, not %s\n",
            imu_dp->td->class_name);
      hard_exit(__func__, __LINE__);
   }
   /////////////////////////////////////////////////////////////////////
   // set all non-active priorities to NULL
   for (uint32_t i=0; i<NUM_IMU_CHANNELS; i++) {
      imu->priority[i] = IMU_PRI_NULL;
   }
   // validate incoming data
   uint32_t err=0;
   if (acc_pri > IMU_PRI_NULL) {
      log_err(imu->log, "IMU source %s (acc) has invalid priority %d", 
            imu_dp->td->obj_name, acc_pri);
      err++;
   }
   if (gyr_pri > IMU_PRI_NULL) {
      log_err(imu->log, "IMU source %s (gyr) has invalid priority %d", 
            imu_dp->td->obj_name, gyr_pri);
      err++;
   }
   if (mag_pri > IMU_PRI_NULL) {
      log_err(imu->log, "IMU source %s (mag) has invalid priority %d", 
            imu_dp->td->obj_name, mag_pri);
      err++;
   }
   /////////////////////////////////////////////////////////////////////
   // set priorities
   imu->priority[IMU_ACC] = (int8_t) acc_pri;
   imu->priority[IMU_MAG] = (int8_t) mag_pri;
   imu->priority[IMU_GYR] = (int8_t) gyr_pri;
   /////////////////////////////////////////////////////////////////////
   // if there were errors, bail out
   if (err > 0) {
      hard_exit(__func__, __LINE__);
   }
}

