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

// update alignment estimate between gyro-based sensors.
//    using measured rotation from different sensors an estimate is made
//    of how (mis)aligned the axes of the sensors are to each other.
// algorithm looks at only some of sampled data
static void perform_gyro_alignment(
      /* in out */       datap_desc_type *self
      )
{
#if PERFORM_AUTO_ALIGNMENT == 0
   return;
#endif   // PERFORM_AUTO_ALIGNMENT == 0
static double start = -1.0;
if (start < 0) {
   start = now();
}
   attitude_class_type *att = (attitude_class_type*) self->local;
   ////////////////////////////
   // see if there's enough rotation to estimate alignment
   const uint32_t base_idx = att->master_gyro_idx;
   const resampled_vector_stream_type *base_stream = att->gyr_stream[base_idx];
   vector_type v = base_stream->resampled[base_stream->read_queue_idx];
//vector_type v2 = v;
   const double sum = fabs(v.v[0]) + fabs(v.v[1]) + fabs(v.v[2]);
//printf(" %.3f   base length: %.2f\n", now()-start, (double) sum);
   if (sum < MIN_ALIGNMENT_DPS)  {
      return;
   }
   unitify(&v);
   /////////////////////////////////////////////////////////////////////
   const double k1 = att->active_tau;
   const double k0 = 1.0 - k1;
   const uint32_t num_producers = self->num_attached_producers;
   const microsecond_type t = att->gyr_stream[base_idx]->read_sample_time;
   uint32_t update_count = 0;
   for (uint32_t i=base_idx+1; i<num_producers; i++) {
      // only evaluate other gyro sources
      resampled_vector_stream_type *stream = att->gyr_stream[i];
      if (stream == NULL) {
         continue;
      }
      if (stream->read_sample_time.usec != t.usec) {
//printf("  partner %d out of sync: %.3f vs %.3f\n", i, (double) stream->read_sample_time.usec * 1.0e-6, (double) t.usec * 1.0e-6);
         // streams not in sync. ignore this sample
         continue;
      }
      vector_type cross;
      vector_type sample = stream->resampled[stream->read_queue_idx];
      double samp_sum = fabs(sample.v[0]) + fabs(sample.v[1]) +
            fabs(sample.v[2]);
      if (samp_sum < MIN_ALIGNMENT_DPS)  {
//printf("  partner %d below thresh\n", i);
         continue;
      }
//vector_type sample2 = sample;
      unitify(&sample);
      cross_product(&v, &sample, &cross);
      vector_type *accum = &att->rotation_axis[i];
      for (uint32_t j=0; j<3; j++) {
         accum->v[j] = k0 * accum->v[j] + k1 * cross.v[j];
      }
      const double dot = dot_product(&v, &sample);
      degree_type theta;
      if (dot <= -1.0) {
         theta.degrees = 180.0;
      } else if (dot >= 1.0) {
         theta.degrees = 0.0;
      } else {
         theta.degrees = R2D * acos(dot);
      }
//printf("   %.3f  A:%+6.3f,%+6.3f,%+6.3f  B:%+6.3f,%+6.3f,%+6.3f   C:%+6.3f,%+6.3f,%+6.3f   %+7.3f\n", now()-start, (double) v2.v[0], (double) v2.v[1], (double) v2.v[2], (double) sample2.v[0], (double) sample2.v[1], (double) sample2.v[2], (double) cross.v[0], (double) cross.v[1], (double) cross.v[2], (double) theta.degrees);
//print_vec(&v, "base");
//print_vec(&sample, "P1n");
//print_vec(&cross, "cross");
//printf("------ theta = %.5f  (%.5f)\n", (double) theta.degrees, (double) dot);
      degree_type *accum_deg = &att->rotation_theta[i];
      accum_deg->degrees = k0 * accum_deg->degrees + k1 * theta.degrees;
      att->rotation_samples[i]++;
      update_count++;
      if (++att->rotation_samples[i] >= ALIGNMENT_REPORTING_INTERVAL) {
         att->rotation_samples[i] = 0;
         // TODO write alignment estimate to log
vector_type unit;
copy_vector(accum, &unit);
unitify(&unit);
//printf("%.3f \tAlignment %d  theta=%+7.3f   vec=%+7.3f,%+7.3f,%+7.3f\n", now(), i, (double) accum_deg->degrees, (double) unit.v[0], (double) unit.v[1], (double) unit.v[2]);
printf("%.3f \tAlignment %d  %+7.3f,%+7.3f,%+7.3f\n", now()-start, i, (double) (accum_deg->degrees * unit.v[0]), (double) (accum_deg->degrees * unit.v[1]), (double) (accum_deg->degrees * unit.v[2]));
      }
   }
   if ((k1 > TARGET_ALIGNMENT_TAU) && (update_count > 0)) {
      att->active_tau *= 0.5;
   }
}

