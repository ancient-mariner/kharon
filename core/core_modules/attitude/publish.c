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

static void write_to_log(
      /* in     */       FILE *logfile,
      /* in     */ const double t,
      /* in     */ const attitude_output_type *out
      )
{
//   if (logfile) {
//      matrix_type *rot = &out->ship2world;
//      fprintf(logfile, "%.3f [[%7.3f,%7.3f,%7.3f];[%7.3f,%7.3f,%7.3f];"
//            "[%7.3f,%7.3f,%7.3f]]\n", t,
//            rot->m[0], rot->m[1], rot->m[2],
//            rot->m[3], rot->m[4], rot->m[5],
//            rot->m[6], rot->m[7], rot->m[8]);
//   }
   if (logfile) {
      fprintf(logfile, "%.3f  \t%4.2f  \t%4.2f  \t%4.2f  %.2f\n", t,
            out->true_heading.degrees,
            out->pitch.degrees,
            out->roll.degrees,
            out->turn_rate.dps);
//      fprintf(logfile, "   acc  %.3f %.3f %.3f\n",
//            out->acc.v[0],
//            out->acc.v[1],
//            out->acc.v[2]);
//      fprintf(logfile, "   mag  %.3f %.3f %.3f\n",
//            out->mag.v[0],
//            out->mag.v[1],
//            out->mag.v[2]);
   }
}

////////////////////////////////////////////////////////////////////////
// apply filter

// unitify vector and report vectors original length
static void normalize_vector(
      /* in     */ const vector_type *in,
      /*    out */       vector_type *out,
      /*    out */       double *len
      )
{
   const double *v = in->v;
   double length = sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
   if (length > 0.0) {
      for (uint32_t i=0; i<3; i++) {
         out->v[i] = v[i] / length;
      }
   }
   *len = length;
}


// apply predicted error to output of complementary filter
static void apply_filter_correction(
      /* in out */       attitude_class_type *att,
      /* in     */ const vector_type *acc,
      /* in     */ const double k_acc,
      /* in     */ const vector_type *mag,
      /* in     */ const double k_mag
      )
{
//copy_vector(&att->comp_acc, &att->corrected_acc);
//copy_vector(&att->comp_mag, &att->corrected_mag);
//return;
   // measure difference between measured acc/mag and complementary
   //    filter values
   vector_type acc_err_axis, mag_err_axis;
   degree_type acc_theta, mag_theta;
   measure_rotation(&att->comp_acc, acc, &acc_err_axis, &acc_theta);
   measure_rotation(&att->comp_mag, mag, &mag_err_axis, &mag_theta);
//log_info(log_, "measured ACC %.5f %.5f,%.5f   err %.5f,%.5f,%.5f   %.5f", acc->v[0], acc->v[1], acc->v[2], acc_err_axis.v[0], acc_err_axis.v[1], acc_err_axis.v[2], acc_theta.degrees);
//log_info(log_, "         MAG %.5f %.5f,%.5f   err %.5f,%.5f,%.5f   %.5f", mag->v[0], mag->v[1], mag->v[2], mag_err_axis.v[0], mag_err_axis.v[1], mag_err_axis.v[2], mag_theta.degrees);
   //
   unitify(&acc_err_axis);
   unitify(&mag_err_axis);
   for (uint32_t i=0; i<3; i++) {
      acc_err_axis.v[i] *= acc_theta.degrees;
      mag_err_axis.v[i] *= mag_theta.degrees;
   }
   // update error running average
   for (uint32_t i=0; i<3; i++) {
      att->est_error_acc.v[i] = (1.0 - k_acc) * att->est_error_acc.v[i] +
            k_acc * acc_err_axis.v[i];
      att->est_error_mag.v[i] = (1.0 - k_mag) * att->est_error_mag.v[i] +
            k_mag * mag_err_axis.v[i];
   }
//log_info(log_, "Avg ACC error   %.4f,%.4f,%.4f   %.3f deg", att->est_error_acc.v[0], att->est_error_acc.v[1], att->est_error_acc.v[2], vector_len(&att->est_error_acc));
//log_info(log_, "    MAG error   %.4f,%.4f,%.4f   %.3f deg", att->est_error_mag.v[0], att->est_error_mag.v[1], att->est_error_mag.v[2], vector_len(&att->est_error_mag));
   // apply correction to complementary filter
   acc_theta.degrees = vector_len(&att->est_error_acc);
   rotate_vector_about_axis(&att->est_error_acc, &att->comp_acc,
         acc_theta, &att->corrected_acc);
   mag_theta.degrees = vector_len(&att->est_error_mag);
   rotate_vector_about_axis(&att->est_error_mag, &att->comp_mag,
         mag_theta, &att->corrected_mag);
//log_info(log_, "ACC corr %.5f,%.5f,%.5f -> %.5f,%.5f,%.5f   %.5f", att->comp_acc.v[0], att->comp_acc.v[1], att->comp_acc.v[2], att->corrected_acc.v[0], att->corrected_acc.v[1], att->corrected_acc.v[2], acc_theta.degrees);
//log_info(log_, "MAG corr %.6f,%.6f,%.6f -> %.6f,%.6f,%.6f   %.5f", att->comp_mag.v[0], att->comp_mag.v[1], att->comp_mag.v[2], att->corrected_mag.v[0], att->corrected_mag.v[1], att->corrected_mag.v[2], mag_theta.degrees);
//////////////////////////////////////////////////
}


// apply complementary filter to gyro and generate new ship2world matrix
// filter adapted with predictive error
static void apply_filter(
      /* in out */       attitude_class_type *att,
      /* in     */ const vector_type *gyr,
      /* in     */ const vector_type *acc,
      /* in     */ const vector_type *mag,
      /* in     */ const dt_second_type dt
      )
{
//printf("Applying filter. Reset ms = %f\n", (double) att->reset_ms.msec * 0.001);
   ///////////////////////
   // filter logic is to apply measured rotation to approximated
   //    attitude vector and to do a weighted average between that
   //    and measured acc,mag
   //////////////////
   // normalize input signals, keeping a record of their magnitude
   vector_type unit_acc, unit_mag;
   normalize_vector(acc, &unit_acc, &att->acc_len);
   normalize_vector(mag, &unit_mag, &att->mag_len);
   ///////////////////////
   // calculate attitude
   vector_type axis;
   degree_type theta;
   gyro_vector_to_rotation_axis(gyr, &axis, &theta);
//log_info(log_, "GYR %.3f,%.3f,%.3f    %.3f deg\n", gyr->v[0], gyr->v[1], gyr->v[2], theta.degrees);
//printf("MAG Theta = %.3f   about %.3f,%.3f,%.3f\n", theta.degrees, axis.v[0], axis.v[1], axis.v[2]);
   // theta is in degrees per second. adjust to degrees per sample
   //theta.degrees *= SAMPLE_DUR_SEC;
   degree_type rotation = { .degrees = theta.degrees * dt.dt_sec };
   // TODO if rotation is too much optical_up can break (e.g., during
   //    initialization if theta is not multiplied by SAMPLE_DUR_SEC
   // this indicates there's a built-in crashing bug FIXME
//log_info(log_, "GYR %.4f,%.4f,%.4f    dps:%.4f  rot:%.4f (%.3f)", gyr->v[0], gyr->v[1], gyr->v[2], theta.degrees, rotation.degrees, dt.dt_sec);
   // att->mag are unit vectors
   vector_type new_acc, new_mag;
   rotate_vector_about_axis(&axis, &att->comp_acc, rotation, &new_acc);
   rotate_vector_about_axis(&axis, &att->comp_mag, rotation, &new_mag);
//log_info(log_, "measured ACC %.3f %.3f,%.3f   comp %.4f,%.4f,%.4f", acc->v[0], acc->v[1], acc->v[2], att->comp_acc.v[0], att->comp_acc.v[1], att->comp_acc.v[2]);
//log_info(log_, "         MAG %.3f %.3f,%.3f   comp %.4f,%.4f,%.4f", mag->v[0], mag->v[1], mag->v[2], att->comp_mag.v[0], att->comp_mag.v[1], att->comp_mag.v[2]);
   ////////////////////////
   // complementary filter
   // weight for merging new signal and existing
   double k_mag = COMPLEMENTARY_TAU_MAG;
   double k_acc = COMPLEMENTARY_TAU_ACC;
   // if in bootstrap then use different scale
   if (att->init_timer.seconds > 0.0) {
      // if the timer is at max then this is either a freshly started system
      //    or the gyro has been unavailable for too long. reset attitude's
      //    acc and mag to whatever is reported now as the old signal we
      //    can no longer trust. then skip complementary filter
      if (att->init_timer.seconds >= BOOTSTRAP_INTERVAL_SEC) {
         att->init_timer.seconds = BOOTSTRAP_INTERVAL_SEC - SAMPLE_DUR_SEC;
         copy_vector(&unit_acc, &att->comp_acc);
         copy_vector(&unit_mag, &att->comp_mag);
         goto end;
      }
      // change tau linearly from ~1/20 to COMPLEMENTARY_TAU as bootstrap
      //    period goes from full to zero. when tau starts out too high
      //    there's unacceptably high motion (eg, tau starting at 1/5
      //    can yield up to 50dps variation in the first few hundred ms)
      double k = (double)
            (0.05 * att->init_timer.seconds / BOOTSTRAP_INTERVAL_SEC);
      k_acc = (1.0 - k) * COMPLEMENTARY_TAU_ACC + k;
      k_mag = (1.0 - k) * COMPLEMENTARY_TAU_MAG + k;
      att->init_timer.seconds -= SAMPLE_DUR_SEC;
//log_info(log_, "Init timer %.3f    k_acc %.3f   k_mag %.3f", att->init_timer.seconds, k_acc, k_mag);
   }
   // merge gyro-rotated vector with ACC and MAG measurements and store
   //    in att, w/ gyro. these will be pulled from class when publishing
   //    data (extraneous data copy, but simplifies logic of code flow)
   for (uint32_t i=0; i<3; i++) {
      att->comp_acc.v[i] = k_acc * unit_acc.v[i] + (1.0 - k_acc) * new_acc.v[i];
      att->comp_mag.v[i] = k_mag * unit_mag.v[i] + (1.0 - k_mag) * new_mag.v[i];
   }
   // compensate for approximate error in complementary filter
   apply_filter_correction(att, &unit_acc, k_acc, &unit_mag, k_mag/4.0);
//printf("MAG sens:%.5f,%.5f  (%.3f)  filt:%.5f,%.5f -> %.5f,%.5f\n", unit_mag.v[0], unit_mag.v[2], R2D * atan2(unit_mag.v[0], unit_mag.v[2]), new_mag.v[0], new_mag.v[2], att->mag.v[0], att->mag.v[2]);
   // copy gyro unchanged
   copy_vector(gyr, &att->gyr);
end:
   ;
}

// apply filter
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
// publish data

// apply complementary filter to gyro and generate new ship2world matrix
static void publish_data(
      /* in out */       attitude_class_type *att,
      /* in out */       datap_desc_type *self
      )
{
   /////////////////////////////////////////////
   // get data sink
   const uint32_t idx =
         (uint32_t) self->elements_produced % self->queue_length;
   const double t = real_from_timestamp(att->next_publish_time);
   self->ts[idx] = t;
   attitude_output_type *out = (attitude_output_type*)
         dp_get_object_at(self, idx);
   /////////////////////////////////////////////
   // write data to output
   copy_vector(&att->corrected_acc, &out->acc);
   copy_vector(&att->corrected_mag, &out->mag);
   copy_vector(&att->gyr, &out->gyr);
   out->acc_len = att->acc_len;
   out->mag_len = att->mag_len;
   // build ship2world matrix
   // acc is y. collapse mag to acc plane to get ship's z axis
   vector_type ship_z;
   project_onto_plane(&att->corrected_acc, &att->corrected_mag, &ship_z);
   unitify(&ship_z);
   build_orthogonal_matrix_yz(&att->corrected_acc, &ship_z, &out->ship2world);
//print_mat(&out->ship2world, "ship2world");
   // heading -- apply ship2world to z axis to calculate heading. this can
   //    be pulled directly from ship2world or done explicitly (one form
   //    of code is commented out)
   //vector_type world_mag;
   //vector_type z_axis = { .v = { 0.0f, 0.0f, 1.0f } };
   //mult_matrix_vector(&out->ship2world, &z_axis, &world_mag);
   //out->heading.degrees = atan2f(world_mag.v[0], world_mag.v[2]) * R2D;
   out->mag_heading_reference.degrees =
         atan2(out->ship2world.m[6], out->ship2world.m[8]) * R2D;
   if (out->mag_heading_reference.degrees < 0.0) {
      out->mag_heading_reference.degrees += 360.0;
   }
   out->true_heading.degrees =
         out->mag_heading_reference.degrees - declination_.degrees;
   if (out->true_heading.degrees < 0.0) {
      out->true_heading.degrees += 360.0;
   }
   double dt = t - att->heading_sec;
   double dps = 0.0;
   if (dt > 0.0) {
      double d_heading =
            out->mag_heading_reference.degrees - att->mag_heading.degrees;
      if (d_heading > 180.0) {
         d_heading -= 360.0;
      } else if (d_heading <= -180.0) {
         d_heading += 360.0;
      }
      dps = d_heading / dt;
   }
   // bad behavior -- burying constants in code
   // to estimating turn rate and reduce noise, keep running average w/
   //    tau of ~1/2 second
   const double turn_rate_tau = 1.0 / (double) (SAMPLE_FREQ_HZ/2);
   att->turn_rate.dps = (1.0 - turn_rate_tau) * att->turn_rate.dps +
         turn_rate_tau * dps;
   att->mag_heading = out->mag_heading_reference;
   att->heading_sec = t;
   out->turn_rate = att->turn_rate;
//log_info(log_, "MATR corners 0268   %.5f %.5f  %.5f %.5f", out->ship2world.m[0], out->ship2world.m[2], out->ship2world.m[6], out->ship2world.m[8]);
//printf("MAG  %.5f,x,%.5f   mat %.5f,x,%.5f  gyr-y %.4f  Heading: %.4f\n", att->mag.v[0], att->mag.v[2], out->ship2world.m[6], out->ship2world.m[8], att->gyr.v[1], out->heading.degrees);
//printf("MAG    t=%.3f    heading %.2f\n", self->ts[idx], out->heading.degrees);
   // use ship's acc vector to calculate pitch and roll
   // roll -- project acc to XY plane and normalize. roll is asin(x)
   vector_type roll_vec = { .v =
         { att->corrected_acc.v[0], att->corrected_acc.v[1], 0.0 } };
   unitify(&roll_vec);
   double roll_x = roll_vec.v[0];
   // trap for roundoff errors that bring |x| above 1.0 (it happens, or
   //    at least did when using float)
   if (roll_x > 1.0) {
      roll_x = 1.0;
   } else if (roll_x < -1.0) {
      roll_x = -1.0;
   }
   out->roll.degrees = -asin(roll_x) * R2D;
   // pitch -- project acc to YZ plane and normalize. pitch is asin(z)
   vector_type pitch_vec = { .v =
         { 0.0, att->corrected_acc.v[1], att->corrected_acc.v[2] } };
   unitify(&pitch_vec);
   double pitch_z = pitch_vec.v[2];
   if (pitch_z > 1.0) {
      pitch_z = 1.0;
   } else if (pitch_z < -1.0) {
      pitch_z = -1.0;
   }
   out->pitch.degrees = -asin(pitch_z) * R2D;
//log_info(log_, "Heading: %.4f  roll: %.4f  pitch: %.4f  turn: %.4f", out->heading.degrees, out->roll.degrees, out->pitch.degrees, out->turn_rate.dps);
   /////////////////////////////////////////////
   // output is set. finish up and publish
   att->next_publish_time.usec += SAMPLE_DUR_USEC;
   // publish regardless of whether in initialization or not. runtime re-init
   //    (eg, due force publish) causes blackout that subscribers interpolate
   //    through anyway, so not publishing doesn't help there. on fresh
   //    init, everything's a mess so there's not much to be lost for
   //    initial bad data, which is much more likely during first part of init
   if (att->logfile) {
      write_to_log(att->logfile,
            real_from_timestamp(att->next_publish_time), out);
   }
   self->elements_produced++;
   dp_signal_data_available(self);
}


