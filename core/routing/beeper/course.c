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

// on suggested change, make beep notification every 30 seconds
// on requested change, make klaxon notification every 10 seconds
// for evasion, sound klaxon alarm continuously
static void examine_course_data_heading(
      /* in     */ const driver_output_type *driver_out,
      /* in     */ const double t
      )
{
   const route_info_type *route = &driver_out->route;
//   if (route->flags_course & ROUTE_INFO_COURSE_EVASION) {
//      log_info(beeper_->log, "Evasion indicated (0x%04x)",
//            route->flags_course);
//      requests_.klaxon_alarm = 1;
//   } else
   if (route->flags_course & ROUTE_INFO_COURSE_MAKE_CHANGE) {
      // get time of most previous alert
      double dt = t - beeper_->last_change_alert;
      if (dt > BEEPER_MAKE_CHANGE_INTERVAL) {
         log_info(beeper_->log, "Requesting change. Turn %.1f to "
               "%02.0f  %.1f",
               (double) route->turn_rate.dps,
               (double) route->sug_heading.tru.angle32 * BAM32_TO_DEG,
               (double) route->true_path_heading.angle16 * BAM16_TO_DEG);
         // turn in suggested direction
         if (route->turn_rate.dps > 0.0) {
            requests_.beep_left = 1;
            beeper_->last_change_alert = t;
         } else if (route->turn_rate.dps < 0.0) {
            requests_.beep_right = 1;
            beeper_->last_change_alert = t;
         }
      }
   } else if (route->flags_course & ROUTE_INFO_COURSE_SUGGEST_CHANGE) {
      // get time of most previous alert
      double dt = t - beeper_->last_change_alert;
      if (dt > BEEPER_SUGGEST_CHANGE_INTERVAL) {
         log_info(beeper_->log, "Suggesting change. Turn %.1f to "
               "%02.0f  %.1f",
               (double) route->sug_heading.tru.angle32 * BAM32_TO_DEG,
               (double) route->true_path_heading.angle16 * BAM16_TO_DEG);
         // turn in suggested direction
         if (route->turn_rate.dps > 0.0) {
            requests_.beep_left = 1;
            beeper_->last_change_alert = t;
         } else if (route->turn_rate.dps < 0.0) {
            requests_.beep_right = 1;
            beeper_->last_change_alert = t;
         }
      }
   }
}

// on suggested change, make beep notification every 30 seconds
// on requested change, make klaxon notification every 10 seconds
// for evasion, sound klaxon alarm continuously
static void examine_course_data_speed(
      /* in     */ const driver_output_type *driver_out,
      /* in     */ const double t
      )
{
   (void) t;
   const route_info_type *route = &driver_out->route;
   // define sounds and trigger as appropriate
   if (route->flags_speed & ROUTE_INFO_SPEED_FULL_STOP) {
      log_info(beeper_->log, "Fullstop indicated - trigger klaxon");
      requests_.klaxon_fullstop = 1;
   } else if (route->flags_state & ROUTE_INFO_STATE_RUNNING_BLIND) {
      if (route->flags2_persistent & ROUTE_INFO_STARTING_UP_BLIND) {
         log_info(beeper_->log, "Running blind during startup -- ping");
         // when starting up, don't blare the klaxon if we're
         //    blind for a few seconds. let it be known that
         //    there's no signal, but don't scare anyone
         requests_.ping_soft = 1;
      } else {
         log_info(beeper_->log, "Running blind - trigger klaxon");
         requests_.klaxon_fullstop = 1;
      }
   }
}


static void examine_course_data(
      /* in     */ const driver_output_type *driver_out,
      /* in     */ const double t
      )
{
   uint32_t flags2 = driver_out->route.flags2_persistent;
   // doun't inhibit routing alerts even when autopilot active, at least for now
   examine_course_data_heading(driver_out, t);
   examine_course_data_speed(driver_out, t);
   if (flags2 & ROUTE_INFO_AUTOPILOT_ERROR) {
      requests_.klaxon_autopilot_error = 1;
   }
//   if ((flags2 & ROUTE_INFO_AUTOPILOT_ACTIVE) == 0) {
//      examine_course_data_heading(driver_out, t);
//      examine_course_data_speed(driver_out, t);
//   } else if (flags2 & ROUTE_INFO_AUTOPILOT_ERROR) {
//      requests_.klaxon_autopilot_error = 1;
//   }
}

