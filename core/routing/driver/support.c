

// set position somewhere in deep water, far from land. this is the
//    state that's reverted to when position data is lost. in effect,
//    what happens is that only tracking data is considered for heading
static void set_dummy_position(void)
{
   world_coordinate_type pos = { .x_deg = 235.0, .y_deg = 30.0 };
   driver_->route.start_position = pos;
   driver_->route.last_known_position = pos;
   driver_->vessel.position = pos;
   // clear position flag
   driver_->route.flags2_persistent = (uint32_t)
         (driver_->route.flags2_persistent & ~ROUTE_INFO_HAVE_POSITION);
}

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
// handle requests posted by other threads

// logic underlying set_destination()
static void handle_set_autotracking(
      /* in     */ const uint32_t on_off
      )
{
   if (on_off == 0) {
      if (autotracking_ != 0) {
         log_info(driver_->log, "Autotracking is now OFF");
         autotracking_ = 0;
      }
   } else {
      if (autotracking_ == 0) {
         log_info(driver_->log, "Autotracking is now ON");
         // set to 2 on init. forces course packet to be sent now
         autotracking_ = 2;   
      }
   }
}


// logic underlying set_autopilot_heading()
static void handle_set_autopilot_heading(
      /* in     */ const uint32_t degs
      )
{
   if (degs >= 360) {
      log_info(driver_->log, "Explicit heading off. Restoring map-based "
            "course");
      set_default_active_course(driver_->path_map);
      otto_heading_degs_ = 512;
   } else {
      log_info(driver_->log, "Explicit heading set to %d", degs);
      bam16_type course;
      CVT_DEG_TO_BAM16((double) degs, course);
      override_active_course_all(driver_->path_map, course);
      otto_heading_degs_ = degs;
   }
}


// logic underlying set_destination()
static void handle_set_destination(
      /* in     */ const world_coordinate_type destination,
      /* in     */ const meter_type radius
      )
{
   log_info(driver_->log, "Setting new destination: lat=%.5f lon=%.5f, "
         "rad=%.1fm", destination.y_deg, destination.x_deg, 
         (double) radius.meters);
   driver_->route.destination = destination;
   if (driver_->route.destination.x_deg < 0.0) {
      driver_->route.destination.x_deg += 360.0;
   }
   driver_->route.destination_radius = radius;
   driver_->route.flags2_persistent |= ROUTE_INFO_HAVE_DESTINATION;
   // disable explicit heading
   otto_heading_degs_ = 512;
   // when destination changes, force full map reload
   driver_->destination_current = 0;
   driver_->map_current = 0;
}


// see if request was made by other thread, e.g., for setting destination
// read request content under mutex, cache request locally, release
//    request, then make changes in this (driver_) thread
// returns 0 if no messages received, 1 otherwise
static int check_for_messages(void)
{
   int received = 0;
   if (driver_->exchange_all != 0) {
log_debug(log_, "Message received");
      received = 1;
      // destination change
      int dest_change = 0;
      world_coordinate_type new_destination  = { .lat=0.0, .lon=0.0 };
      meter_type new_radius = { .meters = 0.0 };
      // autotracking change
      int autotrack_change = 0;
      uint32_t tracking_on_off = 0;
      // heading change
      int head_change = 0;
      uint32_t new_heading_degs = 0;
      //////////////////////////////////////////////////////////////////
      // fetch data
      // acquire lock
      pthread_mutex_lock(&driver_->exchange_mutex);
      if (driver_->destination_change != 0) {
         // destination's changed, which means that we have one, so
         //    set persistent flag
         dest_change = 1;
         new_destination = driver_->new_destination;
         new_radius = driver_->new_radius;
log_debug(log_, "-> dest change");
      }
      if (driver_->autotracking_change != 0) {
         autotrack_change = 1;
         tracking_on_off = driver_->autotracking_on_off;
log_debug(log_, "-> auto change");
      }
      if (driver_->heading_change != 0) {
         head_change = 1;
         new_heading_degs = driver_->new_autopilot_heading_degs;
log_debug(log_, "-> heading change");
      }
      driver_->exchange_all = 0;
      // release lock
      pthread_mutex_unlock(&driver_->exchange_mutex);
      //////////////////////////////////////////////////////////////////
      // handle requests
      if (dest_change == 1) {
         handle_set_destination(new_destination, new_radius);
      }
      if (autotrack_change == 1) {
         handle_set_autotracking(tracking_on_off);
      }
      if (head_change == 1) {
         handle_set_autopilot_heading(new_heading_degs);
      }
   }
   return received;
}

// handle requests posted by other threads
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
//

// if vessel has moved more than X distance from where it was when map
//    was last drawn then it's time to load a new one
static void check_for_stale_map(void)
{
   if (driver_->map_current == 0) {
      // map not loaded -- nothing to do here
      goto end;
   }
   /////////////////////////////////////////////
   // check vessel position since path was last built
   // if it's moved far enough, rebuild path again
   image_coordinate_type pres_pix = get_pix_position_in_map(
         driver_->path_map, driver_->vessel.position);
   path_map_type *path_map = driver_->path_map;
   int32_t mvmt_dx = pres_pix.x - path_map->vessel_start_pix.x;
   int32_t mvmt_dy = pres_pix.y - path_map->vessel_start_pix.y;
meter_type dx,dy;
world_coordinate_type start = { .lon=237.49571, .lat=48.74896 };
calc_meter_offset(driver_->vessel.position, start, &dx, &dy, __func__);
log_debug(log_, "Position %f,%f   pix %d,%d  delta(m) %.1f,%.1f", driver_->vessel.position.x_deg, driver_->vessel.position.y_deg, path_map->vessel_start_pix.x, path_map->vessel_start_pix.y, pres_pix.x, pres_pix.y, dx.meters, dy.meters);
   // we can skip a sqrt (and skip floating point) for the comparison 
   //    if we square threshold limit
   int32_t mvmt_dist2 = mvmt_dx*mvmt_dx + mvmt_dy*mvmt_dy;
   int32_t mvmt_limit2 = VESSEL_MOTION_PIX_FOR_MAP_REBUILD
         * VESSEL_MOTION_PIX_FOR_MAP_REBUILD;
//log_debug(log_, "Movement %f, limit %f", sqrt((double) mvmt_dist2), sqrt((double) mvmt_limit2));
   if (mvmt_dist2 < mvmt_limit2) {
      // vessel hasn't moved far enough -- no rebuild necessary
      goto end;
   }
   /////////////////////////////////////////////
   // vessel has moved far enough to rebuild. but don't bother
   //    if center is close by. dest may be off map, in which case
   //    dest_pix value could be huge. switch to floating point
   // not rebuilding will avoid edge case of map being rebuilt with vessel
   //    and destination in same pixel. some logic paths (at least in
   //    the past) depended on this not happening
   double dest_dx = 
         (double) (path_map->dest_pix.x - path_map->vessel_start_pix.x);
   double dest_dy = 
         (double) (path_map->dest_pix.y - path_map->vessel_start_pix.y);
   double dest_dist2 = dest_dx*dest_dx + dest_dy*dest_dy;
   double dest_limit2 = (double) (PIX_DIST_AVOID_MAP_REBUILD * 
         PIX_DIST_AVOID_MAP_REBUILD);
   if (dest_dist2 < dest_limit2) {
      // destination is pretty close -- skip the rebuild
      goto end;
   }
   // time to rebuild. flag map as stale
log_err(log_, "Flaggin for rebuild");
   driver_->map_current = 0;
end:
   ;
}


// checks to see if a new map should be loaded and loads it if necessary
// TODO if this starts taking very long to calculate (e.g., for long
//    routes) then consider putting it in a separate thread so calculations
//    can be done asynchronously and not take driver offline
static void reload_map(void)
{
static int ctr = 0;
   // [re]load map. requires current-ish position data
   uint32_t flags = driver_->route.flags2_persistent;
   if ((driver_->map_current == 0) && 
         ((flags & ROUTE_INFO_HAVE_POSITION) != 0)) {
      // if destination hasn't changed than we can do a map update, 
      //    otherwise we need a full initialization
      if (driver_->destination_current == 0) {
log_err(log_, "Loading map for fresh destination");
         // this is a fresh map. need to do full initialization
         // destination
         world_coordinate_type dest = driver_->route.destination;
         // if position and/or destination not set then map is not valid,
         //    so set destination equal to position
         if ((flags & ROUTE_INFO_HAVE_POS_DEST_MASK) != 
               ROUTE_INFO_HAVE_POS_DEST_MASK) {
            dest = driver_->vessel.position;
         }
         // vessel position
         world_coordinate_type vessel_pos = driver_->vessel.position;
         // build full map
         if (trace_route_initial(driver_->path_map, dest, vessel_pos) != 0) {
            // failed to trace map for some reason. map is not loaded
            goto end;
         }
         // push declination to attitude controller, but only if position
         //    is known (otherwise could be dummy)
         if (driver_->attitude != NULL) {
            // if attitude is not null then attitude controller is running.
            // don't reset declination if we have no valid position (this
            //    can happen if dummy position is loaded)
            if ((flags & ROUTE_INFO_HAVE_POSITION) != 0) {
               // push declination value
               set_declination(driver_->path_map->declination);
            }
         }
         driver_->destination_current = 1;
      } else {
log_err(log_, "Updating map");
         // update map
         image_coordinate_type pres_pix = get_pix_position_in_map(
               driver_->path_map, driver_->vessel.position);
         rebuild_map_by_vessel_offset(driver_->path_map, pres_pix, 
               driver_->vessel.position);
      }
      // if course override indicated, propagate that into map
      if (otto_heading_degs_ < 360) {
         bam16_type heading;
         CVT_DEG_TO_BAM16((double) otto_heading_degs_, heading);
         override_active_course_all(driver_->path_map, heading);
      }
      driver_->map_current = 1;
      driver_->path_changed = 1;
char buf[STR_LEN];
sprintf(buf, "map-%d.pnm", ctr++);
write_path_map(driver_->path_map, buf);
   }
end:
   ;
}


// update location, speed and heading in route_info
static void plot_course(
      /* in     */ const double t
      )
{
   // cases to consider
   //    course explicitly set Y/N
   //    autotracking on/off
   //    gps feed available Y/N
   //    associator feed available Y/N
   // course selection is handled elsewhere, in check_course
   // whether or not autotracking is active, pretend that it is
   // if GPS feed not available, this is a subset of case that position
   //    is not available. check for that
   // if associator not available treat same as tracking data being stale
   // states
   //    running blind                    0
   //    only tracking available          1
   //    only position available          2
   //    position & tracking available    3
   uint32_t state = 0;
   associator_output_type *ass_out = NULL;
#if defined(USE_TRACKING)
   ass_out = &driver_->associator_out;
   if ((driver_->associator != NULL) &&
         ((t - driver_->associator_sec) <= STALE_POSITION_WINDOW_SEC)) {
      state |= 1;
   }
#endif   // USE_TRACKING
   if ((driver_->route.flags2_persistent | ROUTE_INFO_HAVE_POSITION) != 0) {
      state |= 2;
   }
   switch (state & 0x03) {
      case 0:     // running blind
//log_info(driver_->log, "Running blind");
         break;
      case 1:     // tracking only
         // missing position data. map should be reverted to open-ocean in
         //    this case, so plotting route will only look at traffic
//log_info(driver_->log, "Plotting course, no position data");
//printf("### Plotting course, no position data");
         plot_route(driver_->path_map, driver_->route_map, ass_out,
               &driver_->route, &driver_->vessel, t);
         break;
      case 2:     // position only
//log_info(driver_->log, "Plotting course, no target data");
//printf("### Plotting course, no target data %.3f %.3f\n", t, driver_->associator_sec);
         plot_route(driver_->path_map, driver_->route_map, NULL, 
               &driver_->route, &driver_->vessel, t);
         break;
      case 3:     // tracking and position
         plot_route(driver_->path_map, driver_->route_map, ass_out,
               &driver_->route, &driver_->vessel, t);
         break;
   }
   if (driver_->route.flags_state & ROUTE_INFO_STATE_PATH_LOCAL_MINIMUM) {
      log_err(log_, "Reached local path minimum at %.4f,%.4f. Deleting "
            "beacon", driver_->vessel.position.lon, 
            driver_->vessel.position.lat);

assert(1 == 0);
   }
   // pass in driver_ values separately as unit tests rely on 
   //    passing in independent values
   decide_course_change(t, driver_->last_course_request_sec, 
         &driver_->route, &driver_->vessel);
}


// send course change to autopilot
static void check_course(
      /* in     */ const double t
      )
{
   // bulk of logic here is to determine course to follow. if course
   //    is set, set send_info flag so other data is updated
   int32_t send_info = 0;
   if (autotracking_ == 0) {
//log_info(driver_->log, "Checking course -- autotracking off");
      // autotracking is off. if it's been a while since the last
      //    otto command, send another

      if ((t - driver_->last_otto_command_sec) > OTTO_COMMAND_INTERVAL_SEC) {
         // if degrees to follow was explicitly set,
         //    follow that. otherwise specify an invalid course, which
         //    should bring the tiller to neutral
         if (otto_heading_degs_ < 360) {
            heading_data_.course = (uint16_t) otto_heading_degs_;
         } else {
            heading_data_.course = 512;
         }
         send_info = 1;
      }
      // set 'course' to be present heading. when in autotracking course
      //    is set explicitly. when autotracking is off, tracking computations
      //    are still performed. have them be based on present heading
      driver_->route.autopilot_course = driver_->route.measured_heading;
      driver_->route.autopilot_course_score = 
            driver_->route.measured_heading_score;
      // else, otto packet was recent -- don't send another
   } else if (autotracking_ == 2) {
//log_info(driver_->log, "Checking course -- autotracking just turned on");
      // autotracking just turned on
      // select most recently suggested course as path to follow
      autotracking_ = 1;
      send_info = 1;
      driver_->last_course_request_sec = t;
      heading_data_.course = (uint16_t) 
            (driver_->route.sug_heading.tru.angle32 * BAM32_TO_DEG);
      driver_->route.autopilot_course = driver_->route.sug_heading;
      driver_->route.autopilot_course_score = driver_->route.sug_heading_score;
   } else {
      // autotracking already on
printf("Checking course -- autotracking on\n");
//log_info(driver_->log, "Checking course -- autotracking on");
      if (driver_->route.flags_course & ROUTE_INFO_COURSE_CHANGE_MASK) {
         // course change decided
         log_info(driver_->log, "   course %d, should be %.0f,  severity %d", 
               heading_data_.course,
               (double) driver_->route.sug_heading.tru.angle32 * BAM32_TO_DEG, 
               driver_->route.flags_course);
         heading_data_.course = (uint16_t)
               (driver_->route.sug_heading.tru.angle32 * BAM32_TO_DEG);
         driver_->route.autopilot_course = driver_->route.sug_heading;
         driver_->route.autopilot_course_score = driver_->route.sug_heading_score;
printf("  set pres course (change mask)\n");
         //
         send_info = 1;
         driver_->last_course_request_sec = t;
      } else if (driver_->path_changed != 0) {
log_err(log_, "Reset course");
         // map updated. reset course
         heading_data_.course = (uint16_t)
               (driver_->route.sug_heading.tru.angle32 * BAM32_TO_DEG);
         driver_->route.autopilot_course = driver_->route.sug_heading;
         driver_->route.autopilot_course_score = driver_->route.sug_heading_score;
printf("  set pres course (path changed)\n");
         log_info(driver_->log, "   new course %d", heading_data_.course);
         send_info = 1;
         driver_->last_course_request_sec = t;
      } else if ((t - driver_->last_otto_command_sec) > 
            OTTO_COMMAND_INTERVAL_SEC) {
//log_info(driver_->log, "Checking course -- update heading");
         // it's been a while since the last sent packet. resend course data
         //    and updated heading
         send_info = 1;
      }
      // else, don't send command
   }
   driver_->path_changed = 0;
   /////////////////////////////
   if (send_info == 1) {
//log_info(driver_->log, "  Sending info");
//printf("Sending DPS %.3f\n", driver_->turn_rate.dps);
      heading_data_.dps = (float) driver_->turn_rate.dps;
      heading_data_.heading = (uint16_t) 
            (driver_->attitude_latest.true_heading.degrees + 0.5);
      heading_data_available_ = 1;
   }
}


//
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
// get latest data

// returns 0 if data was received and -1 otherwise
static int32_t get_latest_attitude_data(void)
{
   int32_t rc = 0;
   producer_record_type *pr = driver_->attitude;
   datap_desc_type *prod = pr->producer;
   if (pr->consumed_elements != prod->elements_produced) {
      // advance to most recent data
      pr->consumed_elements = prod->elements_produced;
      // fetch the data -- index position is #consumed -1
      const uint32_t p_idx = (uint32_t)
            (pr->consumed_elements - 1) % prod->queue_length;
      attitude_output_type *out = (attitude_output_type*)
            dp_get_object_at(prod, p_idx);
      memcpy(&driver_->attitude_latest, out, sizeof *out);
      driver_->attitude_sec = prod->ts[p_idx];
      driver_->turn_rate = out->turn_rate;
   }  // else, we already have the most recently measured attitude
   // if attitude data isn't available then there's not much we
   //    can do
   if (driver_->attitude_sec < 0.0) {
      log_info(driver_->log, "Attitude data unavailable. Sleeping again");
      rc = -1;
      goto end;
   }
   // get latest heading. attitude stream should be reliable. if it
   //    breaks then it's still the best guess of heading that we have
   CVT_DEG_TO_BAM32(driver_->attitude_latest.true_heading.degrees,
         driver_->vessel.true_heading.tru);
//   driver_->vessel.true_heading.tru.angle32 = (uint32_t) 
//         (driver_->attitude_latest.true_heading.degrees * DEG_TO_BAM32);
   // FIXME redundant set -- since PS became optional, heading
   //    is no longer set via set_vessel_position() below so needs
   //    to be set seperately. code should be refactored to split
   //    position and heading sets
end:
   return rc;
}


static void get_latest_gps_data(
      /* in     */ const double start_time,
      /* in     */ const double t
      )
{
   if (driver_->gps != NULL) {
      producer_record_type *pr = driver_->gps;
      datap_desc_type *prod = pr->producer;
//      if ((pr->consumed_elements != prod->elements_produced) && 
//            (prod->elements_produced > 0)) {
      if (pr->consumed_elements != prod->elements_produced) {
         // GPS packets can be mixed. to get most recent data, read all
         //    recent and pull as much data as is available
         gps_receiver_output_type combined;
         memset(&combined, 0, sizeof combined);
         double gps_time = 0.0;
         while (pr->consumed_elements < prod->elements_produced) {
            const uint32_t p_idx = 
                  (uint32_t) pr->consumed_elements % prod->queue_length;
            gps_receiver_output_type *out = (gps_receiver_output_type*)
                  dp_get_object_at(prod, p_idx);
            if (out->available & GPS_REC_AVAILABLE_LATITUDE) {
               combined.pos.y_deg = out->pos.y_deg;
            }
            if (out->available & GPS_REC_AVAILABLE_LONGITUDE) {
               combined.pos.x_deg = out->pos.x_deg;
            }
            if (out->available & GPS_REC_AVAILABLE_TRACK) {
               combined.heading = out->heading;
            }
            if (out->available & GPS_REC_AVAILABLE_SPEED) {
               combined.speed = out->speed;
            }
            if (out->available & GPS_REC_AVAILABLE_TIME) {
               combined.zulu_time = out->zulu_time;
            }
            if (out->available & GPS_REC_AVAILABLE_DATE) {
               combined.zulu_date = out->zulu_date;
            }
            combined.available |= out->available;
            gps_time = prod->ts[p_idx];
            pr->consumed_elements++;
         }
         // if GPS has provided data, make sure it's enough to push into
         //    stream
         if ((combined.available & GPS_REC_MIN_DATA_FOR_PUBLISH) == 
               GPS_REC_MIN_DATA_FOR_PUBLISH) {
//printf("DRIVER  gps %d  pos %.4f,%.4f\n", p_idx, out->pos.x_deg, out->pos.y_deg);
            if (combined.pos.x_deg < 0.0) {
               combined.pos.x_deg += 360.0;
            }
            memcpy(&driver_->position_latest, &combined, sizeof combined);
            // if GPS data is very old, trigger a map reset
            if ((gps_time - driver_->position_sec) > GPS_ROUTE_TIMEOUT_SEC) {
               // flag map as dirty so it's reloaded
               driver_->map_current = 0;
               // if this is the first position report set route's
               //    start position
               if (driver_->position_sec < 0.0) {
                  driver_->route.start_position = combined.pos;
               }
            }
            driver_->position_sec = gps_time;
            // only (re)set route speed if GPS has provided that data
            if (combined.available & GPS_REC_AVAILABLE_SPEED) {
               driver_->route.present_speed = combined.speed;
            }
//else { fprintf(stderr, "SPEED NOT AVAILABLE\n"); }
            set_vessel_position(&driver_->vessel, &driver_->route,
                  driver_->position_latest.pos, driver_->route.present_speed, 
                  driver_->vessel.true_heading, driver_->position_sec);
            driver_->route.last_known_position = driver_->position_latest.pos;
         } else {
            // no new position data to use. just update heading
            update_vessel_heading(&driver_->vessel, 
                  driver_->vessel.true_heading, driver_->route.present_speed);
         }
      } else if (pr->consumed_elements > 0) {
         // we already have the most recently measured position.
         //    update heading
         update_vessel_heading(&driver_->vessel, 
               driver_->vessel.true_heading, driver_->route.present_speed);
      }
      // check for currency of data
      // we need a conditional here so this isn't executed before
      //    there's GPS data in the stream
      if (pr->consumed_elements > 0) {
         uint32_t flags = driver_->route.flags2_persistent;
         if ((t - driver_->position_sec) < STALE_POSITION_WINDOW_SEC) {
            // position data is available or at least recently was
            if ((flags & ROUTE_INFO_HAVE_POSITION) == 0) {
               // position is newly acquired
               // set position flag
               driver_->route.flags2_persistent = (uint32_t)
                     (driver_->route.flags2_persistent | 
                     ROUTE_INFO_HAVE_POSITION);
               // set flag to recalculate route map
               driver_->map_current = 0;
            }
         } else {
            // position data is stale
            if ((flags & ROUTE_INFO_HAVE_POSITION) != 0) {
               // flags still indicate position data is available. change that
               // TODO  override active course in map to suggested 
               //    heading so we at least maintain present course
               // TODO  enable some form of dead reckoning to infer position
               driver_->route.flags2_persistent = (uint32_t)
                     (driver_->route.flags2_persistent &
                     ~ROUTE_INFO_HAVE_POSITION);
               // set flag to recalculate route map when position available
               //    again (that should only happen a few lines above here,
               //    so this is duplicate code, but it's safer this way,
               //    e.g., in case map_current is checked by something else
               driver_->map_current = 0;
            }
         }
      }
      // check startup -- don't signal error during startup as being
      //    blind is expected
      if (driver_->route.flags2_persistent & ROUTE_INFO_STARTING_UP_BLIND) {
         if ((t - start_time) > DRIVING_BLIND_OK_WINDOW_SEC) {
            driver_->route.flags2_persistent = (uint32_t)
                  (driver_->route.flags2_persistent &
                  ~ROUTE_INFO_STARTING_UP_BLIND);
         }
      }
   }
}

////////////////////////////////////////////////////////////////////////

#if defined(USE_TRACKING)

// for every record in associator output, calculate trajectory and update
//    appearance
static void compute_trajectories(
      /* in     */ const double t
      )
{
   associator_output_type *ass_out = &driver_->associator_out;
   const uint32_t n_targets = ass_out->num_records;
   dt_second_type dt = { .dt_sec = t - ass_out->alignment_time_sec };
   assert(dt.dt_sec >= 0.0);
   target_record_export_type *targets = ass_out->targets;
   for (uint32_t i=0; i<n_targets; i++) {
      target_record_export_type *record = &targets[i];
      calculate_record_trajectory(record);
      update_target_appearance_and_range(record, dt);
   }
}

// get latest target data
// returns 0 if data is loaded and -1 otherwise
static int32_t get_latest_target_data(
      /* in     */ const double t
      )
{
   int32_t rc = -1;
   if (driver_->associator != NULL) {
      producer_record_type *pr = driver_->associator;
      datap_desc_type *prod = pr->producer;
      if (prod->elements_produced == 0) {
         driver_->associator_out.num_records = 0;
         driver_->associator_out.alignment_time_sec = 0.0;
      } else if (pr->consumed_elements != prod->elements_produced) {
         // advance to most recent data
         pr->consumed_elements = prod->elements_produced;
         // fetch the data -- index position is #consumed -1
         const uint32_t p_idx = (uint32_t)
               (pr->consumed_elements-1) % prod->queue_length;
         // take a pointer to the associator output. this is either
         //    used right away, in which case it's current, or it's
         //    used sometime in the future. but if it's used in the
         //    future, that means that the associator didn't produce
         //    anything since this sample, so this sample will still
         //    be the most current
         const associator_output_type *c_out = 
               (associator_output_type*) dp_get_object_at(prod, p_idx);
         memcpy(&driver_->associator_out, c_out, sizeof *c_out);
         driver_->associator_sec = prod->ts[p_idx];
         // compute trajectories and update appearance for records, in case
         //    this wasn't done previously
         compute_trajectories(t);
         rc = 0;
      }
   }
   return rc;
//   //////////////////////////////////////////////////////////////////
//   // reload_map and compute_path check
//   // do this even if autotracking presently disabled
//   reload_map();
//   compute_path();
//   plot_course(t);
}

#endif   // USE_TRACKING

