#include "pinet.h"
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>
#include <assert.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <errno.h>
#include "timekeeper.h"

#include "routing/driver.h"
#include "routing/mapping.h"
#if defined(USE_TRACKING)
#include "tracking/collision.h"
#endif   // USE_TRACKING


// TODO FIXME if autotracking is enabled when vessel is in too shallow
//    of water, it will turn and steer a course of 000. it should fail
//    to engage and set an alarm

static driver_type * driver_ = NULL;
static datap_desc_type * driver_dp_ = NULL;

// new style approach -- use static log entry to avoid requiring
//    parent pointer
static __thread log_info_type *log_ = NULL;

// default to no autotracking. this can be turned on through
//    config file or charlie (ie, postmaster)
// autotracking on means path-finding and collision avoidance are active,
//    assuming data streams provide sufficient info for this
// values:
//       0 autotracking off
//       1 autotracking on
//       1 autotracking just turned on (sends course packet to otto asap)
static uint32_t autotracking_ = 0;

// course tracking (ie, human sets course; map not used)
// course can be set in one of two ways -- 'hard' and 'soft'
// hard course set tells autopilot to follow compass heading no matter
//    what. autotracking is disabled
// soft set tells autopilot to follow heading but autotracking 
//    (collision avoidance) is active
// this can be achieved using autotracking_ and otto_heading_degs_
//    -> if autotracking is 0, system using hard course following. when
//    autotracking is 1, system permits overriding course to avoid
//    collision
//
// code must be tolerant for position and/or tracking data to be
//    absent

// autopilot told to hold this heading when it's on [0,360)
// overrides map-based (destination-based) course. if autotracking is
//    on then this heading will be adjusted when collision (w/ land
//    or target) is predicted. when off it's a traditional autopilot
// valid headings 0,359
// if abive valid range (typically 512) then autopilot should neutralize
//    tiller if autotracking is off.
static uint32_t otto_heading_degs_ = 512;

// speed hint when speed data unavailable from other sources. start with
//    non-zero value as fallback - that's safer than assuming no motion
//    in case default is not set and speed systems fall offline
static knot_type cruise_speed_ = { .knots = 5.0f };

// inhibit signals until driver ready to go
static int32_t driver_ready_to_run = 0;   // set to 1 in pre_run


#include "comm.c"
//#include "avoidance.c"
#include "route.c"
#include "support.c"

////////////////////////////////////////////////////////////////////////
static void driver_add_producer(
      /* in out */       datap_desc_type *self, 
      /* in     */       datap_desc_type *prod
      )
{
   int num = self->num_attached_producers;
   if (num >= MAX_ATTACHED_PRODUCERS) {
      fprintf(stderr, "Too many producers for driver (max %d). Need "
            "to reconfigure or recompile", MAX_ATTACHED_PRODUCERS);
      fprintf(stderr, "Consumer: %s\n", self->td->obj_name);
      fprintf(stderr, "Producer: %s (%s)\n", prod->td->obj_name, 
            prod->td->class_name);
      hard_exit(__FILE__, __LINE__);
   }
   //////////////////////////////////////////////
   // make sure this is a supported producer type
   if (strcmp(prod->td->class_name, GPS_RECEIVER_CLASS_NAME) == 0) {
      // only allowed to subscribe to one GPS source
      if (driver_->gps != NULL) {
         fprintf(stderr, "Driver '%s' is already subcribed to GPS "
               "'%s'. Attempted to subscribe to '%s'\n", self->td->obj_name,
               driver_->gps->producer->td->obj_name, prod->td->obj_name);
         hard_exit(__FILE__, __LINE__);
      }
      driver_->gps = &self->producer_list[num];
#if defined(USE_TRACKING)
   } else if (strcmp(prod->td->class_name, ASSOCIATOR_CLASS_NAME) == 0) {
      // only allowed to subscribe to one associator source
      if (driver_->associator != NULL) {
         fprintf(stderr, "Driver '%s' is already subcribed to associator "
               "'%s'. Attempted to subscribe to '%s'\n", self->td->obj_name,
               driver_->associator->producer->td->obj_name,
               prod->td->obj_name);
         hard_exit(__FILE__, __LINE__);
      }
      driver_->associator = &self->producer_list[num];
#endif   // USE_TRACKING
   } else if (strcmp(prod->td->class_name, GPS_RECEIVER_CLASS_NAME) == 0) {
      // only allowed to subscribe to one GPS source
      if (driver_->gps != NULL) {
         fprintf(stderr, "Driver '%s' is already subcribed to GPS "
               "'%s'. Attempted to subscribe to '%s'\n", self->td->obj_name,
               driver_->gps->producer->td->obj_name, prod->td->obj_name);
         hard_exit(__FILE__, __LINE__);
      }
      driver_->gps = &self->producer_list[num];
   } else if (strcmp(prod->td->class_name, ATTITUDE_CLASS_NAME) == 0) {
      // only allowed to subscribe to one GPS source
      if (driver_->attitude != NULL) {
         fprintf(stderr, "Driver '%s' is already subcribed to attitude "
               "'%s'. Attempted to subscribe to '%s'\n", self->td->obj_name,
               driver_->gps->producer->td->obj_name, prod->td->obj_name);
         hard_exit(__FILE__, __LINE__);
      }
      driver_->attitude = &self->producer_list[num];
   } else {
      fprintf(stderr, "Attempted to subscribe to incompatible producer\n");
      fprintf(stderr, "Consumer: %s\n", self->td->obj_name);
      fprintf(stderr, "Producer: %s (%s)\n", prod->td->obj_name, 
            prod->td->class_name);
      hard_exit(__FILE__, __LINE__);
   }
   // add producer to list
   self->producer_list[num].producer = prod;
   self->producer_list[num].consumed_elements = 0;
   //
   self->num_attached_producers++;
}


////////////////////////////////////////////////////////////////////////
// pre-run

static void handler_sigusr1(int x)
{
   // triggering this handler should wake the thread, which is what
   //    we wanted
   // swallow the signal itself
   log_info(driver_->log, "Received wakeup signal %d", x);
}


static void driver_pre_run(
      /* in out */       struct datap_desc *self
      )
{
   struct sigaction sa;
   memset(&sa, 0, sizeof sa);
   sa.sa_handler = handler_sigusr1;
   sigaction(SIGUSR1, &sa, NULL);
   //
   uint32_t errs = 0;
   if (driver_->gps == NULL) {
      log_warn(driver_->log, "No GPS subscription (no position data)");
      // TODO set beeper alarm to indicate position data not available
      //    if autotracking activated
      fprintf(stderr, "Driver '%s' is not subscribed to a GPS "
            "(no position data)\n", self->td->obj_name);
   }
   if (driver_->attitude == NULL) {
      // attitude is required for any driver action. consider no
      //    attitude source a fatal error
      fprintf(stderr, "Driver '%s' is not subscribed to an attitude source\n", 
            self->td->obj_name);
      errs++;
   }
#if defined(USE_TRACKING)
   if (driver_->associator == NULL) {
      log_warn(driver_->log, "No associator (no tracking data)");
      // TODO set beeper alarm to indicate tracker data not available
      //    if autotracking activated
      fprintf(stderr, "Driver '%s' is not subscribed to an associator "
            "(-> no tracking data)\n", self->td->obj_name);
   }
#endif   // USE_TRACKING
   if (errs > 0) {
      hard_exit(__FILE__, __LINE__);
   }
   /////////////////////////////////////////////
   int rc = pthread_create(&driver_->comm_tid, NULL, comm_thread_main, self);
   if (rc != 0) {
      log_err(driver_->log, "Failed to launch communication thread: %s\n",
            strerror(rc));
      // treat as a fatal error, at least during development and testing
      hard_exit(__func__, __LINE__);
   }
   /////////////////////////////////////////////
   driver_ready_to_run = 1;
}


////////////////////////////////////////////////////////////////////////
// run

// procedure called by other thread (e.g., via set_destination()) to wake 
//    the driver
void wake_driver(void)
{
   if (driver_ready_to_run) {
      pthread_kill(driver_dp_->td->thread_id, SIGUSR1);
   }
}


static void driver_run(
      /* in out */       struct datap_desc *self
      )
{
   //
   driver_->route.flags2_persistent |= ROUTE_INFO_STARTING_UP_BLIND;
   double start_time = now();
   //
   // normally procedure is to loop and wait for condition variable,
   //    but here we'd rather sleep for a finite period of time and
   //    then wake up and see if we should change course and also to
   //    send heading update information to autopilot
   // assessing traffic need not occur as often as updating heading,
   //    so need to run on multiple parallel clocks
   // TODO clocks should be dynamic -- eg, in mid-ocean w/ no observed
   //    traffic, tracking analysis can be less frequent than in crowded
   //    inland waters
   // heading is updated every waking interval
   //
#if defined(USE_TRACKING)
   // map and course are checked every time there's new associator data.
   //    when that data's not available, these are checked periodically
   // map update interval 
   const double MAP_UPDATE_SEC = 10.0;
   double map_timer = start_time + MAP_UPDATE_SEC;
   // interval between course updates when these occur w/o new tracking data
   //    being available. packet sent to autopilot when course checked,
   //    updating autopilot w/ present heading data
   const double COURSE_UPDATE_SEC = 1.5;
   double course_timer = start_time + MAP_UPDATE_SEC;
#endif   // USE_TRACKING
   //
   const double WAKING_INTERVAL_SEC = 0.1;
   while ((self->run_state & DP_STATE_DONE) == 0) {
      //////////////////////////////////////////
      // sleep until time for next update
      struct timespec ts;
      driver_->waketime = now() + WAKING_INTERVAL_SEC;
      double_to_timespec(driver_->waketime, &ts);
      // before taking nap, check map validity. do this even if 
      //    autotracking presently disabled
      // check is made right before nap so that in case map reload takes
      //    very long it's will have shorter response lag (ie, map update
      //    takes place when driver would otherwise be sleeping)
      // if we've moved more than X miles then it's time to reload map,
      //    or if destination change was made
      check_for_messages();
      check_for_stale_map();  
      reload_map();     // reload map if needed
      while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL) != 0) {
         if ((self->run_state & DP_STATE_DONE) != 0) {
            goto end;
         }
         if (errno == EINTR) {
            break;
         }
         int err = errno;
         log_err(driver_->log, "Error in nanosleep: '%s'", strerror(err));
      }
      //////////////////////////////////////////////////////////////////
      double t = now();
      //////////////////////////////////////////
      // fetch data
      if (get_latest_attitude_data() != 0) {
         continue;
      }
      get_latest_gps_data(start_time, t);
#if defined(USE_TRACKING)
      // check to see if associator has published new data. if so, fetch it
      if (get_latest_target_data(t) < 0) {
         // no new target data
         // if a long enough time has elapsed since the last checking course,
         //    do it again (e.g., to avoid land)
         if (t < map_timer) {
            // nope -- tracking was done recently
            // update heading w/ autopilot and go back to wait
            if (t < course_timer) {
               check_course(t);
               course_timer = t + COURSE_UPDATE_SEC;
            }
            continue;
         }
      }
      //////////////////////////////////////////
      map_timer = t + MAP_UPDATE_SEC;
      course_timer = t + COURSE_UPDATE_SEC;
#endif   // USE_TRACKING
      if (check_for_messages() != 0) {
         // request came in to change state, possibly requiring 
         //    reloading maps. do that before plotting course, even 
         //    if there's a noticable delay
         reload_map();
      }
      plot_course(t);
      // evaluate course and send heading data to autopilot
      check_course(t); // also sends heading data to autopilot
      // check autopilot error
      if ((t - driver_->last_otto_reply_sec) > OTTO_ERR_TIMEOUT_SEC) {
         driver_->route.flags2_persistent |= ROUTE_INFO_AUTOPILOT_ERROR;
      } else {
         driver_->route.flags2_persistent &= ~ROUTE_INFO_AUTOPILOT_ERROR;
      }
      // copy array to output buffer
      // get data sink
      uint32_t idx = (uint32_t) 
            (self->elements_produced % self->queue_length);
      driver_output_type *out = 
            (driver_output_type*) dp_get_object_at(self, idx);
      memcpy(&out->route, &driver_->route, sizeof out->route);
      self->ts[idx] = t;
      ///////////////////////////////
      // all done. let others know
      self->elements_produced++;
      dp_signal_data_available(self);
   }
end:  // use goto label to allow breaking out of inner loop
   ;
}

////////////////////////////////////////////////////////////////////////
// thread may be sleeping which can block exiting -- wake it up
static void driver_abort(struct datap_desc *dp)
{
   (void) dp;
   log_info(driver_->log, "%s aborting", dp->td->obj_name);
   // thread can sleep for seconds at a time. send a wakeup call so
   //    driver can exit now
   if (driver_ready_to_run) {
      pthread_kill(driver_dp_->td->thread_id, SIGUSR1);
   }
   // comm thread may be sleeping too -- wake it so it can exit
   pthread_kill(driver_->comm_tid, SIGUSR1);
}


////////////////////////////////////////////////////////////////////////
static void driver_post_run(struct datap_desc *self)
{
   (void) self;
   //
   int rc = pthread_join(driver_->comm_tid, NULL);
   if (rc != 0) {
      log_err(driver_->log, "Failed in pthread_join: %s\n", strerror(rc));
      // we're exiting so just continue and treat error as fact of life
   }
}


////////////////////////////////////////////////////////////////////////
static void * driver_get_object_at(
      /* in     */ const datap_desc_type *self,
      /* in     */ const uint32_t idx
      )
{  
   return &self->void_queue[idx * sizeof(driver_output_type)];
} 


////////////////////////////////////////////////////////////////////////

void * driver_init(void *driver_setup)
{
   if (driver_ != NULL)
   {
      fprintf(stderr, "Multiple drivers defined -- can only have one\n");
      hard_exit(__FILE__, __LINE__);
   }
   /////////////////////////////////////////////////////////////////////
   datap_desc_type *self = dp_create();
   driver_type *driver = calloc(1, sizeof(*driver));
   driver_ = driver;
   driver_->log = get_logger(DRIVER_CLASS_NAME);
   log_ = driver_->log;
   report_thread_id(self, log_);
   set_log_level(driver_->log, DRIVER_LOG_LEVEL);
   log_calloc(driver, 1, sizeof(*driver));
   self->local = driver;
   driver_dp_ = self;
   //
   //
   driver_setup_type *setup = (driver_setup_type*) driver_setup;
   if (setup == NULL) {
      fprintf(stderr, "Fatal (internal) error -- setup is NULL\n");
      hard_exit(__FILE__, __LINE__);
   }
   free(setup);
   //
   if (init_beacon_list() != 0) {
      fprintf(stderr, "Failed to load map beacons. Maps are presently "
            "required, so this is a hard error\n");
      hard_exit(__FILE__, __LINE__);
   }
   //
   // allocate publish queues (self->void_queue, self->ts)
   self->ts = calloc(1, DRIVER_QUEUE_LEN * sizeof(*self->ts));
   log_calloc(self->ts, 1, DRIVER_QUEUE_LEN * sizeof(*self->ts));
   self->queue_length = DRIVER_QUEUE_LEN;
   self->element_size = sizeof(driver_output_type);
   self->void_queue = calloc(1, DRIVER_QUEUE_LEN * self->element_size);
   log_calloc(self->void_queue, 1, DRIVER_QUEUE_LEN * self->element_size);
   //
   pthread_mutex_init(&driver_->exchange_mutex, NULL);
   //
   image_size_type world_map_size = 
         { .x = WORLD_MAP_WIDTH_NODES, .y = WORLD_MAP_HEIGHT_NODES };
   driver_->path_map = create_path_map(world_map_size);
   meter_type route_node_width = { .meters = ROUTE_NODE_WIDTH_MET };
   driver_->route_map = create_route_map(route_node_width);
   // set negative position times to flag that position data is invalid
   // this should disable using map for routing info
   driver_->vessel.estimated_pos_time = 0;
   driver_->vessel.confirmed_pos_time = 0;
   driver_->vessel.position_accuracy.meters = 1000.0;
#if defined(USE_TRACKING)
   driver_->associator_sec = -1000.0;
#endif   // USE_TRACKING
   driver_->position_sec = -1000.0;
   driver_->attitude_sec = -1000.0;
   // initialize position to somewhere in deep water, far from land
   set_dummy_position();
   // in case default speed was set before driver was created, pull in
   //    cashed value
   driver_->route.default_speed.mps = cruise_speed_.knots * KNOTS_TO_MPS;
   //
   self->add_producer = driver_add_producer;
   self->pre_run = driver_pre_run;
   self->post_run = driver_post_run;
   self->abort = driver_abort;
   self->run = driver_run;
   self->get_object_at = driver_get_object_at;
   // once initialization done, put into runtime mode
   dp_execute(self);
   return NULL;
}

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

// turn self-driving interface on (1) or off (0)
void set_autotracking(
      /* in     */ const uint32_t on_off
      )
{
   // request has been made by. remember it and handle request in 
   //    the driver thread
   pthread_mutex_lock(&driver_->exchange_mutex);
   driver_->autotracking_on_off = on_off;
   driver_->autotracking_change = 1;
   pthread_mutex_unlock(&driver_->exchange_mutex);
   wake_driver();
}


// manually set autopilot heading. this overrides course settings
//    in map. on negative heading, map-based course is restored
// valid is 0-359. a higher value will disable manual heading set
void set_autopilot_heading(
      /* in     */ const uint32_t degs
      )
{
   // request has been made by. remember it and handle request in 
   //    the driver thread
   pthread_mutex_lock(&driver_->exchange_mutex);
   driver_->new_autopilot_heading_degs = degs;
   driver_->heading_change = 1;
   pthread_mutex_unlock(&driver_->exchange_mutex);
   wake_driver();
}


void set_destination(
      /* in     */ const world_coordinate_type destination,
      /* in     */ const meter_type radius
      )
{
   // verify that driver is assigned. remote edge case (pointed out by
   //    compiler) is that external call to postmaster setting direction
   //    can theoretically occur before driver is created
   if (driver_ == NULL) {
      return;
   }
   // request has been made by. remember it and handle request in 
   //    the driver thread
   pthread_mutex_lock(&driver_->exchange_mutex);
   driver_->destination_change = 1;
   driver_->new_destination = destination;
   driver_->new_radius = radius;
   if (driver_->new_destination.x_deg < 0.0) {
      driver_->new_destination.x_deg += 360.0;
   }
   pthread_mutex_unlock(&driver_->exchange_mutex);
   wake_driver();
}


// only to be called during initialization, to provide a rough position
//    so that map can be loaded
void set_default_cruise_speed_kts(
      /* in     */ const double kts
      )
{
   // this may be called before driver is created. in that case, store
   //    value. let driver read it when it starts
   cruise_speed_.knots = kts;
   if (driver_ != NULL) {
      // driver is available. update default speed directly
      driver_->route.default_speed.mps = kts * KNOTS_TO_MPS;
   }
}


