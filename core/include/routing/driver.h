#if !defined(DRIVER_H)
#define DRIVER_H
#if !defined(_GNU_SOURCE)
#define _GNU_SOURCE
#endif   // _GNU_SOURCE
#include "pinet.h"
#include "datap.h"
#include "logger.h"

#if defined(USE_TRACKING)
#include "tracking/associator.h"
#else
struct associator_output;
typedef struct associator_output associator_output_type;
#endif

#include "routing/mapping.h"
#include "core_modules/gps_receiver.h"
#include "core_modules/attitude.h"

#define DRIVER_LOG_LEVEL     LOG_LEVEL_DEFAULT

// TODO create 'position' module between GPS and here that's also
//    capable of DR

// takes input target associator and position/movement data (GPS),
//    combines this with mapping to determine collision and land avoidance
//    paths, and makes decision on what path to take
//
// a destination must be provided. path is recomputed if/when destination
//    changes

// how many seconds old the most recent targetting info can be and still
//    be considered useful. if data is too old then collision avoidance
//    is turned off due lack of data
// note that even if target records are available more recently, that doesn't
//    mean that the records are complete. there will be blank areas when
//    targetting is first coming online, and if there's not 360d camera
//    coverage there will obviously be incomplete coverage
#define TARGET_RECORD_USE_BY_WINDOW_SEC      15.0


// amount of time that GPS is offline that, after it comes back, the 
//    map is reloaded and route recomputed
#define GPS_ROUTE_TIMEOUT_SEC    600.0

// time w/o position data for map to no longer be consulted
// TODO consider implementing some form of DR to extend this window. 
//    incorporate uncertainty window
// TODO consider adding position uncertainty window logic, with window
//    growing while data unavailable
// TODO provide alert when position data goes offline
#define STALE_POSITION_WINDOW_SEC   30.0

// when first starting up the system will be driving blind, if it's
//    driving at all. inhibit alerts during this time as it's expected.
//    perhaps trigger a pleasant beep during this period instead of 
//    alarm klaxon
#define DRIVING_BLIND_OK_WINDOW_SEC    30.0

// approx interval for heading/course data packets to be sent to
//    autopilot
#define OTTO_COMMAND_INTERVAL_SEC    0.250

// interval such that if no response heard from autopilot in this time,
//    autopilot error is signalled
#define OTTO_ERR_TIMEOUT_SEC     5.0

// once course change sent to autopilot, otto acknowledges change. however,
//    ack may not be received before next decision for course change
//    is made. defer subsequent decisions for this interval to allow otto 
//    to ack previous request
#define OTTO_COURSE_CHANGE_RESPSONSE_WINDOW_SEC      1.5

////////////////////////////////////////////////
// number of pixels a vessel must move since last path update for path
//    to be updated again
// vessel's placement in map should be offset X miles from map center
//    (at time of this comment's writing) with center in the direction
//    that vessel should go to reach destination
// map is 720 pixels wide, which is 60 miles, so 12 pixels per mile
// the distance here should be less than mapping.h's
//    VESSEL_BEACON_INHIBITION_RING_NM in case beacon is local minimum.
//    that should allow vessel to escape, or to never be drawn into it
#define VESSEL_MOTION_PIX_FOR_MAP_REBUILD       (1 * 12)
// ('12' here is 720/60, or number of pixels per nautical mile)

// map will not be regenerated if destination is sufficiently close
#define PIX_DIST_AVOID_MAP_REBUILD              (10 * 12)

////////////////////////////////////////////////
// route viability interval bounds (seconds)
// at 5 knots (~2.6mps), distance traveled in 10 seconds is ~26m (~85ft)

#define VIABILITY_INTERVAL_START_0     0.0
#define VIABILITY_INTERVAL_START_1    12.0
#define VIABILITY_INTERVAL_START_2    25.0
#define VIABILITY_INTERVAL_START_3    45.0
#define VIABILITY_INTERVAL_START_4    90.0

#define VIABILITY_INTERVAL_END_0       VIABILITY_INTERVAL_START_1
#define VIABILITY_INTERVAL_END_1       VIABILITY_INTERVAL_START_2
#define VIABILITY_INTERVAL_END_2       VIABILITY_INTERVAL_START_3
#define VIABILITY_INTERVAL_END_3       VIABILITY_INTERVAL_START_4
#define VIABILITY_INTERVAL_END_4       1.0e10


// NOTE it's possible for sequential outputs to have same timestamp.
//    this can occur if data is received from one modality that's
//    older than data has been published from a different one
struct driver_output {
   route_info_type route;
};
typedef struct driver_output driver_output_type;

// driver_output is published to network (eg, for otto)

// queue size needn't be large, but make sure that it's enough so that
//    entire queue isn't run through between wakings of communication
//    thread so that index will be changed if data is available
#define DRIVER_QUEUE_LEN   64

//
////////////////////////////////////////////////////////////////////////
//

#define DRIVER_CLASS_NAME  "driver"

#define WORLD_MAP_WIDTH_NODES    WORLD_MAP_5SEC_WIDTH
#define WORLD_MAP_HEIGHT_NODES   WORLD_MAP_5SEC_HEIGHT

#define ROUTE_NODE_WIDTH_MET     20.0

struct driver_class {
   log_info_type *log;
   // clock time that driver should wake up to recheck status
   double waketime;
   // present route info
   // route memory is written to by multiple threads. the field
   //    course_changed_sec is written to by comm thread, while all
   //    others are written by main driver thread. comm thread 
   //    shouldn't update at more than 1Hz (usually much less) so
   //    the cache coherency issues shouldn't affect performance
   //    too mcuh
   route_info_type   route;
   // state data about the vessel
   vessel_position_info_type  vessel;
   /////////////////////////////////////////////
   // maps and planning
   path_map_type *path_map;
   route_map_type *route_map;
   /////////////////////////////////////////////
   // control flags 
   union {
      // all values are zero except for the conditions as described
      struct {
         // 1 if destination has been provided to map, allowing
         //    it to develop high-level plan, and 0 if destination
         //    is new or changed
         uint8_t destination_current;
         // 1 if map is up-to-date or at least deemed usable, and 0
         //    if it's time to (re)load it
         uint8_t map_current;
         // 1 indicates path needs to be recomputed (this happens
         //    if map or destination are updated)
         uint8_t path_changed;
         uint8_t unused_2;
      };
      uint32_t flags_all;
   };
   /////////////////////////////////////////////
   // multi-threading interface
   // content here should only be changed when there's a mutex lock,
   //    as it's modifiable by this and other treads
   union {
      // all values are zero except for the conditions as described
      struct {
         // set to 1 when there's been a call to change one of 
         //    these values from an outside thread
         uint8_t destination_change;
         uint8_t autotracking_change;
         uint8_t heading_change;
         uint8_t unused;
      };
      uint32_t exchange_all;
   };
   // new destination
   world_coordinate_type new_destination;
   meter_type new_radius;
   // autotracking
   uint32_t autotracking_on_off; // turned on is 1, off 0
   // new heading in degrees, on [0,359], or >359 to disable
   uint32_t new_autopilot_heading_degs; 
   //
   pthread_mutex_t exchange_mutex;
   /////////////////////////////////////////////////////////////////////
   // communication thread (to communicate w/ autopilot)
   pthread_t comm_tid;
   // time of last data packet sent to remote. written by main thread
   double last_otto_command_sec;
   // time of last data packet sent to remote requesting course change
   // keep track of this separately so multiple changes aren't sent
   //    while waiting for otto to acknowledge previous request
   double last_course_request_sec;
   // time of last data received from remote. written by comm thread
   double last_otto_reply_sec;
   /////////////////////////////////////////////////////////////////////
   // shortcuts to producers and their output
#if defined(USE_TRACKING)
   producer_record_type *associator;
   // local copy of associator output, as this will be modified
   associator_output_type associator_out;
   double associator_sec;
#endif   // USE_TRACKING
   // 
   producer_record_type *gps;
   gps_receiver_output_type position_latest;
   double position_sec;
   // 
   producer_record_type *attitude;
   attitude_output_type attitude_latest;
   double attitude_sec;
   degree_per_second_type turn_rate;
};
typedef struct driver_class driver_class_type;
typedef struct driver_class driver_type;

// thread entry point
void * driver_init(void *);

// struct to pass config data to thread
struct driver_setup {
   int unused;
};
typedef struct driver_setup driver_setup_type;

////////////////////////////////////////////////////////////////////////
// API

// turns on/off dynamic autopilot control 
void set_autotracking(
      /* in     */ const uint32_t on_off
      );

// set autopilot on specific heading. turns off dynamic control
// if set value is >= 360 then explicit heading is turned off. if
//    autotracking is on then course control will be based on position
//    and destination. if autotracking is off, this should neutralize
//    the tiller
void set_autopilot_heading(
      /* in     */ const uint32_t degs
      );

// set new destination. induces new path calculation and route update
// FIXME this may generate a race condition if called after 
//    initialization (unlikely, but very possible)
void set_destination(
      /* in     */ const world_coordinate_type destination,
      /* in     */ const meter_type radius
      );


// only to be called during initialization, to provide a rough position
//    so that map can be loaded
void set_position_hint(
      /* in     */ const world_coordinate_type pos
      );


// wake driver now and have it update route
void wake_driver(void);

// 'cruise speed' is the default speed for a vessel. this is used when
//    actual speed information is unavailable, as this at least provides
//    a guess as to what the vessel is doing

// 'set' is called during initialization. it can also be called during
//    runtime, for example to provide a running average of recent performance
void set_default_cruise_speed_kts(double kts);


// set vessel position info speed, heading and motion/sec
void update_vessel_heading(
      /* in out */       vessel_position_info_type *ves,
      /* in     */ const true_heading_type true_heading,
      /* in     */ const meters_per_second_type speed
      );

// sets vessel position plus updates heading
void set_vessel_position(
      /* in out */       vessel_position_info_type *ves,
      /* in out */       route_info_type *route,
      /* in     */ const world_coordinate_type pos,
      /* in     */ const meters_per_second_type speed,
      /* in     */ const true_heading_type true_heading,
      /* in     */ const double t
      );

////////////////////////////////////////////////////////////////////////
// functions shared for benefit of integration testing

// puts subscore in lowest if it's lower than values there
// lowest[0] should be the lowest of all values and lowest[1] is the 
//    2nd lowest. if lowest[0] is repeated, repeats is incremented
void update_subscore(
      /* in out */       double lowest[2], 
      /* in out */       int32_t *repeats,
      /* in     */ const double subscore
      );

// combine lowest scores into single score value
// provide weight of lowest value with 7 + 3 times number repeats, 
//    and weight of 1 for 2nd lowest
double combine_subscores(
      /* in     */ const double lowest[2], 
      /* in     */ const int32_t repeats
      );

// provides score for how similar selected route is relative to desired
//    'direction' vector
// considers both short and long directions (ie, 'direction' and 'direction'
//    once removed)
// direction score on [ROUTE_SCORE_RECIPROCAL_HEADING,1]
double calc_direction_agreement(
      /* in     */ const uint8_t radial,
      /* in     */ const bam16_type course
      );

// establish course through local traffic and obstacles
// follows path map flow as closely as practical, with constraint that
//    course changes are assumed to be infrequent
// calling function should check to make sure route isn't in error 
// world map and path map must already be loaded
// updates route map and route_info
// NOTE calling function should make sure route_info state isn't
//    foul, such as if PATH_LOCAL_MINIMUM is set, which means beacon
//    should be removed and path redrawn
void plot_route(
      /* in     */ const path_map_type *path_map,
      /* in out */       route_map_type *route_map,
      /* in out */       associator_output_type *ass_out,
      /* in out */       route_info_type *route_info,
      /* in out */       vessel_position_info_type *vessel_info,
      /* in     */ const double t_sec
      );

// decide whether or not to change course
void decide_course_change(
      /* in     */ const double t_sec,
      /* in     */ const double last_course_request_sec,
      /* in out */       route_info_type *route_info,
      /* in     */ const vessel_position_info_type *vessel_info
      );

#endif   // DRIVER_H

