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
#if !defined(ROUTE_CONTROL_H)
#define ROUTE_CONTROL_H
#include "pin_types.h"

union path_offset {
   struct {
      int16_t dx;
      int16_t dy;
   };
   uint32_t all;
};
typedef union path_offset path_offset_type;


//// data published by routing's driver
//// this is in external because it's also provided over socket interface
//
//// when route control info is received a reply should be sent
//struct route_request_reply {
//   // TODO include information about present heading and state of
//   //    steering system
//   uint64_t placeholder;
//};
//typedef struct route_request_reply route_request_reply_type;

//// deprecate these
//#define x_ROUTE_INFO_FLAG_HEADING_CHANGE    1
//#define x_ROUTE_INFO_FLAG_SPEED_CHANGE      2
//#define x_ROUTE_INFO_FLAG_REACHED_DESTINATION  4

// normal flag indicates expected deviations from course or speed.
//    unset value for course or speed means the same (for now at least)
#define ROUTE_INFO_COURSE_NORMAL           0x01  // also 0
// course change suggested
#define ROUTE_INFO_COURSE_SUGGEST_CHANGE   0x02
// course change requested for collision prevention
#define ROUTE_INFO_COURSE_MAKE_CHANGE      0x04
//// course change urgently requested for collision avoidance
//#define ROUTE_INFO_COURSE_URGENT_CHANGE    0x08

#define ROUTE_INFO_COURSE_CHANGE_MASK     \
      (ROUTE_INFO_COURSE_SUGGEST_CHANGE |    \
      ROUTE_INFO_COURSE_MAKE_CHANGE)


// TODO evaluate and implement speed control
// slow down, as able
#define ROUTE_INFO_SPEED_NORMAL              0x01
// stop, as much as able
#define ROUTE_INFO_SPEED_SUGGEST_CHANGE      0x02
#define ROUTE_INFO_SPEED_MAKE_CHANGE         0x04
// stop, as much as able, as we're in a potentially hazardous situation.
//    full-stop if self-driving, and 'take over'
//    if a human crew is available
// TODO when self-driving, this should be over-ridden if sea
//    conditions do not permit a safe stop
#define ROUTE_INFO_SPEED_FULL_STOP           0x80


// charting and tracking data are unavailable so we're running blind
// for now this is synonymous with full-stop, but at some point there
//    should be a fall-back system in place that takes over routing
#define ROUTE_INFO_STATE_RUNNING_BLIND          0x10u

// set if terrain has been checked
#define ROUTE_INFO_STATE_CHECK_TERRAIN             0x01u
// set if traffic has been evaluated
#define ROUTE_INFO_STATE_CHECK_TRAFFIC             0x02u

// TODO add logic to support what happens when destination reached
#define ROUTE_INFO_STATE_REACHED_DESTINATION    0x20u

// false destination -- path must have lead to beacon that was
//    a local minimum
#define ROUTE_INFO_STATE_PATH_LOCAL_MINIMUM     0x40u


#define ROUTE_INFO_STATE_CHECK_MASK    \
      (ROUTE_INFO_STATE_CHECK_TERRAIN | ROUTE_INFO_STATE_CHECK_TRAFFIC)

/////////////////
// persistent flags
// unlike most route flags, the persistent ones are set until explicitly
//    cleared
// set when best course, as determined by path map, is prevented from
//    being taken due and obstruction or target
#define ROUTE_INFO_DIVERT               0x00000001u
// set when desired path is clear of obstacles
#define ROUTE_INFO_PATH_CLEAR           0x00000002u

// set when position data is recent/accurate enough to use map
#define ROUTE_INFO_HAVE_POSITION        0x00000100u

// when have position and destination, route map can be created
#define ROUTE_INFO_HAVE_DESTINATION     0x00000200u

#define ROUTE_INFO_HAVE_POS_DEST_MASK        \
      (ROUTE_INFO_HAVE_POSITION | ROUTE_INFO_HAVE_DESTINATION)

// this is set when vessel is vessel is expected to be running blind
//    because the computer recently started and it's OK to be running
//    blind (ie, it's expected)
#define ROUTE_INFO_STARTING_UP_BLIND    0x00000800u

#define ROUTE_INFO_AUTOPILOT_ACTIVE     0x00010000u
#define ROUTE_INFO_AUTOPILOT_ERROR      0x00020000u

#define ROUTE_INFO_DIVERT_RECOVER_MASK    \
      (ROUTE_INFO_DIVERT | ROUTE_INFO_PATH_CLEAR)



// route planning information, plus suggested changes to route
// TODO 'route_info' used by gps.h -- find new name for this struct. for
//    now, rename it locally but keep typedef the same
//struct route_info {
struct route_control {
   /////////////////////////////////////////////////////////////////////
   // speed and heading
   // suggested heading
   true_heading_type sug_heading;
   double sug_heading_score;
   declination_type declination;
   // direction of turn. positive is right (to higher degree), negative
   //    is left. for now, only direction of turn is used (ie, + or -).
   //    this should be expanded to include rate of turn (to match name)
   degree_per_second_type turn_rate;
   // most recent time course was explicitly changed
   double course_changed_sec;
   // most recently set course. store this to calculate delta between
   //    newly suggested heading and previously decided course
   // TODO FIXME need safety check in case heading too far off course
   true_heading_type autopilot_course;
   double autopilot_course_score;
   /////////////////////////////
   // actual heading
   true_heading_type measured_heading;
   double measured_heading_score;
//   /////////////////////////////////////////////
//   // preferred and suggested speed
   meters_per_second_type default_speed;
//   meters_per_second_type sug_speed;
   meters_per_second_type present_speed;  // as measured by GPS or from water
   /////////////////////////////////////////////////////////////////////
   // route planning
   // approximate heading indicated by path map
   bam16_type true_path_heading;    // true, not magnetic
   // path offsets are 'direction' in path node. note that +y offset aligns
   //    with grid indices, NOT latitude
   // path start point
   world_coordinate_type start_position;
   // last known position, or at least a recent one. this is what's used
   //    to fall back to in event that position data is lost, as we
   //    still need some sort of routing info
   // this will usually have the same data as vessel.position, but the
   //    latter can be updated to projected position. last-known is not
   //    changed after being set, unless other solid position data comes in
   world_coordinate_type last_known_position;
   // destination
   world_coordinate_type destination;
   meter_type destination_radius;
   // temporary flags -- these are refreshed each round
   union {
      struct {
         uint8_t flags_course;
         uint8_t flags_speed;
         uint8_t flags_state;
         uint8_t flags_unused;
      };
      uint32_t flags_all;
   };
   // 'persistent' flags -- once set they persist until explicitly cleared
   uint32_t flags2_persistent;
};
typedef struct route_control route_info_type;

#endif   // ROUTE_CONTROL_H
