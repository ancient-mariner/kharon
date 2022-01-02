#if !defined(MAPPING_H)
#define MAPPING_H
#include "pin_types.h"
#include "external/route_control.h"
#include "beacon.h"

// which way is up?
// y motion is along lines of latitude, so down (south) is negative.
//    note that this is reverse of image coordinate system and of
//    map coordinates (where origin is where date line meets north pole)

// magnetic vs true heading
// optical system operates on magnetic heading as true requires position
//    data (which may not be available)
// vessel and target heading and bearings are magnetic. course radials
//    are true when examining possible courses, to align them with
//    map representation, and the suggested routes are converted back
//    to magnetic once selected
// mapping system is based on true north
//
// course plotting must reconcile difference
// TODO consider overriding magnetic values when close to poles,
//    using either GPS data or vessel-centric value (the latter not
//    useful for navigation per se but at least it preserves collision
//    avoidance)

struct target_record_export;
typedef struct target_record_export target_record_export_type;

#define MAPPER_LOG_LEVEL     LOG_LEVEL_DEFAULT

////////////////////////////////////////////////////////////////////////
// map of outside world

// per wikipedia
#define METERS_PER_DEG_LAT          (40007863 / 360.0) 
#define METERS_PER_DEG_LON_EQUATOR  (40075017 / 360.0) 
// adapt degs to meters longitude by multiplying by cos(lat)

#define NM_PER_DEG_LON_EQUATOR  (21638.77791 / 360.0) 
#define NM_LON_EQUATOR           21638.7779

#define DEGS_LAT_PER_METER       (1.0 / METERS_PER_DEG_LAT)

#define DEG_LAT_TO_METER            (40007863.0 / 360.0) 
#define DEG_LON_EQUATOR_TO_METER    (40075017.0 / 360.0) 

#define METER_TO_DEG_LAT            (360.0 / 40007863.0)

#define NM_TO_METERS                (1852.0)
#define METERS_TO_NM                (1.0 / 1852.0)

#define DEG_TO_NM_LAT            (60.0)

////////////////////////////////////////////////
// width and height, in nodes, of 5-second map. note that height
//    of map nodes are 5 arc-seconds. width is in nautical mile 
//    equivalent of 5 arc-seconds latitude (ie, 1/12nm)
#define WORLD_MAP_5SEC_WIDTH     720
#define WORLD_MAP_5SEC_HEIGHT    720

// path penalty is how high of a weight is assigned to pass through
//    a grid square. normal cost is 1.0. path takes route of least
//    cost (ie, least weight)
// path penalty is used initially when building plot of all viable paths
//    to destination
#define PATH_ADJACENT_NON_PASSABLE_PENALTY_BASE    25.0
#define PATH_ADJACENT_NON_PASSABLE_PENALTY_INC      5.0
#define PATH_SEMI_ADJACENT2_NON_PASSABLE_PENALTY_INC    0.3

#define PATH_BELOW_MIN_DEPTH_PENALTY_PER_METER     20.0

// terrain penalties increase the cost of plotting a course through
//    each grid square. initially a path is projected (all routes to
//    destination) and that is used as a base for selecting immediate
//    heading. actual heading is determined by considering the relative
//    score (cost) of traversing each radial outward from a given
//    position, taking into account suggested path, terrain and
//    obstacles (targets). the radial with the highest score is the
//    preferred route at a given point in time. this preferred route
//    will be selected if it is sufficiently higher score than the
//    present heading and/or if a certain amount of time has elapsed
//    since the previous course change


// water considered non-passable if at or below reported minimum depth
// passage penalty is reduced as water deepens up to pref depth
// abs_min is to allow passage/escape from very shallow water that
//    vessel may find itself in (eg, if autotracking is activated
//    in non-traversable water). terrain score is based on depth. if
//    depth is below 'min' then max terrain score is 0.001, and that
//    goes to zero at 'abs min'
#define ABS_MIN_TRAVERSABLE_DEPTH_METERS    3.0
#define MIN_TRAVERSABLE_DEPTH_METERS    6.0
#define PREF_TRAVERSABLE_DEPTH_METERS   10.0


// below penalties are multiplicative

// adjacent non-passable should have high penalty, as rocks in 
//    non-passable area may be uncomfortably close to boundary of 
//    adjacent cell. if cell considered passable it should score above
//    reciprocal heading score, otherwise turning around will be
//    preferred to crossing cell
#define TERRAIN_PENALTY_ADJACENT_NON_PASSABLE   0.2

// cell that's next to land-adjacent cell. not dangerous but something
//    perhaps better avoided 
// this now applies to next to land-adjacent, and next-to-next
#define TERRAIN_PENALTY_SEMI_ADJACENT_NON_PASSABLE   0.8

// provide reciprocal as an out, but an unattractive one
#define ROUTE_SCORE_RECIPROCAL_HEADING    0.10

// how long tracking data can be lost before it's considered to old
//    to use. when new tracking data not available the most recent
//    data is used, based on last known position and trajectory.
// if the timeout windows is longer than what's required to reset
//    the optical tracking stream then course interruption shouldn't
//    occur if a target being avoided is now ignored. if timeout window
//    too long then vessel traveling w/ stale data
// 15 sec should be fine assuming a relatively slow speed, and a brief
//    camera interruption
// TODO perhaps lower speed when tracking lost?
#define MAX_TARGET_TRACKING_AGE_SEC    15.0

// limit estimated approach speed as this can be artifact prone, esp.
//    at large distances (10mps is ~19.5 knots)
// TODO implement smarter algorithm
#define AXIAL_SPEED_LIMIT_MPS    10.0


// WMM declares region around magnetic pole a 'caution' zone when 
//    inclination is >84 degrees, and an 'unstable' zone
//    when inclination >88 degrees. these areas should be avoided
//    unless there's a reliable non-magnetic sensor to determine north
// TODO add logic for magnetic fallback, and/or a system to avoid
//    routing through these zones (e.g., prohibit beacons being stationed
//    there and have a high transition cost between beacons on opposite
//    sides)
#define MAGENTIC_POLE_CAUTION_INCLINATION    84.0
#define MAGENTIC_POLE_UNSTABLE_INCLINATION   88.0


// takes signed double representing distance from horizon and 
//    approximates range. this should not be used for angles
//    above 25 degrees
// use planar approximation of surface. for camera height of >10m and
//    range <1km, the error should be <1% (Gordon, 2001). Error increases
//    w/ increasing range, but implementing a wedge cutoff below the
//    horizon at 0.2 deg limits range estimate to ~3km w/ 10m high cam,
//    and ranging will be poor w/o high resolution cams at this range
//    anyway (and marginal at best even with) so estimation error
//    should be tolerable
// takes degrees below horizon and approxmiates range
// NOTE that below-horizon is a negative degree value
#define APPROX_RANGE_DEG(x)     \
            (-VIRTUAL_CAMERA_HEIGHT_MET /  TAN_APPROX_DEG(x))

// takes signed int32 (eg, bottom.alt_sangle32)
#define APPROX_RANGE32(x)     \
         (APPROX_RANGE_DEG((double) (x) * BAM32_TO_DEG))

// inverse of approx_range -- converts from est. range to degrees below horiz
#define APPROX_WEDGE_DEG(x)     \
            (-ATAN_APPROX_DEG(VIRTUAL_CAMERA_HEIGHT_MET / (x)))

// when a map is generated, vessel is placed this many nautical
//    miles from map center, with center in the direction vessel
//    should take to reach destination
#define VESSEL_OFFSET_FROM_MAP_CENTER_NM     10.0

// if beacon is w/in this distance of vessel, don't include it in map
// this should be larger than 
#define VESSEL_BEACON_INHIBITION_RING_NM     4.0


// path_map in this context is equivalent to a level-3 map in the
//    external 'mapping' module. the 8-bit level-3 depth code is decoded
//    for depth_meters, so depth will be very jumpy esp. in deep water

union map_feature_node {
   struct {
      // >0 for water
      // <0 for land
      int16_t  depth_meters;
      // number of nearby nodes of different types. this is used for
      //    route scoring
      // node itself may be impassable -- that's separately 
      //    determined by depth
      union {
         struct {
            uint8_t land_cnt;
            uint8_t near_cnt;
         };
         uint16_t cnt_all;
      };
   };
   uint32_t features_all;
};
typedef union map_feature_node map_feature_node_type;

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
// mapping path

union path_map_index {
   uint32_t idx;
   int32_t val;      // -1 for unset
};
typedef union path_map_index path_map_index_type;

#define PATH_NODE_FLAG_PROCESSED   1
#define PATH_NODE_FLAG_NO_ACCESS   2

// each node has a path vector (beyond adjacent parent) indicating its
//    approximate trajectory
#define NUM_ANCESTORS_FOR_DIRECTION    5

struct path_map_node {
   // node that's one-closer to destination
   path_map_index_type    parent_id;
   // position of node in map
   image_coordinate_type pos;
   // follow path along several nodes to get approx direction to steer
   // vector of path, e.g., offset to node 5 parents removed, such
   //    as g-g-g-grandparent
   // approximate course to steer in this path node to reach destination
   bam16_type true_course;
   // actual course to follow -- true course may be overriden
   bam16_type active_course;  
   //
   float weight;    // effective 'distance' of this node from destination
   float passage_penalty;   // weight penalty to transition this node
   uint32_t flags;
};
typedef struct path_map_node path_map_node_type;


struct map_beacon_reference {
   akn_position_type coords;  // position in world
   uint32_t index;   // master index
   image_coordinate_type pos_in_map;      // position w/in map
   //
   float center_dist_met;      // distance from map center
   // this is path weight to destination. it's pulled from list of
   //    beacon path weights to destination (ie, from where path
   //    traced at beacon level)
   float path_weight;
};
typedef struct map_beacon_reference map_beacon_reference_type;


struct path_map {
   // TODO reformat after path and world map merge
   /////////////////////////////////////////////
   // migrated from world_map
   // number of nodes (pixels) in map
   image_size_type size;
   // center longitude on (0,360]
   world_coordinate_type center;
   // width and height of map
   meter_type map_width;     // at map vertical center
   meter_type map_height;
   //
   meter_type node_width;
   meter_type node_height;
   //
   path_map_node_type   *nodes;
   map_feature_node_type   *feature_nodes;
   /////////////////////////////////////////////
   declination_type declination;
   // inclination necessary to determine stability of magnetic signal
   //    in this area. when too close to poles compass cannot be relied
   //    upon so other sensors (e.g., GPS) must be used. this is
   //    generally a good-to-avoid region
   declination_type inclination;
   /////////////////////////////////////////////
   // beacon management
   path_map_index_type *stack;
   uint32_t write_idx;
   uint32_t read_idx;
   // beacon info
   uint32_t num_beacons;
   map_beacon_reference_type beacon_ref[MAX_PATH_MAP_BEACONS];
   /////////////////////////////////////////////////////////////////////
   // positions
   // maintaining state for determining when update is necessary
   // path is updated when vessel has moved more than X nm from 
   //    point when map was last updated
   image_coordinate_type   vessel_start_pix;
   // destination may be outside of map bounds.
   akn_position_type    destination;   // no index and zero weight
   image_coordinate_type    dest_pix;  // display coord in map
};
typedef struct path_map path_map_type;

// mapping path
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
// local routing

// used for collision avoidance and route planning
// algorithm seeks to find course that vessel can take at present
//    speed that avoids obstacles
// vessel motion for route is designed to be relatively stable so it
//    is at least semi-predictable for other vessels. home vessel
//    should not change direction in less than a minute after previous
//    change unless it's necessary

// route map is designed as a 255x255 array of 20 meter tall/wide grids,
//    representing a circle of radius 127. center of map is vessel 
//    location. a ring is traced out from center based on present
//    speed, representing the position of vessel for all possible
//    headings it can take. turn is initially considered instanteous.
//    in future version, need to take into account that ring edge
//    in opposite direction of travel must be moved in toward center.
// position of nearby targets and land are overlayed 
//    on map at their forecast position every X(=5) seconds. if target
//    overlaps ring then that ring location is flagged as collision
//    risk. note that target (and land) positions are relative. land
//    and depth only need be assessed on first iteration and if there's
//    a collision risk then that radial can be eliminated, or eliminated
//    after time T (duration to reach that point)
// for all collision risks, the radial of the risk is eliminated from
//    being a possible course to take
// algorithm only checks for home vessel being on linear path from 
//    start point. multi-path gets computationally much harder using
//    the strategy here. a simple multi-path approach is select a
//    handful of points, say 1-minute away from center, 10-degrees
//    apart, and re-calculating from those points on all radials where
//    collision risk on that radial is >1-minute
//
// 20 meter grid size is granularity that CPA ring can be set at, and
//    given rectangular representation of circle, the CPA ring will be
//    only a rough estimate
// 20-meters * 127 = 2500 meters plan
// larger vessels can use same approach but with larger grids (eg, 50m
//    for 6km plan)
// longitude measurement based on vessel longitude. for smallish grids,
//    eg, <600 miles tall) there shouldn't be significant spherical
//    distortion, except nearer poles (eg, N60 degrees is ~15% distortion
//    for 600nm tall grid)
//
// radial analysis
// during radial analysis, check that path map continues on this
//    trajectory, or at least that it doesn't loop back on itself.
//    better -- use path node weights when determining which radial
//    to take (if changing course)
// if land blocks straight radial but path says course exists, handle as
//    special case and follow path
//
//
/**
Rules for course change  TODO verify in code -- it may have evolved
1  Don't change course closer than X minute(s) to previous change, unless
      this can't be avoided
2  If optimal course is w/in Y degrees of present direction, stay
      on same course even if rule 1 is satisfied
3  If home is give-way vessel, make course change if collision risk
      seems likely w/in Z minutes. If home is stand-on vessel, make
      course change if collisions seems likely in Z/2 minutes
where
   X = 1 min (minimum interval between course changes)
   Y = 10 degrees (window considered close-enough to being on course)
   Z = 2 min (time to predicted collision that course change occurs by)
**/

#define ROUTE_MAP_RADIUS   127
#define ROUTE_MAP_WIDTH    (2 * ROUTE_MAP_RADIUS + 1)
#define ROUTE_MAP_HEIGHT   ROUTE_MAP_WIDTH
#define ROUTE_MAP_NUM_NODES   (ROUTE_MAP_WIDTH * ROUTE_MAP_HEIGHT)

#define ROUTE_MAP_NODE_WIDTH_METERS    20.0


// TODO base intervals depend on camera height from water. when low (eg, 3m)
//    then long intervals are mostly useless, as range detection beyond
//    a few hundred meters isn't very accurate
enum { 
   VIABILITY_INTERVAL_0,      // e.g., <15 sec
   VIABILITY_INTERVAL_1,      // e.g., >15 and <30 sec
   VIABILITY_INTERVAL_2,      // e.g., >30 and <60 sec
   VIABILITY_INTERVAL_3,      // e.g., >60 and <120 sec
   VIABILITY_INTERVAL_4,      // e.g., >120 sec
   NUM_VIABILITY_INTERVALS
};
// for interval bounds, see driver.h


#define NUM_ROUTE_RADIALS     256


// traversability of point (radial or node) across different time intervals
struct route_node_viability {
   // score for avoiding terrain
   double terrain_score[NUM_VIABILITY_INTERVALS];
//   // score to avoiding stand-on vessels (ie, avoiding vessels that
//   ///   have priority)
//   double stand_on_score[NUM_VIABILITY_INTERVALS];
//   // score to avoid give-way vessels (ie, not hit vessels that are
//   //    supposed to give way, but might not)
//   double give_way_score[NUM_VIABILITY_INTERVALS];
};
typedef struct route_node_viability route_node_viability_type;


// traversability of point (radial or node) across different time intervals
// radials are based on TRUE NORTH
struct radial_viability {
   ////////////////////////////////////////////////////////////////////
   // intermediate data
   // score for avoiding terrain
   double terrain_score[NUM_VIABILITY_INTERVALS];
   // score to avoiding stand-on vessels (ie, avoiding vessels that
   ///   have priority)
   double stand_on_score[NUM_VIABILITY_INTERVALS];
   // score to avoid give-way vessels (ie, not hit vessels that are
   //    supposed to give way, but might not)
   double give_way_score[NUM_VIABILITY_INTERVALS];
//   // evasion is used in situations where there's an acute problem
//   //    that must be addressed. this is absolute and is not
//   //    broken down into intervals
//   double evasion_score;
   /////////////////////////////////////////////////////////////////////
   // bams used for calculation
   // how closely this radial aligns with desired course
   double direction_score;
   // arc score is a measure of the arc width of passage along each radial
   // low score indicates narrow or shallow passage. for open passage, 
   //    arc score should equal regular score (arc width hard-coded in
   //    share.c:calculate_arc_scores()
   double terrain_arc[NUM_VIABILITY_INTERVALS];
   double stand_on_arc[NUM_VIABILITY_INTERVALS];
   double give_way_arc[NUM_VIABILITY_INTERVALS];
   // evasion_score is applied to arc values
   /////////////////////////////////////////////////////////////////////
   double net_score;
};
typedef struct radial_viability radial_viability_type;


struct route_map_node {
   /////////////////////////////////////////////////////////////////////
   // more or less static values
   signed_coordinate_type pos;   // position in route map
   // corresponding grid position in world and path maps
   image_coordinate_type world_pos;   
   // radial from map center of node center
   bam16_type  radial;
   // make note of left and right radial edges of node (necessary
   //    for making sure entire arc that node represents is used
   //    for marking collision risk bearing
   bam16_type  radial_left_edge;
   bam16_type  radial_right_edge;
   // distance from center, in map radians
   radian_type distance;
   /////////////////////////////////////////////////////////////////////
   // transient values
   // index of corresponding node in path and world maps
   uint32_t world_node_idx;
   //
   double terrain_score;
   // time required to reach and exit this node
   double arrival_dt_sec;
   double exit_dt_sec;
};
typedef struct route_map_node route_map_node_type;


// analysis of possible routes that avoid collisions
// route map uses high resolution grid (e.g., 20x20m)
// now that collision avoidance doesn't rely on route map, there's
//    limited reason to keep using it, as path and world maps are the
//    only maps used for navigation
// TODO reduce map resolution to be closer to world map; consider 
//    eliminating to reduce code complexity
struct route_map {
   image_size_type   size;
   // nodes should be square, so width=height
   meter_type node_width;
   //
   route_map_node_type *nodes;
   // viability of route over different intervals
   // 0 is true north, 64 east, etc
   radial_viability_type  radials[256];
};
typedef struct route_map route_map_type;


////////////////////////////////////////////////////////////////////////

// represents position, heading and speed of vessel as well as 
//    estimated future (or past) position based on heading and speed
// represents vessel motion plus target motion when analyzing paths to
//    avoid collision risks
struct vessel_position_info {
   /////////////////////////////////////////////
   // most recently measured (estimated) position
   world_coordinate_type position;
   double estimated_pos_time; // time that position corresponds to
   double confirmed_pos_time; // last time position was confirmed (eg, by gps)
   // approximate accuracy radius along least accurate dimension
   // radar has better range while optical has better radial
   // for GPS position accuracy is negligably small, but for other forms
   //    (eg, DR, loran, stellar) the error radius will be notably larger
   // this is not used presently -- it's a placeholder for future
   meter_type  position_accuracy;
   /////////////////////////////////////////////
   // estimated heading (absolute)
   true_heading_type true_heading;   
   // motion along NS/EW (x/y) directions -- this is based on TRUE heading
   // degree representation is corrected for latitude
   degree_offset_type motion_per_second;
   // xy motion is based on true north
   meter_2d_motion_type xy_motion;
   // water speed if it's available, and the best guess if not
   meters_per_second_type speed;
   // TARGET ONLY: bearing from home vessel
   true_heading_type true_bearing; // was 'mag_bearing'
};
typedef struct vessel_position_info vessel_position_info_type;


// avoidance radius
// old definition. keep this around and consider reviving parts of it.
//    a hard avoidance ring is useful but it can limit flexibility
//    in a high traffic area ('panics' were common w/ the hard radius).
//    new approach is much more fluid but lacks a hard radius
//// radius generates a ring around all vessels that should not be entered
//// there are two parts to this ring, a 'hard' do-not-enter and a 'soft'
////    zone. the hard zone is 1/2 the total radius. the hardness linearly
////    reduces to zero at the outer ring border (the outside of the soft
////    zone)
//// the avoidance zone of both vessel and target are compared, and the
////    larger of those is used. this results in a larger avoidance ring
////    around large objects (eg, container ship) and a smaller around
////    flotsam or buoys (which will default to the vessels avoidance
////    radius)
// 
// avoidance is now a repulsive force, the magnitude of which is 
//    sqrt(distance/ring)/2, w/ low number repulsive and high being OK.
//    repulsive force is reduced by confidence level, so a low-confidence
//    target will have its avoidance score moved most of the way to
//    1.0. using this elastic repulsion means that the avoidance
//    radius needs to be relatively large to achieve a reasonable avoidance
//    distance
// avoidance score is 0.5 with full confidence when CPA = avoidance radius

// avoidance radius about an object is a multiple of that objects approximate
//    size. this is that multiple
// target avoidance applied to 4x ring size, so for low camera this could
//    be almost entire area where speed and direction is estimable
#define AVOIDANCE_RADIUS_SIZE_MULTIPLE    15.0

// minimum size of avoidance radius
#define AVOIDANCE_RADIUS_MIN_SIZE_METERS    20.0

// max size of avoidance radius, no mater target size
// high multiple means that most objects will be have max ring
#define AVOIDANCE_RAD_MAX_METERS    50.0

////////////////////////////////////////////////

// offset between two adjacent pixels 
// no pixels set means there's no offset. 0x80 is north, 0x40 is NE, etc
struct pixel_offset_bitfield {
   uint8_t mask;
};
typedef struct pixel_offset_bitfield pixel_offset_bitfield_type;


////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
// API

struct associator_output;
typedef struct associator_output associator_output_type;

struct optical_target_summary;
typedef struct optical_target_summary optical_target_summary_type;
struct target_motion;
typedef struct target_motion target_motion_type;


// load 5-arc-second map from /pinet/mapping/master
void load_world_5sec_map(
      /* in     */ const world_coordinate_type map_center,
      /*    out */       path_map_type *path_map
      );


////////////////////////////////////////////////////////////////////////

world_map_type * create_world_map(
      /* in     */ const image_size_type size
      );

path_map_type * create_path_map(
      /* in     */ const image_size_type size
      );

route_map_type * create_route_map(
      /* in     */ const meter_type node_width
      );


// override active course for all nodes in path map
// active course is true (not magnetic)
void override_active_course_all(
      /*    out */       path_map_type *path_map,
      /* in     */ const bam16_type new_active
      );

// sets active course in all path nodes to their normal path values (ie,
//    removes override)
void set_default_active_course(
      /*    out */       path_map_type *path_map
      );

////////////////////////////////////////////////////////////////////////
// from zoutput.c

// create file showing direction of path to destination
void write_direction_map(
      /* in     */ const path_map_type *path_map,
      /* in     */ const char *fname
      );

// create file showing direction of active path that's stored. this
//    may be different than direction map if value overridden
void write_active_direction_map(
      /* in     */ const path_map_type *path_map,
      /* in     */ const char *fname
      );

// create file showing weight of path (and land) across map area
void write_path_map(
      /* in     */ const path_map_type *path_map,
      /* in     */ const char *fname
      );

// create topo image of map area
// if path_map is not null, and if there are beacons, those are indicated
//    by a red dot
void write_depth_map(
      /* in     */ const path_map_type *path_map,
      /* in     */ const char *fname
      );

////////////////////////////////////////////////////////////////////////

// sets destination. traces high-level route to destination, and returns
//    local path map with vectors to follow to get there
// returns 0 on success and -1 on failure
int trace_route_initial(
      /* in out */       path_map_type *path_map,
      /* in     */ const world_coordinate_type dest,
      /* in     */ const world_coordinate_type vessel_pos
      );

// draw paths to already-established destination and beacons in map
void trace_route_simple(
      /*    out */       path_map_type *path_map,
      /* in     */ const world_coordinate_type vessel_pos
      );

////////////////////////////////////////////////////////////////////////

// get and set full path to map folder
void set_world_map_folder_name(
      /* in     */ const char *name
      );
const char* get_world_map_folder_name(void);

// load beacon data from map folder
// returns 0 on success and -1 on failure
int init_beacon_list(void);


// returns coordiantes that are offset by 'range' at 'heading' from 'source'
world_coordinate_type calc_offset_position(
      /* in     */ const world_coordinate_type source,
      /* in     */ const degree_type heading,
      /* in     */ const meter_type range
      );

// returns pixel location in map for specified world coordinate
// if coordinate is outside of map area then pixel's x and/or y value will
//    be 65535 (ie, 0xffff)
image_coordinate_type get_pix_position_in_map(
      /* in     */ const path_map_type *path_map,
      /* in     */ const world_coordinate_type pos
      );

// generate map based on existing map, with map center being 15nm in
//    front of vessel with vessel following existing path map's route
//    to get to destination
void rebuild_map_by_vessel_offset(
      /* in out */       path_map_type *path_map,
      /* in     */ const image_coordinate_type vessel_pix,
      /* in     */ const world_coordinate_type vessel_pos
   );

//// calculate beacon's display locations
//// beacon_num is number of beacon in path map, eg, on [0,12)
//void calculate_beacon_map_position(
//      /* in out */       path_map_type *path_map,
//      /* in     */ const uint32_t beacon_num    // beacon # in path map
//      );

//////////

// calculates meter offset between two world coordinates
// positive dx is rightward
// positive dy is upward
// for dx, latitude correction is performed (based on dest's latitude)
void calc_meter_offset(
      /* in     */ const world_coordinate_type src,
      /* in     */ const world_coordinate_type dest,
      /*    out */       meter_type *dx,
      /*    out */       meter_type *dy,
      /* in     */ const char *label
      );


void init_route_info(
      /*    out */       route_info_type *info,
      /* in     */ const world_coordinate_type start_point,
      /* in     */ const world_coordinate_type destination,
      /* in     */ const meter_type destination_radius
      );


void reset_path_map(
      /* in out */       path_map_type *path_map
      );

#endif   // MAPPING_H
