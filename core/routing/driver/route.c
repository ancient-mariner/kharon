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
#if !defined(ROUTE_MAP_C)
#define ROUTE_MAP_C
#include "lin_alg.h"
#include "timekeeper.h"
#include "logger.h"

// at 5 knots (~2.6mps), distance traveled in 10 seconds is ~26m (~85ft)
// 12 seconds @ 5 knots is approx 100ft
static const double viability_interval_end_[NUM_VIABILITY_INTERVALS] = {
   VIABILITY_INTERVAL_END_0,
   VIABILITY_INTERVAL_END_1,
   VIABILITY_INTERVAL_END_2,
   VIABILITY_INTERVAL_END_3,
   VIABILITY_INTERVAL_END_4
};

////////////////////////////////////////////////////////////////////////
// land avoidance

// assess path viability as function of terrain

// push viability score of each node to radials that node covers
static void push_node_viabilities_to_radials(
      /* in out */       route_map_type *route_map
      )
{
   radial_viability_type *radials = route_map->radials;
   //////////
   // update radials w/ route node values
   image_size_type size = route_map->size;
   uint32_t idx = 0;
//FILE *fp = fopen("/tmp/path.pnm", "w");
//fprintf(fp, "P6\n%d %d\n255\n", size.cols, size.rows);
   for (uint32_t y=0; y<size.y; y++) {
      for (uint32_t x=0; x<size.x; x++) {
         route_map_node_type *node = &route_map->nodes[idx++];
         // left scoresec
         uint32_t start = node->radial_left_edge.angle16;
         // delta between left and right
         uint32_t delta = (uint16_t) (node->radial_right_edge.angle16 -
               node->radial_left_edge.angle16);
         // right scoresec -- unwound in sense can be greater than uint16
         uint32_t end = (uint32_t) (start + delta);
         start >>= 8;
         end >>= 8;
//double vals[5];
//for (uint32_t ival=0; ival<NUM_VIABILITY_INTERVALS; ival++) {
//   vals[ival] = node->viability.terrain_score[ival];
//}
//double a = vals[0] < vals[1] ? vals[0] : vals[1];
//double b = vals[2] < vals[3] ? vals[2] : vals[3];
//double c = vals[4];
//uint8_t rgb[3];
//rgb[0] = (uint8_t) (a * 255.0);
//rgb[1] = (uint8_t) (b * 255.0);
//rgb[2] = (uint8_t) (c * 255.0);
//fwrite(rgb, 1, 3, fp);
         ///////////////////
         // push viability score for each radial at each time
         const double terrain_score = node->terrain_score;
         if (terrain_score < 1.0) {
            uint32_t start_ival = NUM_VIABILITY_INTERVALS;
            uint32_t end_ival = NUM_VIABILITY_INTERVALS - 1;
            // find arrival interval. this is first interval where
            //    arrival time is before interval end
            // get departure interval. this is the interval where
            //    interval_end is after departure time. this covers
            //    the case where start of interval was still w/in
            //    route node
            double arrival_sec = node->arrival_dt_sec;
            double exit_sec = node->exit_dt_sec;
            for (uint32_t ival=0; ival<NUM_VIABILITY_INTERVALS; ival++) {
               double ival_end_sec = viability_interval_end_[ival];
//printf("  %d ival_end %.3f  arrival %.3f   exit %.3f\n", ival, ival_end_sec, arrival_sec, exit_sec);
               if (arrival_sec < ival_end_sec) {
                  if (start_ival == NUM_VIABILITY_INTERVALS) {
                     start_ival = ival;
                  }
                  if (ival_end_sec > exit_sec) {
                     end_ival = ival;
                     break;
                  }
               }
            }
//if (start_ival < 4) {
//   printf("node %d,%d   score %.5f   ival %d,%d   arc %d,%d (%d)   arr %.3f  exit %.3f\n", x, y, terrain_score, start_ival, end_ival, start, end, delta, arrival_sec, exit_sec);
//}
            // push score to all radials that cross through this node
            //    over relevant intervals
            for (uint32_t bin=start; bin<=end; bin++) {
               uint32_t radial = bin & 255;
               for (uint32_t ival=start_ival; ival<=end_ival; ival++) {
                  if (terrain_score < radials[radial].terrain_score[ival]) {
                     radials[radial].terrain_score[ival] = terrain_score;
                  }
               }
            }
         }
      }
   }
//fclose(fp);
}


// sets terrain_score for this route node
static void update_terrain_viability(
      /* in     */       map_feature_node_type *feature_node,
      /* in     */       route_map_node_type *route_node
      )
{
   double score = 1.0f;
   // adjust score based on depth
   // score should be on (0,1]
   const double abs_min_depth = (double) ABS_MIN_TRAVERSABLE_DEPTH_METERS;
   const double min_depth = (double) MIN_TRAVERSABLE_DEPTH_METERS;
   const double pref_depth = (double) PREF_TRAVERSABLE_DEPTH_METERS;
   assert(min_depth < pref_depth);  // sanity check
   assert(abs_min_depth < min_depth);  // sanity check
   //
   const double depth = (double) feature_node->depth_meters;
   const double DEPTH_ABS_MIN_SCORE = 0.001;
   const double DEPTH_MIN_SCORE = 0.01;
   if (depth < pref_depth) {
      if (depth <= abs_min_depth) {
         score = 0.0001 +
               (DEPTH_ABS_MIN_SCORE - 0.0001) * (depth / abs_min_depth);
      } else if (depth <= min_depth) {
         score = DEPTH_ABS_MIN_SCORE +
               (DEPTH_MIN_SCORE - DEPTH_ABS_MIN_SCORE) *
               (depth - abs_min_depth) / (min_depth - abs_min_depth);
      } else {
         score = DEPTH_MIN_SCORE + (1.0 - DEPTH_MIN_SCORE) *
               (depth - min_depth) / (pref_depth - min_depth);
      }
   }
   // TODO adjacent should be considered non-passable. when grid resolution
   //    increased to under 5-seconds, set adjacent to zero
   // the problem is that a rock at the corner of a non-passable
   //    grid can be next to the corner of an adjacent, and it is
   //    foreseeable that leaving the adjacent grid at 45-deg angle
   //    could get too near rock, although depth _should_ filter
   //    out that possibility
   if (feature_node->land_cnt > 0) {
      score *= TERRAIN_PENALTY_ADJACENT_NON_PASSABLE;
   } else if (feature_node->near_cnt > 0) {
      score *= TERRAIN_PENALTY_SEMI_ADJACENT_NON_PASSABLE;
   }
   route_node->terrain_score = score;
//printf("%.5f\n", score);
}


// evaluate world map to determine areas to avoid
static void assess_terrain_risks(
      /* in     */ const path_map_type *path_map,
      /* in out */       route_map_type *route_map
      )
{
   image_size_type size = route_map->size;
   uint32_t route_idx = 0;
   for (uint32_t y=0; y<size.y; y++) {
      for (uint32_t x=0; x<size.x; x++) {
//printf("node %d,%d  ", x, y);
         route_map_node_type *node = &route_map->nodes[route_idx++];
         map_feature_node_type *feature_node =
               &path_map->feature_nodes[node->world_node_idx];
         update_terrain_viability(feature_node, node);
      }
   }
}

// land avoidance
////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////
// path scoring

// provides score for how similar selected route is relative to desired
//    'direction' vector
// considers both short and long directions (ie, 'direction' and 'direction'
//    once removed)
// direction score on [ROUTE_SCORE_RECIPROCAL_HEADING,1]
double calc_direction_agreement(
      /* in     */ const uint8_t radial,
      /* in     */ const bam16_type course
      )
{
//   // if long and short delta is small, give preference to long
//   // otherwise stick w/ short
//   // casting difference to 0-255 gives delta regardless of pole
//   // recasting on -128,127 provides signed delta. that can be stored in
//   //    regular int
//   int32_t ls_delta = (int8_t) ((uint8_t) (long_theta - short_theta));
//   uint8_t target_theta = short_theta;
//   if (abs(ls_delta) < 6) {
//      // long is close to short. assume that it's a better guess and
//      //    point to it instead
//      target_theta = long_theta;
//   } else if (abs(ls_delta) < 12) {
//      // long is still pretty close to short. aim for a midpoint between
//      //    the two
//      target_theta = (uint8_t) ((int32_t) target_theta + ls_delta/2);
//   }
   // approx goal:
   //    drop to 50% at 45 degrees from course (32 bams)
   //    drop to 25% at 90 degrees from course (64 bams)
   //    drop to 10% at 180 degrees from course (128 bams)
   // approx fn:
   //    1.0 - 0.9 * sqrt(delta/128)
   // ->        0 deg  1.00
   //          45 deg   .55
   //          90 deg   .36
   //         135 deg   .22
   //         180 deg   .10
   double delta = (double) abs((int8_t) ((course.angle16 >> 8) - radial));
   double score = 1.0 -
         (1.0 - ROUTE_SCORE_RECIPROCAL_HEADING) * sqrt(delta / 128.0);
   return score;
}


// using specified path node, set path vectors in route_info
static void set_path_direction_from_path_node(
      /* in     */ const path_map_node_type *path_root,
      /* in out */       route_info_type *route_info
      )
{
   // check to see if we're at destination. note that this can only occur
   //    if position and destination are set
   uint32_t mask = ROUTE_INFO_HAVE_POS_DEST_MASK;
   if (((route_info->flags2_persistent & mask) == mask) &&
         (path_root->parent_id.val < 0)) {
      // we have position info and we're at a local path minimum
      // this probably means that we're at destination, but a beacon
      //    can erroneously give this signal. check path node's weight.
      //    at destination, weight is zero
      if (path_root->weight > 1.0f) {
printf("  local minimum detected\n");
         route_info->flags_state |= ROUTE_INFO_STATE_PATH_LOCAL_MINIMUM;
      } else {
printf("  path node at destination\n");
         // this is the path destination node. mission complete
         route_info->flags_state |= ROUTE_INFO_STATE_REACHED_DESTINATION;
      }
      // no new path information. stick with previous value. if we
      //    leave destination cell then route will be updated
      return;
   }
   route_info->true_path_heading = path_root->active_course;
}


// extract desired route from path map
static void set_path_offset_by_last_known_position(
      /* in     */ const path_map_type *path_map,
      /* in out */       route_info_type *route_info
      )
{
   world_coordinate_type pos = route_info->last_known_position;
   meter_type dx, dy;
   calc_meter_offset(path_map->center, pos, &dx, &dy, __func__);
   // distance from map top and left
   double inset_left_meters = dx.meters +
         path_map->node_width.meters * (double) (path_map->size.x / 2);
   // +dy is up. invert it here as origin in map is in top left
   double inset_top_meters = -dy.meters +
         path_map->node_height.meters * (double) (path_map->size.y / 2);
   // get index for grid square in path map
   uint32_t idx_x = (uint32_t) ((int32_t)
         (inset_left_meters / (double) path_map->node_width.meters));
   uint32_t idx_y = (uint32_t) ((int32_t)
         (inset_top_meters / (double) path_map->node_height.meters));
   if ((idx_x < path_map->size.x) && (idx_y < path_map->size.y)) {
      //////////////
      // vessel position is w/in map. set offsets
      uint32_t world_node_idx = (uint32_t) (idx_x + idx_y * path_map->size.x);
      // code below leveraged from get_path_offsets()
      // get basic path to destination
      path_map_node_type *path_root = &path_map->nodes[world_node_idx];
      set_path_direction_from_path_node(path_root, route_info);
   }
}


// extract desired route from path map
static void get_path_offsets(
      /* in     */ const path_map_type *path_map,
      /* in out */       route_map_type *route_map,
      /* in out */       route_info_type *route_info
      )
{
   // root node is at center of route map (ie, vessel location)
   uint32_t root_idx = (uint32_t) (route_map->size.x/2 +
         route_map->size.x * (route_map->size.y/2));
//printf("route center idx: %d (from %d,%d)\n", root_idx, route_map->size.x, route_map->size.y);
   route_map_node_type *root = &route_map->nodes[root_idx];
//printf(" root route at %d,%d. world idx is %d\n", root->pos.x, root->pos.y, root->world_node_idx);
   // get basic path to destination
   path_map_node_type *path_root = &path_map->nodes[root->world_node_idx];
   set_path_direction_from_path_node(path_root, route_info);
}


// get desired route from path map and set viability score for each radial
//    based on how far away it is from desired route
static void calc_desired_heading_score(
      /* in out */       route_map_type *route_map,
      /* in out */       route_info_type *route_info
      )
{
   // get score for each radial relative to desired course
   for (uint32_t rad=0; rad<NUM_ROUTE_RADIALS; rad++) {
      double direction_score = calc_direction_agreement((uint8_t) rad,
            route_info->true_path_heading);
      route_map->radials[rad].direction_score = direction_score;
   }
}

// arc score is a measure of the arc width of passage along each radial
// low score indicates shallow and/or narrow passage; high score indicates
//    deep water and wide arc for passage
static void calculate_arc_scores(
      /* in out */       route_map_type *route_map

      )
{
   enum { TERRAIN, STAND_ON, GIVE_WAY, NUM_VIABILITIES };
   const int32_t ARC_SIZE = 24;
   const double scale = 1.0 / (double) ARC_SIZE;
//printf("Scale: %f\n", (double) scale);
   radial_viability_type *radials = route_map->radials;
   //
   for (uint32_t ival=0; ival<NUM_VIABILITY_INTERVALS; ival++) {
      for (uint32_t base=0; base<NUM_ROUTE_RADIALS; base++) {
         // get score for traversable arc size on base radial
         // look left and right X radials. define arc as sum of scores
         //    on radial with restriction that score for radial is <=
         //    lowest score of radials closest to base
         double ceiling[NUM_VIABILITIES];
         ceiling[TERRAIN] = radials[base].terrain_score[ival];
         ceiling[STAND_ON] = radials[base].stand_on_score[ival];
         ceiling[GIVE_WAY] = radials[base].give_way_score[ival];
         double score[NUM_VIABILITIES];
         for (uint32_t i=0; i<NUM_VIABILITIES; i++) {
            score[i] = ceiling[i];
         }
         // uint8 wraps around at 256 so we can avoid fancy clock logic
         uint8_t left_idx = (uint8_t) (base - 1);
         uint8_t right_idx = (uint8_t) (base + 1);
         for (int32_t r=1; r<ARC_SIZE; r++) {
            // get values on left and right for distance r
            // score for each viability type is lowest of left and right
            //    values and of previous lowest value
            double left[NUM_VIABILITIES];
            left[TERRAIN] = radials[left_idx].terrain_score[ival];
            left[STAND_ON] = radials[left_idx].stand_on_score[ival];
            left[GIVE_WAY] = radials[left_idx].give_way_score[ival];
            double right[NUM_VIABILITIES];
            right[TERRAIN] = radials[right_idx].terrain_score[ival];
            right[STAND_ON] = radials[right_idx].stand_on_score[ival];
            right[GIVE_WAY] = radials[right_idx].give_way_score[ival];
            for (uint32_t j=0; j<NUM_VIABILITIES; j++) {
               double low = left[j] < right[j] ? left[j] : right[j];
               ceiling[j] = low < ceiling[j] ? low : ceiling[j];
               score[j] += ceiling[j];
            }
            // advance indices
            left_idx--;
            right_idx++;
         }
         radials[base].terrain_arc[ival] = scale * score[TERRAIN];
         radials[base].stand_on_arc[ival] = scale * score[STAND_ON];
         radials[base].give_way_arc[ival] = scale * score[GIVE_WAY];
      }
   }
}

// path scoring
////////////////////////////////////////////////////////////////////////


// decide whether or not to change course
void decide_course_change(
      /* in     */ const double t_sec,
      /* in     */ const double last_course_request_sec,
      /* in out */       route_info_type *route_info,
      /* in     */ const vessel_position_info_type *vessel_info
      )
{
   (void) vessel_info;
//log_info(log_, "course change? t=%.3f  %.3f  %.3f", t_sec, last_course_request_sec, route_info->course_changed_sec);
   /////////////////////
   if ((t_sec - last_course_request_sec) <
         OTTO_COURSE_CHANGE_RESPSONSE_WINDOW_SEC) {
log_info(log_, "NOT Deciding course change, window");
      // previous course change request was recent enough that otto
      //    may not have acknowledged. don't try to change course again yet
      return;
   }
//   if (route_info->flags_course & ROUTE_INFO_COURSE_URGENT_CHANGE) {
//log_info(log_, "NOT Deciding course change, evasion");
//      // if urgent flag set, decision already made
//      return;
//   }
   /////////////////////////////////////////////////////////////////////
   dt_second_type dt = { .dt_sec = t_sec - route_info->course_changed_sec };
//log_info(log_, "Deciding course change. dT=%.3f", dt.dt_sec);
//   // if no course change for 5 minutes and best course is more than
//   //    6 deg different, suggest course change
   // if no course change for 3 minutes and best course is more than
   //    6 deg different, suggest course change
   // if best score is more than 10% different than current score and more than
   //    one minute has elapsed since previous change, suggest change
   // if best score is more than 20% above present, make change
   true_heading_type reference_heading;
   double reference_score;
   if (route_info->autopilot_course_score >= 0.0) {
//printf("Autopilot course\n");
      reference_heading = route_info->autopilot_course;
      reference_score = route_info->autopilot_course_score;
   } else {
      // otto not engaged -- use measured heading instead
//printf("measured heading\n");
      reference_heading = route_info->measured_heading;
      reference_score = route_info->measured_heading_score;
   }
   double avg = 0.5 * (route_info->sug_heading_score + reference_score);
   double pct_delta =
         fabs(route_info->sug_heading_score - reference_score) / avg;
//printf("pct_delta %.3f   dt %.3f  heading %.3f=%.3f (score %.3f)  course %.3f (%.3f)  sug heading %.3f (%.3f)\n", pct_delta, dt.dt_sec, (double) vessel_info->true_heading.tru.angle32 * BAM32_TO_DEG, (double) route_info->measured_heading.tru.angle32 * BAM32_TO_DEG, route_info->measured_heading_score, (double) route_info->autopilot_course.tru.angle32 * BAM32_TO_DEG, route_info->autopilot_course_score, (double) route_info->sug_heading.tru.angle32 * BAM32_TO_DEG, route_info->sug_heading_score);
   if (pct_delta >= 0.2) {
log_info(log_, "%.3f CHANGE  (absolute) best %.3f at %.1f; present %.3f at %.1f", t_sec, route_info->sug_heading_score, (double) route_info->sug_heading.tru.angle32 * BAM32_TO_DEG, reference_score, reference_heading.tru.angle32 * BAM32_TO_DEG);
      route_info->flags_course |= ROUTE_INFO_COURSE_MAKE_CHANGE;
      goto end_heading;
   }
   if ((pct_delta >= 0.1) && (dt.dt_sec > 60.0)) {
log_info(log_, "%.3f CHANGE  (improvement) best %.3f at %.1f; present %.3f at %.1f", t_sec, route_info->sug_heading_score, (double) route_info->sug_heading.tru.angle32 * BAM32_TO_DEG, reference_score, reference_heading.tru.angle32 * BAM32_TO_DEG);
      route_info->flags_course |= ROUTE_INFO_COURSE_SUGGEST_CHANGE;
      goto end_heading;
   }
//   const int32_t ten_deg32 = (int32_t) (10.0 * DEG_TO_BAM32);
   const int32_t five_deg32 = (int32_t) (5.0 * DEG_TO_BAM32_);
   int32_t delta_bam32 = abs((int32_t) (route_info->sug_heading.tru.angle32
         - reference_heading.tru.angle32));
   if ((dt.dt_sec > 180.0) && (delta_bam32 > five_deg32)) {
//log_info(log_, "dt %.3f   delta32 %d (%.1f)   ten32 %d (%.1f)", dt.dt_sec, delta_bam32, (double) delta_bam32 * BAM32_TO_DEG, ten_deg32, (double) ten_deg32 * BAM32_TO_DEG);
log_info(log_, "%.3f CHANGE  (stale 180) best %.3f at %.1f; present %.3f at %.1f", t_sec, route_info->sug_heading_score, (double) route_info->sug_heading.tru.angle32 * BAM32_TO_DEG, reference_score, reference_heading.tru.angle32 * BAM32_TO_DEG);
//log_info(log_, "ves head %.1f", (double) vessel_info->tru.heading.tru.angle32 * BAM32_TO_DEG);
      route_info->flags_course |= ROUTE_INFO_COURSE_SUGGEST_CHANGE;
      goto end_heading;
   }
   /////////////////////////////////////////////////////////////////////
   // TODO if no route is selected, or selection is poor (bad score,
   //    bad neighborhood scores, poor direction) then re-assess
   //    by making intermediate stops and checking routing
   //    possibilites from there    TODO
end_heading:
   // evalute for speed changes TODO
   // for now, recommended speed is the preferred speed (full speed ahead,
   //    damn the torpedoes)
   //route_info->sug_speed = route_info->pref_speed;
//end_speed:
   ;
}



// puts subscore in lowest if it's lower than values there
// lowest[0] should be the lowest of all values and lowest[1] is the
//    2nd lowest. if lowest[0] is repeated, repeats is incremented
void update_subscore(
      /* in out */       double lowest[2],
      /* in out */       int32_t *repeats,
      /* in     */ const double subscore
      )
{
assert(subscore > 0.0);
   if (lowest[0] > subscore) {
      lowest[1] = lowest[0];
      lowest[0] = subscore;
      *repeats = 0;
   } else if (lowest[0] == subscore) {
      *repeats =  *repeats + 1;
   } else if (lowest[1] > subscore) {
      lowest[1] = subscore;
   }
}


// combine lowest scores into single score value
// provide weight of lowest value with 7 + 3 times number repeats,
//    and weight of 1 for 2nd lowest
double combine_subscores(
      /* in     */ const double lowest[2],
      /* in     */ const int32_t repeats
      )
{
   double weight = (double) (7 + repeats * 3);
   double score = (weight + 1.0) / (weight / lowest[0] + 1.0 / lowest[1]);
   return score;
}



// calculate scores for each radial at each interval
// score based on agreement between radial and desired direction and
//    also on width of arc around that's free from obstacles
static void calc_radial_score(
      /* in out */       route_map_type *route_map
      )
{
   // 'offset' is bad name. this is the portion of the score on a given
   //    radial that is not touched -- ie, when an avoidance score is
   //    computed it's scaled on (1.0 - ival_offset), so an 'offset'
   //    score of 0.2 means the score is not less than 0.2 and the score
   //    is scale from [0,1] to [0.2,1]
   const double ival_offset[NUM_VIABILITY_INTERVALS] = {
      0.0, 0.1, 0.4, 0.8, 0.9
   };
   // examine arc around terrain and collision risks
   calculate_arc_scores(route_map);
   // combine scores
   radial_viability_type *radials = route_map->radials;
   // direction is selected based on radial with highest score, with each
   //    radial score being based on the lowest scores on that radial
   for (uint32_t rad=0; rad<256; rad++) {
      radial_viability_type *radial = &radials[rad];
      // nearby risks are scored fully (short ival) while further
      //    risks are used more for bias (ie, are given positive offsets)
//      // take lowest score for all intervals for all modalities
//      double min_collis = 20.0;
//double min_stand_on = 2.0;
//printf("rad %d     direction %.3f\n", rad, (double) radial->direction_score);
//if ((rad & 7) == 0) {
//   printf("rad %d (%.1f)    direction %.3f\n", rad, (double) rad * BAM8_TO_DEG, (double) radial->direction_score);
//}
      // for each modality, use harmonic average to combine lowest scores
      // first attempt was to use lowest score on radial for all modalities.
      //    this failed because if there was a very low score on all
      //    modalities (e.g., because vessel was in very shallow water)
      //    then all radials would return that low score, and the first
      //    radial (ie, north) would be selected
      // now take into account multiple low scores. this can be done
      //    by selecting lowest 2 or 3 and combining those in a weighted
      //    harmonic mean. do this on each modality separately and
      //    later combine modalities
      double lowest_terrain[2] = { 1.0, 1.0 };
      double lowest_stand_on[2] = { 1.0, 1.0 };
      // how many repeats there are of lowest score
      int32_t terrain_repeats = 0;
      int32_t stand_on_repeats = 0;
      //
      //    depth
      for (uint32_t ival=0; ival<NUM_VIABILITY_INTERVALS; ival++) {
         double offset = ival_offset[ival];
         double scale = 1.0 - offset;
         //
         double ter = offset + radial->terrain_score[ival] * scale;
         double sta = offset + radial->stand_on_score[ival] * scale;
         update_subscore(lowest_terrain, &terrain_repeats, ter);
         update_subscore(lowest_stand_on, &stand_on_repeats, sta);
//if ((rad & 7) == 0) {
//   printf("  %d   ter: %.4f    col: %.4f   giv: %.4f\n", ival, ter, sta, radial->give_way_score[ival]);
//   //printf("  %d     %.4f\n", ival, (double) radial->terrain_score[ival]);
//}
      }
      double terrain_score =
            combine_subscores(lowest_terrain, terrain_repeats);
      double stand_on_score =
            combine_subscores(lowest_stand_on, stand_on_repeats);
//if ((rad & 7) == 0) {
//   printf("      ter: %.4f    col: %.4f\n", terrain_score, stand_on_score);
//}
      // also use radial->direction_score
      assert(terrain_score > 0.0);
      assert(stand_on_score > 0.0);
      assert(radial->direction_score > 0.0);
      //
      // combine these scores
      const double terrain_wt = 2.0;
      const double stand_on_wt = 2.0;
      const double direction_wt = 1.0;
      double score = terrain_wt + stand_on_wt + direction_wt;
      score /= terrain_wt / terrain_score +
            stand_on_wt / stand_on_score +
            direction_wt / radial->direction_score;
      assert(score > 0.0);
//assert(score > 0.000);
//if ((rad & 3) == 0) {
//   printf("%.3d  dir: %.3f  stand_on: %.3f   collis: %.3f   -> %.3f\n", rad, (double) radial->direction_score, (double) min_stand_on, (double) min_collis, (double) score);
//}
//printf("%.3d  dir: %.3f  stand_on: %.3f   collis: %.3f  -> %.3f\n", rad, (double) radial->direction_score, (double) min_stand_on, (double) min_collis, (double) score);
      radial->net_score = score;
//if ((rad & 7) == 0) {
//   printf("%d  \t\tdirection score %.4f   net %.4f\n", rad, (double) radial->direction_score, (double) radial->net_score);
//   //printf("  %d     %.4f\n", ival, (double) radial->terrain_score[ival]);
//}
   }
//exit(0);
}

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////


// viability data has been collected across radials
// analyze possible routes and select best one
static void select_route(
      /* in     */ const path_map_type *path_map,
      /* in out */       route_map_type *route_map,
      /* in out */       route_info_type *route_info,
      /* in     */ const vessel_position_info_type *vessel_info
      )
{
   // only get data from path map if we know where we are
   // otherwise, stick with last known suggested route
   // TODO need to fade path vector with time, and/or make direction score
   //    masking much weaker
   if (route_info->flags_state & ROUTE_INFO_STATE_CHECK_TERRAIN) {
//printf("get_path_offsets\n");
      // pull desired direction from path map
      get_path_offsets(path_map, route_map, route_info);
   }
   // assess each radial for its deviation from desired direction
   calc_desired_heading_score(route_map, route_info);
   // calculate score for each radial at each interval
   // combine interval scores
   calc_radial_score(route_map);
   /////////////////////////////////////////////////////////////////////
   // select best
   // calculate change speed/heading scores given existing and best
   radial_viability_type *radials = route_map->radials;
   double best_score = -1.0;
   true_heading_type best_heading = { .tru = { .angle32 = 0 } };
   // TODO make a note of whether collision risk prevents from taking
   //    prefered route. if so, once collision risk is over we can
   //    resume course
   double best_path_score = -1.0;
   true_heading_type best_path_heading = { .tru = { .angle32 = 0 } };
   for (uint32_t rad=0; rad<NUM_ROUTE_RADIALS; rad++) {
      if (radials[rad].net_score > best_score) {
         best_score = radials[rad].net_score;
         best_heading.tru.angle32 = (uint32_t) (rad << 24);
      }
      if (radials[rad].direction_score > best_path_score) {
         best_path_score = radials[rad].direction_score;
         best_path_heading.tru.angle32 = (uint32_t) (rad << 24);
      }
   }
   // if course is more than X degrees off desired then set divert flag
   // set X as ~11 degs (360/32)
   if ((best_path_heading.tru.angle32>>27) !=
         (best_heading.tru.angle32>>27)) {
      route_info->flags2_persistent |= ROUTE_INFO_DIVERT;
      route_info->flags2_persistent &=
            (uint32_t) (~ROUTE_INFO_PATH_CLEAR);
   } else {
      // close enough to preferred course
      route_info->flags2_persistent |= ROUTE_INFO_PATH_CLEAR;
   }
   route_info->sug_heading = best_heading;
   route_info->sug_heading_score = best_score;
   route_info->measured_heading = vessel_info->true_heading;
   route_info->measured_heading_score =
         radials[vessel_info->true_heading.tru.angle32>>24].net_score;
//   route_info->autopilot_course_score =
//         radials[route_info->autopilot_course.tru.angle32>>24].net_score;
//         radials[vessel_info->true_heading.tru.angle32>>24].net_score;
log_info(log_, "Best score %.4f at %.1f. Autopilot course score %.4f at %.1f", (double) best_score, (double) best_heading.tru.angle32 * BAM32_TO_DEG, (double) route_info->autopilot_course_score, (double) route_info->autopilot_course.tru.angle32 * BAM32_TO_DEG);
//printf("Best score %.4f at %.1f. Present course score %.4f at %.1f\n", (double) best_score, (double) best_heading.tru.angle32 * BAM32_TO_DEG, (double) route_info->pres_course_score, (double) route_info->pres_course.tru.angle32 * BAM32_TO_DEG);
   // suggest direction of turn to get to suggested heading
   // the better way to do this is to evaluate viability and weight the
   //    most viable turn with how much turn is needed. TODO change approach
   // in most cases it will be just fine to change direction based on
   //    the smallest turn to get to the new desired heading
   uint32_t present = vessel_info->true_heading.tru.angle32;
   uint32_t suggested = route_info->sug_heading.tru.angle32;
   // BAMs requried to get from present to suggested
   bam32_type turn = { .angle32 = suggested - present };
   if (turn.sangle32 < 0) {
      route_info->turn_rate.dps = -1.0;
   } else {
      route_info->turn_rate.dps = 1.0;
   }
   /////////////////////////////////////////////////////////////////////
   // TODO if world grid cannot be traversed at a point because of shore/
   //    depth constraints, and present scoring algorithm, and this occurs
   //    multiple times in the same world grid point, flag point as
   //    non-passable and regenerate path map. these points should be
   //    stored in a persistent location so when map is reloaded they
   //    will be reapplied
}



// see if position is w/in arrival radius of destination
// if already reached map grid of destination, victory will be declared
//    as well (from another code path)
static void check_victory_conditions(
      /* in out */       route_info_type *route_info,
      /* in     */ const vessel_position_info_type *vessel_info
      )
{
   meter_type dx, dy;
   calc_meter_offset(vessel_info->position, route_info->destination,
         &dx, &dy, __func__);
   double dist_met = sqrt(dx.meters * dx.meters + dy.meters * dy.meters);
   if (dist_met <= route_info->destination_radius.meters) {
      route_info->flags_state |= ROUTE_INFO_STATE_REACHED_DESTINATION;
   }
//printf("Distance from destination: %.1f meters\n", (double) dist_met);
}


// vessel is at center
// update route node values. includes assigning route nodes to world and
//    path map nodes. computes time to reach other route nodes, and thus
//    underlying world nodes
// also resets rest of node's state values
// TODO interpolate depth in near-shore world nodes, to stay further away
//    from edges of nodes containing hazards
static void reset_route_nodes(
      /* in     */ const path_map_type *path_map,
      /* in out */       route_map_type *route_map,
      /* in     */ const vessel_position_info_type *vessel_info,
      /* in     */ const route_info_type *route_info
      )
{
   const world_coordinate_type vessel_pos = vessel_info->position;
   const meters_per_second_type vessel_speed = vessel_info->speed;
   // 'radius' of routing map (ie, width/2 in meters)
   const meter_type routing_radius =
         { .meters = route_map->node_width.meters * route_map->size.x / 2 };
   // 'radius' of routing node. radius here is to corners, not sides. this
   //    will result in overlap between nodes but as we're only looking at
   //    intersecting paths, that will provide some safety cushion
   const meter_type route_node_radius =
         { .meters = route_map->node_width.meters / 1.414 };
printf(" vessel pos %.6f,%.6f  speed %.2f heading %.1f  xy mps:%.3f,%.3f\n", (double) vessel_pos.x_deg, (double) vessel_pos.y_deg, (double) vessel_speed.mps, (double) vessel_info->true_heading.tru.angle32 * BAM32_TO_DEG, (double) vessel_info->xy_motion.x_mps, (double) vessel_info->xy_motion.y_mps);
//printf(" routing radius %.1f met    node radius %.1f met\n", (double) routing_radius.meters, (double) route_node_radius.meters);
   /////////////////////////////
   // get meter offset of vessel from world map center
   // world and plan maps are rectangular nautical miles while routing map is
   //    rectangular meters, so no distortion correction necessary (world
   //    and plan used to be degrees, in which case there was distortion)
   // routing map is smaller than world/plan maps and has smaller grids
   image_size_type size = route_map->size;
   meter_type dx, dy;
//fprintf(stderr, "ves pos: %.3f,%.3f\n", vessel_pos.x_deg, vessel_pos.y_deg);
//fprintf(stderr, "map center: %.3f,%.3f\n", path_map->center.x_deg, path_map->center.y_deg);
   calc_meter_offset(path_map->center, vessel_pos, &dx, &dy, __func__);
//printf("  vessel at %.4f,%.4f,   world map center offset (m) %.1f,%.1f\n", vessel_pos.lon, vessel_pos.lat, dx.meters, dy.meters);
// convert meters to pixels (grid squares)
int32_t x_offset_pix =
      (int32_t) floor(dx.meters / path_map->node_width.meters);
// positive dy is up, whereas here it should be down as array origin is
//    in top-left
int32_t y_offset_pix =
      (int32_t) floor(-dy.meters / path_map->node_height.meters);
printf("    node offset %d,%d\n", 360+x_offset_pix, 360+y_offset_pix);
   // distance to top/left edge of route map, relative to world map center
   // positive dx is rightward
   // positive dy should be downward (array origin is top-left) but returned
   //    value has +dy as upward. need to invert it
   double route_left_offset_met = dx.meters -
         (double) (size.x / 2) * ROUTE_MAP_NODE_WIDTH_METERS;
   double route_top_offset_met = -dy.meters -
         (double) (size.y / 2) * ROUTE_MAP_NODE_WIDTH_METERS;
//printf("Relative top/left of route map  %.1f,%.1f (met)\n", route_left_offset_met, route_top_offset_met);
//printf("Relative top/left of route map  %.1f,%.1f (met)\n", 360.0+floor(route_left_offset_met/path_map->node_width.meters), 360.0+floor(route_top_offset_met/path_map->node_width.meters));
   // top/left inset of route map in world map
   double route_left_inset_met =
         (double) (path_map->size.x/2) * path_map->node_width.meters +
         route_left_offset_met;
   double route_top_inset_met =
         (double) (path_map->size.y/2) * path_map->node_height.meters +
         route_top_offset_met;
//printf("Route inset left %.1fm  top %.1fm\n", route_left_inset_met, route_top_inset_met);
   /////////////////////////////
   // update nodes, with route map center being vessel position
assert(size.x == 255);
assert(size.y == 255);
   uint32_t route_idx = 0;
   for (uint32_t y=0; y<size.y; y++) {
      double y_inset_met = route_top_inset_met + y * ROUTE_MAP_NODE_WIDTH_METERS;
      // y index in world map
      uint32_t map_y_pos = (uint32_t)
            floor(y_inset_met / (double) path_map->node_height.meters);
      uint32_t y_row_idx = map_y_pos * path_map->size.x;
      assert(map_y_pos < 720);
      for (uint32_t x=0; x<size.x; x++) {
         route_map_node_type *node = &route_map->nodes[route_idx++];
         double x_inset_met =
               route_left_inset_met + x * ROUTE_MAP_NODE_WIDTH_METERS;
         // x index in world map
         uint32_t map_x_pos = (uint32_t)
               floor(x_inset_met / (double) path_map->node_width.meters);
         assert(map_x_pos < 720);
         uint32_t map_idx = map_x_pos + y_row_idx;
         //
         node->world_node_idx = map_idx;
         node->world_pos.x = (uint16_t) map_x_pos;
         node->world_pos.y = (uint16_t) map_y_pos;
assert(node->world_node_idx < 720 * 720);
//path_map_node_type *wn = &path_map->nodes[map_idx];
////printf("%d,%d -> map_idx %d (%d,%d)  depth %d\n", x, y, map_idx, map_x_pos, map_y_pos, wn->depth_meters);
//printf("%d,%d -> map_idx %d (%d,%d   %.2f,%.2f)  depth %d\n", x, y, map_idx, map_x_pos, map_y_pos, x_inset_met / (double) path_map->node_width.meters, y_inset_met / (double) path_map->node_height.meters, wn->depth_meters);
         ///////////////////////////////////////////////////////////////
         // rest of state values
         // default to "don't go to this node". that will be overriden
         //    when updating terrain viaiblity
         node->terrain_score = 0.0001;
         // time to arrive at and leave node
         double dist_near_m = node->distance.radians * routing_radius.meters
               - route_node_radius.meters;
         if (dist_near_m < 0.0) {
            dist_near_m = 0.0;
         }
         double dist_far_m = node->distance.radians * routing_radius.meters
               + route_node_radius.meters;
assert(node->distance.radians >= 0.0);
         if (vessel_speed.mps > 0.0) {
            node->arrival_dt_sec = dist_near_m / vessel_speed.mps;
            node->exit_dt_sec = dist_far_m / vessel_speed.mps;
//printf("    arrive %.3f   exit %.3f    speed %.3f   dist_near %.3f   dist_far %.3f\n", node->arrival_dt_sec, node->exit_dt_sec, vessel_speed.mps, dist_near_m, dist_far_m);
//printf(  "%d,%d  arrive %.3f   exit %.3f    speed %.3f   dist_near %.3f   dist_far %.3f\n", x, y, node->arrival_dt_sec, node->exit_dt_sec, vessel_speed.mps, dist_near_m, dist_far_m);
         } else {
            // if vessel has no speed then it's possible that the speed
            //    indicator is broken. use default speed
            assert(route_info->default_speed.mps > 0.0);
            node->arrival_dt_sec = dist_near_m / route_info->default_speed.mps;
            node->exit_dt_sec = dist_far_m / route_info->default_speed.mps;
//printf("    arrive %.3f   exit %.3f    def speed %.3f   dist_near %.3f   dist_far %.3f\n", node->arrival_dt_sec, node->exit_dt_sec, route_info->default_speed.mps, dist_near_m, dist_far_m);
//printf("  %d,%d  arrive %.3f   exit %.3f    def speed %.3f   dist_near %.3f   dist_far %.3f\n", x, y, node->arrival_dt_sec, node->exit_dt_sec, route_info->default_speed.mps, dist_near_m, dist_far_m);
         }
      }
//if (y&1) {
//   printf("\n");
//}
   }
}


// analyze routes and provide options. options are stored in radials
//    at each time interval
static void find_available_routes(
      /* in     */ const path_map_type *path_map,
      /* in out */       route_map_type *route_map,
      /* in out */       route_info_type *route_info,
      /* in out */       vessel_position_info_type *vessel_info
      )
{
   // route map is rebuilt each time, centered on vessel
   reset_route_nodes(path_map, route_map, vessel_info, route_info);
//uint32_t center = (uint32_t) (route_map->size.x/2 + (route_map->size.y/2)*route_map->size.x);
//printf("route center idx: %d (from %d,%d)\n", center, route_map->size.x, route_map->size.y);
//route_map_node_type *rnode = &route_map->nodes[center];
//world_map_node_type *wnode = &world_map->nodes[rnode->world_node_idx];
//path_map_node_type *pnode = &path_map->nodes[rnode->world_node_idx];
//printf("  depth %d    direction %d,%d\n", wnode->depth_meters, pnode->direction.dx, pnode->direction.dy);
   // only assess terrain risk if position info available and up to date
   if (route_info->flags2_persistent & ROUTE_INFO_HAVE_POSITION) {
      assess_terrain_risks(path_map, route_map);
      push_node_viabilities_to_radials(route_map);
      route_info->flags_state |= ROUTE_INFO_STATE_CHECK_TERRAIN;
   } else {
      // use last known position to get approximate direction
      if (route_info->present_speed.mps > 0.0) {
         vessel_info->speed = route_info->present_speed;
      } else {
         vessel_info->speed = route_info->default_speed;
      }
      set_path_offset_by_last_known_position(path_map, route_info);
//printf("Position data not available. Using last known position  %.3f,%.3f -> %d,%d  %d,%d\n", route_info->last_known_position.x_deg, route_info->last_known_position.y_deg, route_info->path_offset.dx, route_info->path_offset.dy, route_info->long_path_offset.dx, route_info->long_path_offset.dy);
   }
}


static void clear_viability_radials(
      /* in out */       route_map_type *route_map
      )
{
   radial_viability_type *radials = route_map->radials;
   //////////
   // reset radials to 1.0 then update w/ route node values
   for (uint32_t i=0; i<NUM_ROUTE_RADIALS; i++) {
      for (uint32_t ival=0; ival<NUM_VIABILITY_INTERVALS; ival++) {
         radials[i].terrain_score[ival] = 1.0;
         radials[i].stand_on_score[ival] = 1.0;
         radials[i].give_way_score[ival] = 1.0;
      }
   }
}


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
      )
{
   // TODO check if aiming directly for destination or if at intermediate
   //    beacon. add beacon logic
printf("Plotting route from %.6f,%.6f   dps %f,%f\n", (double) vessel_info->position.x_deg, (double) vessel_info->position.y_deg, (double) vessel_info->motion_per_second.x_deg, (double) vessel_info->motion_per_second.y_deg);
   route_info->flags_all = 0;
   clear_viability_radials(route_map);
   /////////////////////
   // from avoidance.c
   // identify routes that avoid water hazards (eg, shallowness and land)
   find_available_routes(path_map, route_map, route_info, vessel_info);
   // find routes that reduce collision risk
#if defined(USE_TRACKING)
   analyze_traffic(ass_out, route_info, vessel_info, route_map, t_sec);
#else
   (void) ass_out;
   (void) t_sec;
#endif   // USE_TRACKING
   /////////////////////
   if ((route_info->flags_state & ROUTE_INFO_STATE_CHECK_MASK) != 0) {
//printf("selecting route\n");
      // we have terrain (position) and/or traffic info. good enough for
      //    now, but an alert should signal that something's wrong
      //    w/ the data stream if both aren't present TODO
      select_route(path_map, route_map, route_info, vessel_info);
      check_victory_conditions(route_info, vessel_info);
   } else {
printf("RUNNING BLIND\n");
      // terrain and traffic data not available so we're running blind
      route_info->flags_state |= ROUTE_INFO_STATE_RUNNING_BLIND;
   }
}

#if NUM_ROUTE_RADIALS != 256
// if this is triggered, a code review is necessary
#error "Code based on bam8 angle representation which means 256 radials"
#endif   // NUM_ROUTE_RADIALS != 256

#endif   // ROUTE_MAP_C

