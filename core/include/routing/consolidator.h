#if !defined(CONSOLIDATOR_H)
#define CONSOLIDATOR_H
#if !defined(_GNU_SOURCE)
#define _GNU_SOURCE
#endif   // _GNU_SOURCE
#include "pinet.h"
#include "datap.h"
#include "logger.h"
#include "routing/support/consolidate_targets.h"

#define CONSOLIDATOR_LOG_LEVEL     LOG_LEVEL_DEFAULT

// takes input from different sensor streams (eg, optical tracking + AIS)
//    and tries to associate signals that appear to originate from the
//    same targets
//
// target information is aligned to time boundaries in order to avoid getting
//    into unnecessary update loop (that might fall behind) in case of
//    many input feeds
// time boundaries are expected to be quarter or half-second, something
//    smaller than the expected revisit-time for each target
// if this varies from being even divisor of one second (eg, 1/4, 1/3, 1/2)
//    then time alignment logic needs to be evaluated

#define CONSOLIDATOR_ALIGNMENT_BOUNDARY_SEC     0.5

// target records kept to maximum width
#define MAX_RECORD_WIDTH_B16     (6.0 * DEG_TO_BAM16)

// target shrinkage, per second
// eg, 0.05 means record width reduced by 1 deg in 20 sec
#define RECORD_BOUNDARY_REDUCTION_DPS     0.05


// NOTE it's possible for sequential outputs to have same timestamp.
//    this can occur if data is received from one modality that's
//    older than data has been published from a different one
struct consolidator_output {
   consolidation_array_type   targets;
};
typedef struct consolidator_output consolidator_output_type;

// queue needn't be very long as most recent element is what should be
//    pulled
#define CONSOLIDATOR_QUEUE_LEN   8

//
////////////////////////////////////////////////////////////////////////
//

#define CONSOLIDATOR_CLASS_NAME  "consolidator"

struct consolidator_class {
   producer_record_type *tracker_list[MAX_ATTACHED_PRODUCERS];
   uint32_t num_trackers;

   // time of most recent data update. corresponds to final entry int ts[]
   double last_update;

   //
   consolidation_grid_type grid;

   //
   log_info_type *log;

};
typedef struct consolidator_class consolidator_class_type;
typedef struct consolidator_class consolidator_type;

// thread entry point
void * consolidator_init(void *);

// struct to pass config data to thread
struct consolidator_setup {
   int unused;
};
typedef struct consolidator_setup consolidator_setup_type;

////////////////////////////////////////////////////////////////////////
// API

#endif   // CONSOLIDATOR_H

