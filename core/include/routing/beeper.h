#if !defined(BEEPER_H)
#define BEEPER_H
#if !defined(_GNU_SOURCE)
#define _GNU_SOURCE
#endif   // _GNU_SOURCE
#include "pinet.h"
#include "datap.h"
#include "logger.h"

#include "routing/driver.h"

#define BEEPER_LOG_LEVEL     LOG_LEVEL_DEFAULT

// takes output from driver and turns it into audio signal that indicates
//    desired course of action

// at present there are no subscribers planned for beeper. it provides
//    audio output only
struct beeper_output {
   int unused;
};
typedef struct beeper_output beeper_output_type;

#define BEEPER_QUEUE_LEN   4

////////////////////////////////////////////////
#define KLAXON_MAX   1000.0
#define KLAXON_ALARM_MAX   (1.5 * KLAXON_MAX)
#define BEEP_MAX     800.0

#define PING_MAX     800.0

// how long between alerts for suggestions or requests to alter course
// intervals in seconds
#define BEEPER_SUGGEST_CHANGE_INTERVAL    30.0
#define BEEPER_MAKE_CHANGE_INTERVAL       10.0

////////////////////////////////////////////////////////////////////////

#define BEEPER_CLASS_NAME  "beeper"

struct beeper_class {
   log_info_type *log;
   // noise-making thread
   pthread_t   sound_tid;
   ////////////////////////////////
   // latest user alerts
   double last_change_alert;
};
typedef struct beeper_class beeper_class_type;
typedef struct beeper_class beeper_type;

// thread entry point
void * beeper_init(void *);

// struct to pass config data to thread
struct beeper_setup {
   int unused;
};
typedef struct beeper_setup beeper_setup_type;

#endif   // BEEPER_H
