#if !defined(LOGGER_H)
#define LOGGER_H
#if !defined(_GNU_SOURCE)
#define _GNU_SOURCE
#endif   // _GNU_SOURCE

// logger file should not require any project imports for base functionality
// provide wrapper to allow to run detached from rest of project (for testing)
#if defined(TEST_LOGGER)
#define ACQUISITION_STORAGE_ROOT    "/tmp/"
#define STR_LEN    256
#define hard_exit(func, line)  printf("Error at %s:%d\n", func, line)
#else
#include "pinet.h"
#include "pin_types.h"
#endif   // TEST_LOGGER

#include <stdio.h>
#include <stdint.h>

// log levels used by logger. there is one system-wide log setting, and
//    individual loggers can override this
// when an individual logger is flagged as default then the system-wide
//    setting is used. otherwise it should be set as one of the above
//    values

enum log_level { 
   LOG_LEVEL_ALL,    // all logging, including debug (default)
   LOG_LEVEL_INFO,   // info, warnings and errors
   LOG_LEVEL_WARN,   // warnings and errors
   LOG_LEVEL_ERR,    // errors only
   LOG_LEVEL_NONE,   // no logging
   LOG_LEVEL_DEFAULT    // use system-wide log settings
};
typedef enum log_level log_level_type;

#define LOG_LEVEL_DEFAULT  LOG_LEVEL_INFO

// maximum number of registered loggers
#define MAX_LOGGERS   256


struct log_info {
   // pointer to log file for this process
   // file name is /pinet/data/<date>/log_<name>.txt
   FILE *fp;
   char name[STR_LEN];  // process name
   log_level_type level;
};
typedef struct log_info log_info_type;

// returns initialized logger info structure
log_info_type * get_logger(
      /* in     */ const char * name
      );

// set log level on an individual logger, overriding system-wide setting
void set_log_level(
      /* in out */       log_info_type *logger,
      /* in     */ const log_level_type
      );

void set_default_log_level(
      /* in     */ const log_level_type
      );

void set_log_dir_string(
      /* in     */ const char *log_root
      );

const char* get_log_root_dir(void);

// get master log for process
log_info_type * get_kernel_log(void);

// explicitly initialize kernel logger
// returns pointer to kernel log, or NULL if there was an error creating it
log_info_type * init_kernel_log(void);

// like init_kernel_log but forces kernel to use supplied name
// this is relevant for systems where multiple logger processes are running
//    simultaneously (e.g., rpi)
log_info_type * init_kernel_log_rename(
      /* in     */ const char *name
      );


// flushes all logs to disk
void flush_logs(void);

// closes all logs
// NOTE: files are opened in append mode, so it should be safe to
//    close_logs() and subsequently write to the same logger
void close_logs(void);



// write info to log file, if logging call priority is at or above 
//    set logging level
void log_info(
      /* in out */       log_info_type *logger,
      /* in     */ const char *fmt,
      /* in     */ ...
      );

void log_err(
      /* in out */       log_info_type *logger,
      /* in     */ const char *fmt,
      /* in     */ ...
      );

void log_warn(
      /* in out */       log_info_type *logger,
      /* in     */ const char *fmt,
      /* in     */ ...
      );

// prints debug info to info channel if set_debug(1) has been called
void log_debug(
      /* in out */       log_info_type *logger,
      /* in     */ const char *fmt,
      /* in     */ ...
      );


// assert taken from JPL coding guidelines
// output target (eg, file, stdout) controlled in log_print function
// returns 0 on no error
#define c_assert(e) ((e) ? 0 : (log_err(get_kernel_log(), "%s:%d  assertion '%s' failed", __FILE__, __LINE__, #e), 1))

#define c_assert_test(e) ((e) ? 0 : 1)


// returns pointer to static storage that holds logging directory name
const char * get_log_folder_name(void);

void log_errors_to_stdout(void);

// copy file at path 'fname' to logging directory, writing there under
//    name 'outname'
int copy_file_to_logging_dir(
      /* in     */ const char *fname,
      /* in     */ const char *outname
      );

#endif   // LOGGER_H
