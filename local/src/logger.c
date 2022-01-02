#if defined(TEST_LOGGER)
#include "../include/logger.h"
#define ACQUISITION_STORAGE_ROOT    "/tmp/"
#define STR_LEN    256
#define NOW       0.0
#else
#include "logger.h"
#include "timekeeper.h"
#define NOW       now()
#endif   // TEST_LOGGER

#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdlib.h>
#include <time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/sendfile.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <assert.h>


// array slot 0 is reserved for kernel log entries
static log_info_type loggers_[MAX_LOGGERS];

static uint32_t num_active_loggers_ = 0;

// log folder
static char path_[STR_LEN];

// log folder root
static char log_root_[STR_LEN] = { 0 };

// global logging level. this can be overriden by settings in individual
//    logs
static log_level_type level_ = LOG_LEVEL_ALL;  // default logging level

log_info_type *get_kernel_log(void)
{
   if (num_active_loggers_ == 0)
      return NULL;
   return loggers_;
}

void set_default_log_level(
      /* in     */ const log_level_type level
      )
{
   level_ = level;
}

void set_log_dir_string(
      /* in     */ const char *log
      )
{
   size_t len = sizeof log_root_;
   strncpy(log_root_, log, len-1);
   log_root_[len-1] = 0;
}

const char* get_log_root_dir(void)
{
   return log_root_;
}

// creates a directory under ACQUISITION_STORAGE_ROOT and returns the
//    full name of the new directory. the returned string is static
//    and will change if this procedure is called again
static int32_t create_log_folder(void)
{
   struct tm tmt;
   time_t tt;
   struct stat st = {0};
   // check root_dir
   assert(log_root_[0] != 0);
   if (stat(log_root_, &st) != 0) {
      fprintf(stderr, "Log root directory '%s' doesn't exist: %s", 
            log_root_, strerror(errno));
      goto err;
   }
   // get time string
   if (time(&tt) < 0) {
      perror("Error reading time");
      goto err;
   }
   if (localtime_r(&tt, &tmt) == NULL) {
      perror("Error reading localtime");
      goto err;
   }
   char time_buf[32];
   strftime(time_buf, sizeof(time_buf), "%Y-%m-%d_%H-%M-%S", &tmt);
   // create folder name as full path
   snprintf(path_, sizeof(path_), "%s%s/", log_root_, time_buf);
   // make directory
   if (stat(path_, &st) == 0) {
      fprintf(stderr, "Directory '%s' already exists\n", path_);
      // directory may already exist on RPi, as multiple processes
      //    may be have started ~concurrently (e.g., supervisor and s2)
      // this shouldn't break anything so consider it acceptable
      goto done;
   }
   if (mkdir(path_, 0777) != 0) {
      perror("mkdir error for logging directory");
      goto err;
   }
done:
   printf("Logging directory: %s\n", path_);
   return 0;
err:
   fprintf(stderr, "Error creating logging directory\n");
   return -1;
}

log_info_type * init_kernel_log()
{
   return init_kernel_log_rename("kernel");
}

log_info_type * init_kernel_log_rename(
      /* in     */ const char * name
      )
{
   if (num_active_loggers_ == 0) {
      // set all log structure values to NULL
      memset(loggers_, 0, MAX_LOGGERS * sizeof loggers_[0]);
      // create kernel log
      num_active_loggers_ = 1;
      // create kernel's log
      if (create_log_folder() != 0) {
         // unable to create output folder
         // this should be a fatal error but allow it. it's possible
         //    that a running system will run out of storage space, and
         //    there's no reason that failure to write logs should
         //    bring the app down
         fprintf(stderr, "UNABLE TO CREATE LOG FOLDER, but continuing\n");
         return NULL;
      }
      loggers_->level = LOG_LEVEL_DEFAULT;
      size_t len = sizeof loggers_->name;
      strncpy(loggers_->name, name, len-1);
      loggers_->name[len-1] = 0;
      loggers_->fp = NULL;
   }
   return loggers_;
}

log_info_type * get_logger(
      /* in     */ const char * name
      )
{
   // first see if this log has been created. 
   // if so, return it. if not, create new
   for (uint32_t i=0; i<num_active_loggers_; i++) {
      log_info_type *log = &loggers_[i];
      if (strcmp(name, log->name) == 0)
         return log;
   }
   /////////////////////////////////////////////////////////////////////
   // create new logger
   if (num_active_loggers_ >= MAX_LOGGERS) {
      log_err(loggers_, "Too many loggers created (max=%d)", MAX_LOGGERS);
      // treat this as a fatal error. it will fail if a given config has
      //    too many entries, which will be apparent before deployment
      hard_exit(__func__, __LINE__);
   }
   log_info_type *log = &loggers_[num_active_loggers_];
   if (num_active_loggers_ == 0) {
      if (init_kernel_log() == NULL)
         return NULL;
      // advance to next log slot for the requested log creation
      log = &loggers_[++num_active_loggers_];
   } 
   // increment log count
   num_active_loggers_++;
   log->level = LOG_LEVEL_DEFAULT;
   size_t len = sizeof log->name;
   strncpy(log->name, name, len-1);
   log->name[len-1] = 0;
   log->fp = NULL;
   return log;
}

void set_log_level(
      /* in out */       log_info_type *logger,
      /* in     */ const log_level_type level
      )
{
   if (logger)
      logger->level = level;
}

static void init_log_file(
      /* in out */       log_info_type *logger
      )
{
   // open file if it's not already open
   if (logger->fp == NULL) {
      // for full path of file
      char fullpath[2*STR_LEN];
      snprintf(fullpath, sizeof(fullpath), "%slog_%s", path_, logger->name);
      logger->fp = fopen(fullpath, "a");
      if (logger->fp == NULL) {
         if (logger == loggers_) {
            // this is the kernel's log. if file can't be opened here
            //    then there's no way to report the error. continue
            return;
         }
         // report the error to the kernel's log
         log_err(loggers_, "Unable to create output log file '%s'", fullpath);
      }
      log_info(loggers_, "Opened %s for writing", fullpath);
   }
}

#define VFPRINTF(fmt, level)      \
         do {                 \
            va_list args;     \
            va_start(args, fmt);    \
            fprintf(logger->fp, "%s %.6f: ", level, NOW);    \
            vfprintf(logger->fp, fmt, args);    \
            va_end(args);     \
            fprintf(logger->fp, "\n");    \
         } while (0)

void log_debug(
      /* in out */       log_info_type *logger,
      /* in     */ const char *fmt,
      /* in     */ ...
      )
{
   if (logger == NULL)
      return;
   // get log level
   log_level_type level = logger->level;
   if (level == LOG_LEVEL_DEFAULT)
      level = level_;
   // print data
   switch (level) {
      case LOG_LEVEL_ALL:
      {
         if (logger->fp == NULL)
            init_log_file(logger);
         char buf[STR_LEN];
         va_list args;
         va_start(args, fmt);
         vsprintf(buf, fmt, args);
         va_end(args);
         fprintf(logger->fp, "%s %.6f: %s\n", "DEBUG", NOW, buf);
         break;
      }
      case LOG_LEVEL_INFO:
      case LOG_LEVEL_WARN:
      case LOG_LEVEL_ERR:
      case LOG_LEVEL_NONE:
         break;
      default:
         // internal error
         fprintf(stderr, "Internal error -- unexpected log level %d\n", level);
         hard_exit(__func__, __LINE__);
   };
}


void log_info(
      /* in out */       log_info_type *logger,
      /* in     */ const char *fmt,
      /* in     */ ...
      )
{
   if (logger == NULL)
      return;
   // get log level
   log_level_type level = logger->level;
   if (level == LOG_LEVEL_DEFAULT)
      level = level_;
   // print data
   switch (level) {
      case LOG_LEVEL_ALL:
      case LOG_LEVEL_INFO:
      {
         if (logger->fp == NULL)
            init_log_file(logger);
         char buf[STR_LEN];
         va_list args;
         va_start(args, fmt);
         vsprintf(buf, fmt, args);
         va_end(args);
         fprintf(logger->fp, "%s %.6f: %s\n", "INFO", NOW, buf);
         break;
      }
      case LOG_LEVEL_WARN:
      case LOG_LEVEL_ERR:
      case LOG_LEVEL_NONE:
         break;
      default:
         // internal error
         fprintf(stderr, "Internal error -- unexpected log level %d\n", level);
         hard_exit(__func__, __LINE__);
   };
}

void log_warn(
      /* in out */       log_info_type *logger,
      /* in     */ const char *fmt,
      /* in     */ ...
      )
{
   if (logger == NULL)
      return;
   // get log level
   log_level_type level = logger->level;
   if (level == LOG_LEVEL_DEFAULT)
      level = level_;
   // print data
   switch (level) {
      case LOG_LEVEL_ALL:
      case LOG_LEVEL_INFO:
      case LOG_LEVEL_WARN:
      {
         if (logger->fp == NULL)
            init_log_file(logger);
         char buf[STR_LEN];
         va_list args;
         va_start(args, fmt);
         vsprintf(buf, fmt, args);
         va_end(args);
         fprintf(logger->fp, "%s %.6f: %s\n", "WARN", NOW, buf);
         break;
      }
      case LOG_LEVEL_ERR:
      case LOG_LEVEL_NONE:
         break;
      default:
         // internal error
         fprintf(stderr, "Internal error -- unexpected log level %d\n", level);
         hard_exit(__func__, __LINE__);
   };
}

void log_err(
      /* in out */       log_info_type *logger,
      /* in     */ const char *fmt,
      /* in     */ ...
      )
{
   if (logger == NULL)
      return;
   // get log level
   log_level_type level = logger->level;
   if (level == LOG_LEVEL_DEFAULT)
      level = level_;
   // print data
   switch (level) {
      case LOG_LEVEL_ALL:
      case LOG_LEVEL_INFO:
      case LOG_LEVEL_WARN:
      case LOG_LEVEL_ERR:
      {
         if (logger->fp == NULL)
            init_log_file(logger);
         char buf[STR_LEN];
         va_list args;
         va_start(args, fmt);
         vsprintf(buf, fmt, args);
         va_end(args);
         fprintf(logger->fp, "%s %.6f: %s\n", "ERR", NOW, buf);
         break;
      }
      case LOG_LEVEL_NONE:
         break;
      default:
         // internal error
         fprintf(stderr, "Internal error -- unexpected log level %d\n", level);
         hard_exit(__func__, __LINE__);
   };
}

const char * get_log_folder_name(void) 
{ 
   if (path_ == NULL)
      create_log_folder();
   return path_;   
}


void flush_logs(void)
{
   for (uint32_t i=0; i<num_active_loggers_; i++) {
      log_info_type *log = &loggers_[i];
      if (log->fp) {
         fflush(log->fp);
      }
   }
}

void close_logs(void)
{
printf("Closing logs %s\n", path_);
   // close log files, except for master
   for (uint32_t i=1; i<num_active_loggers_; i++) {
      log_info_type *log = &loggers_[i];
      if (log->fp) {
         log_info(loggers_, "Closing log for %s", log->name);
         fclose(log->fp);
         log->fp = NULL;
      }
   }
   // now close master log
   if (loggers_->fp) {
      log_info(loggers_, "Closing master log file");
      fclose(loggers_->fp);
      loggers_->fp = NULL;
   }
printf("Logs closed\n");
}

void log_errors_to_stdout(void)
{
   char cmd[2*STR_LEN];
   printf("\n");
   printf("----------------------------------------------------------\n");
   printf("-- Runtime error log:\n");
   sprintf(cmd, "grep ERR %slog_*", get_log_folder_name());
   execl("/bin/bash", "/bin/bash", "-c", cmd, (char*) NULL);
}


int copy_file_to_logging_dir(
      /* in     */ const char *fname,
      /* in     */ const char *outname
      )
{
   int rc = -1;
   const char *folder = get_log_folder_name();
   log_info_type *log = get_kernel_log();
   char dest[STR_LEN];
   sprintf(dest, "%s%s", folder, outname);
   log_info(log, "Copying '%s' to '%s'", fname, dest);
   int fd_in = -1;
   int fd_out = -1;
   // open input file
   if ((fd_in = open(fname, O_RDONLY)) == -1) {
      log_err(log, "Error opening input '%s': %s", fname, strerror(errno));
      goto end;
   }    
   // create output file
   if ((fd_out = creat(dest, 0660)) == -1)
   {
      log_err(log, "Error opening output '%s': %s", dest, strerror(errno));
      goto end;
   }
   /////////////////////////////////////////////
   // do the work
   struct stat st = {0};
   fstat(fd_in, &st);
   size_t size = (size_t) st.st_size;
   if (sendfile(fd_out, fd_in, NULL, size) < 0) {
      // only check for error conditions -- assume full file contents
      //    were copied -- it's not that big of a file
      log_err(log, "Failed to copy to '%s': %s", dest, strerror(errno));
      goto end;
   }
   // done
   rc = 0;
end:
   if (fd_in >= 0) {
      close(fd_in);
   }
   if (fd_out >= 0) {
      close(fd_in);
   }
   return rc;
}


////////////////////////////////////////////////////////////////////////

#if defined(TEST_LOGGER)
#include <math.h>

int main(int argc, char **argv)
{
   printf("NONE: %d\n", LOG_LEVEL_NONE);
   printf("ALL: %d\n", LOG_LEVEL_ALL);
   printf("INFO: %d\n", LOG_LEVEL_INFO);
   printf("WARN: %d\n", LOG_LEVEL_WARN);
   printf("ERR: %d\n", LOG_LEVEL_ERR);
   printf("Running logger test. NOTE: manual examination of output "
         "is required\n");
   printf("Log files written to /tmp/<date>");
   printf("/tmp/log_test1 should have info, warn and err\n");
   printf("/tmp/log_test2 should have warn and err\n");
   printf("/tmp/log_test3 should have debug, info, warn and err\n");
   printf("/tmp/log_test4 should not exist\n");
   printf("-------------\n\n");
   //
   log_info_type *logger1 = get_logger("test1");
   log_info_type *logger2 = get_logger("test2");
   log_info_type *logger3 = get_logger("test3");
   log_info_type *logger4 = get_logger("test4");
   //
   set_log_level(logger2, LOG_LEVEL_WARN);
   set_log_level(logger3, LOG_LEVEL_ALL);
   set_log_level(logger4, LOG_LEVEL_NONE);
   if (!logger1 || !logger2 || !logger3 || !logger4) {
      fprintf(stderr, "Failed to create one or more loggers\n");
      return 1;
   }
   //
   log_debug(logger1, "debug message");
   log_debug(logger2, "debug message");
   log_debug(logger3, "debug message");
   log_debug(logger4, "debug message");
   //
   log_info(logger1, "info message");
   log_info(logger2, "info message");
   log_info(logger3, "info message");
   log_info(logger4, "info message");
   //
   log_warn(logger1, "warn message");
   log_warn(logger2, "warn message");
   log_warn(logger3, "warn message");
   log_warn(logger4, "warn message");
   //
   log_err(logger1, "err message");
   log_err(logger2, "err message");
   log_err(logger3, "err message");
   log_err(logger4, "err message");
   //
   close_logs();
   //
   log_err(logger1, "err message 2");
   log_err(logger2, "err message 2");
   log_err(logger3, "err message 2");
   log_err(logger4, "err message 2");
   //
   close_logs();
   //
   return 0;
}

#endif   // TEST_LOGGER

