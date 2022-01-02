#include <stdio.h>
#include <string.h>
#include <signal.h>
#include <stdlib.h>
#include <stdint.h>
#include <pthread.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <assert.h>
#include <errno.h>
#include <locale.h>
#include "kernel.h"
#include "datap.h"
#include "pinet.h"
#include "postmaster.h"
#include "timekeeper.h"
#include "script_iface.h"
#include "udp_sync.h"
#include "logger.h"
#include "dev_info.h"
#include "logger.h"
#include "build_version_kernel.h"
#include "build_version_lib.h"


static pthread_t postmaster_tid_;

static int32_t init_globals(void)
{
   pthread_mutex_init(&global_mutex_g, NULL);
   num_threads_g = 0;
   memset(thread_table_g, 0, sizeof(thread_table_g));
   // initialize process-relative clock to zero
   init_timekeeper();
   return 0;
//err:
//   return -1;
}

static void init_signals(void)
{
   struct sigaction sa;
   memset(&sa, 0, sizeof sa);
   sa.sa_handler = signal_exit;
   sigaction(SIGALRM, &sa, NULL);   // perform orderly shutdown
   sigaction(SIGINT, &sa, NULL);   // perform orderly shutdown
   sa.sa_handler = cb_fault;  // cb_fault in lib
   sigaction(SIGSEGV, &sa, NULL);   // segmentation fault
   sigaction(SIGFPE, &sa, NULL);   // floating point exception
   sigaction(SIGILL, &sa, NULL);   // illegal instruction exception
   sigaction(SIGBUS, &sa, NULL);   // bus error
}


static void run_postmaster(void)
{
   if (pthread_create(&postmaster_tid_, NULL, launch_postmaster, NULL) != 0) {
      perror("Error launching postmaster");
      hard_exit("run_postmaster", 1);
   }
}

////////////////////////////////////////////////////////////////////////
// sanity checking

static void sanity_check_env(void)
{
   // make sure environment is set
   const char *env = get_environment();
   if (env[0] == 0) {
      log_info(get_kernel_log(), "Environment name not set. Use "
            "set_environment() in config script");
      hard_exit(__FILE__, __LINE__);
   }
   // ensure environment directory exists
   char path[STR_LEN];
   snprintf(path, STR_LEN, "%s", env);
   struct stat buf;
   if (stat(path, &buf) != 0) {
      log_err(get_kernel_log(), "Failed to stat path '%s': %s", path, 
            strerror(errno));
      hard_exit(__FILE__, __LINE__);
   }
   if (!S_ISDIR(buf.st_mode)) {
      log_err(get_kernel_log(), "Environment directory '%s' not "
            "a directory", env);
      hard_exit(__FILE__, __LINE__);
   }
}

static void sanity_check_dev(void)
{
   // make sure device dir is set
   const char *dev = get_device_dir();
   if (dev[0] == 0) {
      log_info(get_kernel_log(), "Device name not set. Use "
            "set_device_dir() in config script");
      hard_exit(__FILE__, __LINE__);
   }
   // ensure device directory exists
   char path[STR_LEN];
   snprintf(path, STR_LEN, "%s", dev);
   struct stat buf;
   if (stat(path, &buf) != 0) {
      log_err(get_kernel_log(), "Failed to stat path '%s': %s", path, 
            strerror(errno));
      hard_exit(__FILE__, __LINE__);
   }
   if (!S_ISDIR(buf.st_mode)) {
      log_err(get_kernel_log(), "Device directory '%s' not a directory", dev);
      hard_exit(__FILE__, __LINE__);
   }
}

static void sanity_check_log(void)
{
   // make sure log directory is set
   const char *log = get_log_root_dir();
   if (log && (log[0] == 0)) {
      log_info(get_kernel_log(), "Log root not set. Use "
            "set_log_dir() in config script");
      hard_exit(__FILE__, __LINE__);
   }
   // ensure log directory exists
   char path[STR_LEN];
   snprintf(path, STR_LEN, "%s", log);
   struct stat buf;
   if (stat(path, &buf) != 0) {
      log_err(get_kernel_log(), "Failed to stat path '%s': %s", path, 
            strerror(errno));
      hard_exit(__FILE__, __LINE__);
   }
   if (!S_ISDIR(buf.st_mode)) {
      log_err(get_kernel_log(), "Log root directory '%s' not a "
            "directory", log);
      hard_exit(__FILE__, __LINE__);
   }
}

static void sanity_check(void)
{
   // make sure config and log directories have been specified, and exist
   sanity_check_env();
   sanity_check_dev();
   sanity_check_log();
   //
   if (PIX_PER_DEG[0] == 0.0) {
      log_err(get_kernel_log(), "PIX_PER_DEG not set. Run set_ppd() "
            "from config file");
      hard_exit(__FILE__, __LINE__);
   }
   if (WORLD_HEIGHT_ABOVE_HORIZ_DEGS < 0.0) {
      const char *env = get_environment();
      log_err(get_kernel_log(), "Horizon location not set. Run "
            "set_view_above_horizon() from config file", env);
      hard_exit(__FILE__, __LINE__);
   }
   if (WORLD_HEIGHT_BELOW_HORIZ_DEGS < 0.0) {
      log_err(get_kernel_log(), "Horizon location not set. Run "
            "set_view_below_horizon() from config file");
      hard_exit(__FILE__, __LINE__);
   }
}

// sanity checking
////////////////////////////////////////////////////////////////////////

static void run_all_processors(void)
{
   // create barrier and then release the condition wait on all
   pthread_barrier_init(&barrier_g, NULL, num_threads_g);
   // barrier created -- release threads for initialization now
   for (uint32_t i=0; i<num_threads_g; i++) {
      pthread_cond_signal(&thread_table_g[i].condition);
   }
   // wait for all threads to exit
   printf("Master thread waiting for processes to exit\n");
   for (uint32_t i=0; i<num_threads_g; i++) {
      pthread_join(thread_table_g[i].thread_id, NULL);
   }
}

static void shutdown_postmaster(void)
{
   printf("Shutdown postmaster\n");
   pthread_join(postmaster_tid_, NULL);
}

static void clean_up_globals(void)
{
   log_info(get_kernel_log(),
         "Acquisition storage directory: %s", get_log_folder_name());
   pthread_mutex_destroy(&global_mutex_g);
   pthread_barrier_destroy(&barrier_g);
   shutdown_timekeeper();
printf("clearn_up_globals -> close_logs\n");
   close_logs();
}

static void report_errors(void)
{
   char cmd[STR_LEN];
   printf("\n");
   printf("----------------------------------------------------------\n");
   printf("-- Runtime error log:\n");
   sprintf(cmd, "grep ERR %slog_*", get_log_folder_name());
   execl("/bin/bash", "/bin/bash", "-c", cmd, (char*) NULL);
}

////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv)
{
   printf("version\n  bob: %s\n  lib: %s\n", 
         KERNEL_BUILD_VERSION, LIB_BUILD_VERSION);
   if (argc != 2) {
      fprintf(stderr, "Usage: %s <config.lua>\n", argv[0]);
      return -1;
   }
   set_log_dir_string(LOG_DIRECTORY_ROOT);
   init_kernel_log();
fprintf(stderr, "Logging data to %s\n", get_log_folder_name());
   report_thread_id_by_name("runtime", get_kernel_log());
   log_info(get_kernel_log(), "Kernel version %s", KERNEL_BUILD_VERSION);
   log_info(get_kernel_log(), "Lib version %s", LIB_BUILD_VERSION);
//   if (setlocale(LC_ALL, "en_US.UTF-8") == NULL) {
//      log_err(get_kernel_log(), "Unable to set locale to en_US (needed for "
//            "string processing). Crossing fingers...");
//   }
   //
   if (init_globals() != 0)
      return -1;
   //////////////////////////////
   // setup system infrastructure
   init_signals();   // catch and handle system signals
//   //////////////////////////////////////////////
//   // dev-oriented procedure -- set an exit alarm
//   //    so system automatically exits
//   alarm(60);
   //////////////////////////////////////////////
   // enable flush-to-zero and denormals to avoid performance penalty
   //    on many cpus
   flush_to_zero();
   // create producer objects and threads via script input
   // all created threads will block once initialized
   if (configure_environment(argv[1]) != 0) {
      log_err(get_kernel_log(), 
            "Failed to configure environment. Shutting down");
      goto shutdown;
   }
   //////////////////////////////////////
   // establish interface for external control (via socket)
   run_postmaster();
   // run tests to catch setup errors
   sanity_check();
   // let loose the hounds
   // procedure returns when all processor threads have exited
   run_all_processors();
   // wait for postmaster to quit
shutdown:
   printf("Shutting down");
   signal_exit(0);
   shutdown_postmaster();
   // free global resources
   clean_up_globals();
   // pipe error log to stdout
   report_errors();
   // 
   return 0;
}

