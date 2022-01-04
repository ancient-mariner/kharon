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
#if !defined(_GNU_SOURCE)
#define _GNU_SOURCE
#endif   // _GNU_SOURCE
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <assert.h>
#include <stdlib.h>
#include <time.h>
#include <errno.h>
#include <unistd.h>
#include <linux/reboot.h>
int reboot(int);
#include <signal.h>
#include <proc/readproc.h>
#include <dirent.h>
#include "build_version_super.h"

// supervisor process for device drivers and data streaming
// launches process and re-launches them if they fail
//
// log data written to /tmp. in event that file system fills up
//    from too much log data, a reboot should solve the problem
//
// supervisor built on assumption there's at most one sensor of a given
//    type attached to the node (eg, one IMU and/or one camera and/or
//    one GPS, etc)

// limit number of times the supervisor restarts drivers. if restart
//    count exceeds this, something is probably wrong. have supervisor
//    exit and trigger hypervisor to power-cycle node. in the meantime
//    just reboot node
// NOTE: when testing, this will result in nodes rebooting every few
//    hours. to prevent a reboot on a node that's being worked on,
//    run 'sudo /etc/init.d/pi_super.sh stop'.
//    run 'start' to put it back in service, or 'make launch' from w/in
//    this directory
#define MAX_RESTARTS    2500
static int restarts_ = 0;

// when mode=1, processes are launched when it's detected they are
//    absent. when mode=0, launching is inhibited
static int launch_mode_ = 1;

// interval, in seconds, to check to see if watched processes are running
#if !defined(POLL_INTERVAL)
#define POLL_INTERVAL   5
#endif   // POLL_INTERVAL

// maximum length of file system path
#define MAX_PATH_LEN    256
// max length of name (eg, process name)
#define MAX_NAME_LEN    64
// one more than max size of argc
#define ARGV_MAX_LEN    8
// max length of command line argumetns
#define MAX_CMD_LEN     1024
// max number of processes being managed by supervisor
#define MAX_WATCHED_PROCS  16

// use signals to turn supervisor on/off
// SIGUSR1 turns supervisor on (default mode)
// SIGUSR2 turns super off, preventing child relaunching

FILE *log_ = NULL;

static double system_now(void)
{
   struct timespec ts;
   clock_gettime(CLOCK_MONOTONIC, &ts);
   return (double) ts.tv_sec + 1.0e-9 * ((double) (ts.tv_nsec));
}

// fatal error -- close logs and exit, but don't reboot
static void fatal_error(
      /* in     */ const char *fn,
      /* in     */ const int line
      )
{
   if (log_) {
      fprintf(log_, "%.3f Fatal error in %s at line %d\n",
            system_now(), fn, line);
      fclose(log_);
   }
   exit(1);
}

static void signal_handler(int signal_number)
{
   if (signal_number == SIGUSR1) {
      launch_mode_ = 1;
   } else if (signal_number == SIGUSR2) {
      launch_mode_ = 0;
   } else if (signal_number == SIGTERM) {
      fprintf(log_, "%.3f Caught SIGTERM -- exiting\n", system_now());
      fclose(log_);
      exit(0);
   } else {
      fprintf(log_, "%.3f Unrecognized signal in supervisor: %d\n",
            system_now(), signal_number);
   }
}

// name of process and command line to execute it
// name must match argv[0] with path info removed. this is to ensure
//    that no two processes use the same executible (if they did, the
//    logic here to make sure something is running would break)
// TODO add sanity check to make sure this is the case
struct process_description {
   char dev_path[MAX_PATH_LEN];  // location of definition in dev/
   char name[MAX_NAME_LEN];
   char *argv[ARGV_MAX_LEN];
};
typedef struct process_description process_description_type;


// static variables
static process_description_type  procs_[MAX_WATCHED_PROCS];
static char host_[HOST_NAME_MAX];
static uint32_t num_procs_;


// initialize memory and variables
static void init_static_storage(void)
{
#if defined(ARM)
   if (gethostname(host_, HOST_NAME_MAX) < 0) {
      int err = errno;
      fprintf(log_, "%.3f Unable to determine hostname (%s)\n",
            system_now(), strerror(err));
      fatal_error(__func__, __LINE__);
   }
#else
   fprintf(log_, "%.3f pi_super test mode on INTEL\n", system_now());
   strcpy(host_, "pi1");
#endif   // ARM
   // read contents from <hostname>/super/
   // each file is the name of the process to run
   // inside each file is the command to run, including command line args
   //
   // set storage area to NULL
   memset(procs_, 0, sizeof procs_);
   //
   num_procs_ = 0;
}


// turn args to tokens
static void set_argv(
      /* in out */       process_description_type *desc,
      /* in out */       char *cmd
      )
      //char *path, char *args, char **argv)
{
   // parse by white space
   char * tok;
   uint32_t argc = 0;
   // foreach token in 'cmd', parse by white-space and copy tok to argv
   tok = strtok(cmd, " \t\n\r");
   while (tok != NULL) {
      size_t len = strlen(tok);
      desc->argv[argc] = malloc(len + 1);
      strcpy(desc->argv[argc++], tok);
      tok = strtok(NULL, " \t\n\r");
   }
   // make sure we didn't fill up argv
   if (desc->argv[ARGV_MAX_LEN-1] != NULL) {
      fprintf(log_, "%.3f Too many command arguments encountered for "
            "process '%s'\n", system_now(), desc->name);
      fprintf(log_, "%.3f Modify %s and recompile to change this\n",
            system_now(), __FILE__);
      fatal_error(__func__, __LINE__);
   }
   // make sure there's something in argv!
   if (desc->argv[0] == NULL) {
      fprintf(log_, "%.3f Process file '%s' is empty!\nPlease specify "
            "command line in that file\n", system_now(), desc->name);
      fatal_error(__func__, __LINE__);
   }
}


// supervisor is the first to launch on a particular node
// before other apps launch, query dev info to see what's supposed
//    to be running on this node
static uint32_t load_system_info(void)
{
   uint32_t errs = 0;
   init_static_storage();
   // name of directory and file(s) where process info stored
   char proc_path[MAX_PATH_LEN];
   snprintf(proc_path, MAX_PATH_LEN, "/pinet/dev/%s/super/", host_);
   char cmd[MAX_CMD_LEN];
   // read contents of that directory
   DIR *d = NULL;
   struct dirent *dir = NULL;
   d = opendir(proc_path);
   if (d == NULL) {
      fprintf(log_, "%.3fError reading process directory '%s'\n",
            system_now(), proc_path);
      errs++;
      goto end;
   }
   // each file in the directory represents a process. read file contents
   //    and make process descriptor
   while ((dir = readdir(d)) != NULL) {
      if (num_procs_ >= MAX_WATCHED_PROCS) {
         fprintf(log_, "%.3f Too many processes specified for '%s'. "
               "Encountered %d. ", system_now(), host_, num_procs_);
         fprintf(log_, "If more processes are required, alter "
               "MAX_WATCHED_PROCS and recompile\n");
         errs++;
         goto end;
      }
      // ignore '.' and '..' as well as hidden files
      if (dir->d_name[0] == '.')
         continue;
      // load and store process info
      process_description_type *desc = &procs_[num_procs_];
      strncpy(desc->name, dir->d_name, MAX_NAME_LEN);
      // open file to get command line
      snprintf(desc->dev_path, MAX_PATH_LEN, "%s%s", proc_path, desc->name);
      FILE *fp = fopen(desc->dev_path, "r");
      if (fp == NULL) {
         fprintf(log_, "%.3f Unable to open process description '%s'\n",
               system_now(), desc->dev_path);
         fflush(log_);
         // treat as soft error -- this process won't be launched
         continue;
      }
      // read file contents into buffer
      cmd[MAX_CMD_LEN-1] = 0;
      int32_t cnt = (int32_t) fread(cmd, 1, MAX_CMD_LEN, fp);
      fclose(fp);
      if (cnt <= 0)  {
         fprintf(log_, "%.3f Failed to read content from %s\n",
               system_now(), desc->dev_path);
         fflush(log_);
         // treat as soft error -- this process won't be launched
         continue;
      }
      if (cmd[MAX_CMD_LEN-1] != 0) {
         fprintf(log_, "%.3f Command line for %s is too long. ",
               system_now(), desc->dev_path);
         fprintf(log_, "If a longer command lie is required, alter "
               "MAX_CMD_LEN and recompile\n");
         fflush(log_);
         // treat as soft error -- this process won't be launched
         continue;
      }
      // make sure cmd string is null terminated in correct location
      cmd[cnt<MAX_CMD_LEN ? cnt : MAX_CMD_LEN-1] = 0;
      //
      set_argv(desc, cmd);
      // log command string
      fprintf(log_, "%.3f Preparing command: '", system_now());
      for (uint32_t i=0; i<ARGV_MAX_LEN; i++) {
         char *s = desc->argv[i];
         if (s == NULL) {
            break;
         }
         fprintf(log_, "%s ", s);
      }
      fprintf(log_, "'\n");
      //
      num_procs_++;
   }
end:
   return errs;
}


////////////////////////////////////////////////////////////////////////
// process launching

static void launch_process(char **argv)
{
#if defined(ARM)
   fprintf(log_, "%.3f Launching %s (launch count: %d)\n", system_now(),
         argv[0], restarts_);
   pid_t pid = fork();
   if (pid == 0) {
      // child process
      // restart process with desired exe
      char *envp[] = { NULL };
      if (execve(argv[0], argv, envp) != 0) {
         // This log message is likely to be lost  FIXME
         fprintf(log_, "%.3f Child failed to launch '%s' (%s)\n",
               system_now(), argv[0], strerror(errno));
         fclose(log_);
         exit(1);
      }
   } else if (pid < 0) {
      fprintf(log_, "%.3f Forking error (%s)\n", system_now(),
            strerror(errno));
      fflush(log_);
   } else {
      fprintf(log_, "%.3f Child %s has PID %d\n", system_now(), argv[0], pid);
      fflush(log_);
   }
#else
   for (int i=0; i<ARGV_MAX_LEN; i++) {
      const char * arg = argv[i];
      if (!arg)
         break;
      printf("%s ", arg);
   }
   printf("\n");
#endif   // ARM
}

////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
   (void) argc;
   //
   char buf[256];
   sprintf(buf, "/tmp/log_supervisor_%d", getpid());
   log_ = fopen(buf, "w");
   fprintf(log_, "%.3f Starting supervisor %s %s\n", system_now(), argv[0],
         SUPER_BUILD_VERSION);
   struct timespec ts, rem;
   PROCTAB *proc;
   uint8_t alive[MAX_WATCHED_PROCS];
   // setup signal handling
   signal(SIGUSR1, signal_handler);
   signal(SIGUSR2, signal_handler);
   signal(SIGTERM, signal_handler);
   signal(SIGCHLD, SIG_IGN);  // let child die and not become zombie
   // put process in background
   daemon(1, 0);
   //
   int rc;
   if (load_system_info() != 0) {
      fprintf(log_, "%.3f Failed to load system info. Bailing out\n",
            system_now());
      fclose(log_);
      return -1;
   }
   assert(num_procs_ < MAX_WATCHED_PROCS);
   // poll every X seconds to see if desired processes are running
   // if not, (re)start them
   // after too many restarts, reboot system as something likely wrong
   while (restarts_ < MAX_RESTARTS) {
      // search /proc to see if expected driver daemons are running
      proc = openproc(PROC_FILLCOM | PROC_FILLSTAT | PROC_FILLSTATUS);
      proc_t proc_info;
      memset(&proc_info, 0, sizeof(proc_info));
      // keep track of which processes are alive
      memset(alive, 0, MAX_WATCHED_PROCS);
      while(readproc(proc, &proc_info) != NULL) {
         if (!proc_info.cmdline)
            continue;   // kernel thread -- skip it
         for (uint32_t i=0; i<num_procs_; i++) {
            process_description_type *desc = &procs_[i];
            if (strcmp(proc_info.cmdline[0], desc->argv[0]) == 0) {
               alive[i] = 1;
               break;
            }
         }
      }
      closeproc(proc);
      int num_restarts = 0;
      if (launch_mode_ == 1) {
         for (uint32_t i=0; i<num_procs_; i++) {
            if (alive[i] == 0) {
               process_description_type *desc = &procs_[i];
               launch_process(desc->argv);
               num_restarts++;
            }
         }
      }
      if (num_restarts > 0) {
         restarts_++;
      }
#if defined(ARM)
      // check forced-exit indicator
      // look for presence of file to indicate process should exit
      if (access("/pinet/super_exit.txt", F_OK) == 0) {
         fprintf(log_, "%.3f Found 'super_exit.txt' -- exiting\n",
               system_now());
         // avoid reboot if file is present -- system enters continuous
         //    reboot cycle otherwise (!)
         goto end;
      }
#else
      // for non-ARM, always exit
      printf("non-ARM platform so nothing to do -- exiting\n");
      goto end;
#endif   // ARM
      // wait before checking again
      ts.tv_sec = POLL_INTERVAL;
      ts.tv_nsec = 0;
      while ((rc = nanosleep(&ts, &rem)) != 0) {
         if (errno != EINTR) {
            int err = errno;
            fprintf(log_, "%.3f Error in nanosleep (%s)\n",
                  system_now(), strerror(err));
            goto restart;  // unexpected error -- try restarting system
         }
         ts.tv_sec = rem.tv_sec;
         ts.tv_nsec = rem.tv_nsec;
      }
   }
   fprintf(log_, "%.3f Exit main loop\n", system_now());
restart:
   // if we make it here (ie, break from loop, or via goto) then
   //    something bad has likely happened. restart the system
   fprintf(log_, "%.3f Restarting system\n", system_now());
   fclose(log_);
   log_ = NULL;
   sync();
#if defined(ARM)
   reboot(LINUX_REBOOT_CMD_RESTART);
#endif   // ARM
end:
   // exit process only
   if (log_) {
      fprintf(log_, "%.3f Forced exit \n", system_now());
      fclose(log_);
   }
   return 0;
}

