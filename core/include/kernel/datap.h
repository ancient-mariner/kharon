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
﻿#if !defined(DATAP_H)
#define   DATAP_H
#if !defined(_GNU_SOURCE)
#define _GNU_SOURCE
#endif   // _GNU_SOURCE
#include <pthread.h>
#include "pinet.h"

void * cb_datap_launch(void *arg);

//// OLD: all producers generate and publish float array data
// producers generate and publish data of various types
// each can take input from one or more sources


struct producer_record {
   struct datap_desc *producer;
   uint64_t consumed_elements;
//   int32_t  element_size;
//   int32_t  queue_length;
};
typedef struct producer_record producer_record_type;

#define MAX_PROCESSORS   128
#define MAX_ATTACHED_PRODUCERS    8
#define MAX_ATTACHED_CONSUMERS   16
#define MAX_NAME_LEN   64

#define DP_STATE_CREATED      0x0001
#define DP_STATE_INITIALIZED  0x0002
#define DP_STATE_PAUSE        0x2000
#define DP_STATE_DONE         0x4000

struct thread_desc {
   pthread_t thread_id;
   pthread_cond_t    condition;
   pthread_mutex_t   mutex;
   // pointer to object descriptor
   struct datap_desc *dp;
   // name for thread and type of object that it controls
   char class_name[MAX_NAME_LEN];
   char obj_name[MAX_NAME_LEN];
};
typedef struct thread_desc thread_desc_type;

// to avoid race conditions, threads should be started sequentially,
//    with each start being delayed until the previous one is complete
// thread creation procedure:
//    get num threads
//    launch thread
//    wait until thread counter incremented (sleep-poll-repeat)
//    ready to do something with thread or create next
// each processor should have its own thread entry function and
//    call dp_execute() at end
extern uint32_t num_threads_g;
extern struct thread_desc thread_table_g[MAX_PROCESSORS];

extern pthread_barrier_t barrier_g;
extern pthread_mutex_t global_mutex_g;

void dp_execute(
      /* in out */       struct datap_desc *dp
      );

// data processor description structure
// this should be initialized/allocated using dp_create and freed
//    using dp_destroy
struct datap_desc {
   // list of connected producers and consumers
   // objects can maintain different lists for reading different
   //    data types
   struct datap_desc *consumer_list[MAX_ATTACHED_CONSUMERS];
   struct producer_record producer_list[MAX_ATTACHED_PRODUCERS];
   //
   // storage for pointer to 'subclass'-specific data
   void * local;
   //
   uint16_t num_attached_consumers;
   uint16_t num_attached_producers;
   // thread data
   uint16_t thread_desc_idx;
   struct thread_desc *td;
   // to access data in queue, use following approach.
   // reading from producer:
   //    uint32_t p_idx = pr->consumed_elements * producer->queue_length
   //    T *source = (T*) &producer->void_queue[p_idx * PRODUCER_ELEMENT_SIZE]
   //    double when = producer->ts[p_idx]
   // generating data:
   //    uint32_t idx = dp->elements_produced * dp->queue_length
   //    T *sink = (T*) &producer->void_queue[idx * SELF_ELEMENT_SIZE]
   //    dp->ts[p_idx] = when
   // use uint8_t* because pointer arithmetic isn't allowed on void*,
   //    and because all processes are responseible for managing their
   //    own byte alignment of data in queues
   uint8_t *void_queue; // prepend 'void' to remind that there's no type
   double *ts;
   //
   uint64_t elements_produced;
   uint32_t element_size;
   // TODO consider requiring that all modules have queue lengths that are
   //    powers of 2 so that modulus operation to get que pos can be changed
   //    to bit masking operation
   uint32_t queue_length;
   //
   int16_t run_state;
   // flag to signal reload of config
   // reload is done before exiting dp_wait()
   int16_t reload_flag;
   // control how often subscribers are notified that data is available.
   //   eg, notify when 1ms worth of data is ready
   // update counter counts how many samples since the last signal sent
   // notification sent when ++ctr >= interval, then ctr set to zero
   // TODO a negative counter means that this thread never alerts consumers
   int16_t   update_ctr;
   int16_t   update_interval;
   // v-table
   void (*pre_run)(struct datap_desc*);
   void (*run)(struct datap_desc*);
   void (*post_run)(struct datap_desc*);
   void (*add_link)(struct datap_desc* self, struct datap_desc *prod);
   void (*add_producer)(struct datap_desc* self, struct datap_desc *prod);
   void (*add_consumer)(struct datap_desc* self, struct datap_desc *cons);
   void (*abort)(struct datap_desc*);
   //
   void * (*get_object_at)(const struct datap_desc*, const uint32_t idx);
   void (*reload_config)(struct datap_desc*);
   //
};
typedef struct datap_desc datap_desc_type;

extern datap_desc_type *processor_list_[MAX_PROCESSORS];


// core constructor and destructor
datap_desc_type * dp_create(void);
void dp_destroy(datap_desc_type *dp);

void dp_signal_data_available(datap_desc_type *producer);

void dp_wait(datap_desc_type *dp);
void dp_wake(datap_desc_type *dp);
void dp_abort(datap_desc_type *dp);
void dp_quit(datap_desc_type *dp);

// returns pointer to object in data processor's output queue at
//    specified index, cast as void*
void *dp_get_object_at(
      /* in     */ const datap_desc_type *dp,
      /* in     */ const uint32_t idx
      );

// stores the pointer an object in the data processor's output queue,
//    cast as void*, and the corresponding time of that data sample
struct object_time {
   void *   obj;
   double   t;
};
typedef struct object_time object_time_type;

// returns pointer to object in data processor's output queue at
//    specified index, cast as void*, and the timestamp of that object
object_time_type dp_get_object_and_time_at(
      /* in     */ const datap_desc_type *dp,
      /* in     */ const uint32_t idx
      );

// reload config data for process
void dp_reload_config(
      /* in out */       datap_desc_type *dp
      );

// send message to all processors to reload their config data
void request_config_reload(void);

// handles adding a consumer to a module
// objects that over-ride this function for custom behaviors should
//    still call this default one unless there's a clear reason not
//    to, which should be documented
void default_add_consumer(
      datap_desc_type *self,
      datap_desc_type *cons
      );

// if consumers are not supported, this can be used
void default_add_consumer_prohibited(
      datap_desc_type *self,
      datap_desc_type *cons
      );


// TODO document this
void signal_exit(int x);

struct log_info;
typedef struct log_info log_info_type;

////////////////////////////////////////////////////////////////////////
// report thread ID to log file. to see CPU usage of each thread, use
//       top -H -p <PID>

// write thread's process ID to specified log file
void report_thread_id_by_name(
      /* in out */ const char *label,
      /* in out */       log_info_type *log
      );

// write thread's process ID to specified log file
void report_thread_id(
      /* in out */       datap_desc_type *dp,
      /* in out */       log_info_type *log
      );

#endif   //   DATAP_H

