#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <pthread.h>
#include <pthread.h>
#include <assert.h>
#include <unistd.h>
#include <sys/syscall.h>
#include <bits/syscall.h>
#include "datap.h"
#include "logger.h"

uint32_t num_threads_g = 0;

datap_desc_type *processor_list_g[MAX_PROCESSORS];
thread_desc_type thread_table_g[MAX_PROCESSORS];


// thread entry function provided by each data processor
// each entry function should call dp_execute() when it's done
//    initializing the processor
void dp_execute(
      /* in out */       datap_desc_type *dp
      )
{
   // thread ready to go -- store 
   thread_desc_type *td = &thread_table_g[num_threads_g];
   td->dp = dp;
   num_threads_g++;
   dp->run_state |= DP_STATE_INITIALIZED;
   // wait for all other threads to start
   // number of threads unknown during initialization, so barrier 
   //    cannot be initialized yet. use each thread's condition wait
   //    on a global mutex to unbounded achieve barrier
   // once all threads are ready, count is known so threads can be
   //    established
   pthread_mutex_lock(&global_mutex_g);
   pthread_cond_wait(&td->condition, &global_mutex_g);
   pthread_mutex_unlock(&global_mutex_g);
   //-------------------------------------------------------------------
   // central control created barrier after all threads created
   // release from condition wait is an effective barrier, so no
   //    need to call the barrier before pre_run()
   if (dp->pre_run) {
      dp->pre_run(dp);
   }
   pthread_barrier_wait(&barrier_g); // --------------------------------
   if (dp->run) {
      dp->run(dp);
   }
   pthread_barrier_wait(&barrier_g); // --------------------------------
   if (dp->post_run) {
      dp->post_run(dp);
   }
   pthread_barrier_wait(&barrier_g); // --------------------------------
//printf("dp_execute exit\n");
   // all other threads have finished
}

static void default_add_producer(
      /* in     */       datap_desc_type *self, 
      /* in     */       datap_desc_type *x
      )
{
   (void) x;
   log_err(get_kernel_log(), "Error: add_producer() not defined for %s (%s)",
         self->td->obj_name, self->td->class_name);
   hard_exit(__func__, __LINE__);
}

static void default_add_link(
      /* in     */       datap_desc_type *self, 
      /* in     */       datap_desc_type *x
      )
{
   (void) x;
   log_err(get_kernel_log(), "Error: add_link() not defined for %s (%s)",
         self->td->obj_name, self->td->class_name);
   hard_exit(__func__, __LINE__);
}

void default_add_consumer(
      datap_desc_type *self, 
      datap_desc_type *cons
      )
{
   if (self->num_attached_consumers >= MAX_ATTACHED_CONSUMERS) {
      log_err(get_kernel_log(), "Too many consumers attached to %s (%s)",
            self->td->obj_name, self->td->class_name);
      hard_exit(__func__, __LINE__);
   }
   uint32_t n = self->num_attached_consumers++;
   self->consumer_list[n] = cons;
}

void default_add_consumer_prohibited(
      /* in     */       datap_desc_type *self, 
      /* in     */       datap_desc_type *x
      )
{
   (void) x;
   log_err(get_kernel_log(), "Error: %s (%s) does not allow consumers",
         self->td->obj_name, self->td->class_name);
   hard_exit(__func__, __LINE__);
}

static void * default_get_object_at(
      /* in     */ const datap_desc_type *self, 
      /* in     */ const uint32_t idx
      )
{
   (void) idx;
   log_err(get_kernel_log(), "Error: get_object_at() not defined for %s (%s)",
         self->td->obj_name, self->td->class_name);
   hard_exit(__func__, __LINE__);
   return NULL;
}

static void default_reload_config(
      /* in     */       datap_desc_type *self
      )
{
   (void) self;
}


// allocate space for new datap structure, associate it with 
//    already defined thread description, initialize fields
//    and set default values
datap_desc_type * dp_create()
{
   datap_desc_type *dp = (datap_desc_type*) malloc(sizeof(*dp));
   memset(dp, 0, sizeof(*dp));
   // this object is created with single-threaded logic. num_threads
   //    is how many threads have been assigned so far, and it's safe
   //    to use this to access the already-defined thread descriptor
   //    and associate self with it
   thread_desc_type *td = &thread_table_g[num_threads_g];
   dp->run_state = DP_STATE_CREATED;
   // num threads is incremented after create() -- the present value is 
   //    is this thread's ID (index)
   dp->thread_desc_idx = (uint16_t) num_threads_g;
   dp->td = td;
   td->thread_id = pthread_self();
   pthread_cond_init(&td->condition, NULL);
   pthread_mutex_init(&td->mutex, NULL);
   // 
   dp->num_attached_consumers = 0;
   memset(dp->consumer_list, 0, 
         MAX_ATTACHED_CONSUMERS * sizeof(dp->consumer_list[0]));
   dp->num_attached_producers = 0;
   memset(dp->producer_list, 0, 
         MAX_ATTACHED_PRODUCERS * sizeof(dp->producer_list[0]));
   //
   dp->update_ctr = 0;
   dp->update_interval = 1;
   //
   dp->elements_produced = 0;
   dp->element_size = 0;
   dp->queue_length = 0;
   dp->void_queue = NULL;
   // set v-table to NULL
   dp->pre_run = NULL;
   dp->run = NULL;
   dp->post_run = NULL;
   dp->add_producer = default_add_producer;
   dp->add_consumer = default_add_consumer;
   // add_link is like add_producer but 'consumer' is only provided a pointer to
   //    the 'producer', such as to access a non-standard API. consumer is not
   //    alerted to changes in the producer and the producer doesn't generate
   //    any signal to alert linked processes
   // this is implemented on the 'consumer' side. producer knows nothing of it
   dp->add_link = default_add_link;
   //
   dp->abort = NULL;
   //
   dp->get_object_at = default_get_object_at;
   dp->reload_config = default_reload_config;
   //
   return dp;
}

void dp_signal_data_available(datap_desc_type *src)
{
   if (src->update_interval >= 0) {
      if (++src->update_ctr >= src->update_interval) {
         src->update_ctr = 0;
         for (uint32_t i=0; i<src->num_attached_consumers; i++)
            dp_wake(src->consumer_list[i]);
      }
   }
}

void dp_destroy(datap_desc_type *dp)
{
   pthread_mutex_destroy(&dp->td->mutex);
}

void dp_wait(datap_desc_type *dp)
{
   assert(dp->td->thread_id == pthread_self());
   pthread_cond_wait(&dp->td->condition, &dp->td->mutex);
   // if pending reload, call dp_reload_config(dp); clear reload flag
   if (dp->reload_flag != 0) {
      dp_reload_config(dp);
   }
}

void dp_quit(datap_desc_type *dp)
{
   dp->run_state |= DP_STATE_DONE;
}

void dp_wake(datap_desc_type *dp)
{
   pthread_cond_signal(&dp->td->condition);
}

void dp_abort(
      /* in out */       datap_desc_type *dp
      )
{
   if (dp->abort)
      (dp->abort)(dp);
}

void * dp_get_object_at(
      /* in     */ const datap_desc_type *dp,
      /* in     */ const uint32_t idx
      )
{
   if (c_assert(idx < dp->queue_length) != 0) {
      log_err(get_kernel_log(), "Error: exceeded array bounds in %s (%s)",
            dp->td->obj_name, dp->td->class_name);
      log_err(get_kernel_log(), "Accessed element %d of queue length %d\n",
            idx, dp->queue_length);
      hard_exit("dp_get_object_at", 1);
   }
   return (dp->get_object_at)(dp, idx);
}

void dp_reload_config(
      /* in out */       datap_desc_type *dp
      )
{
   (dp->reload_config)(dp);
   dp->reload_flag = 0;
}

object_time_type dp_get_object_and_time_at(
      /* in     */ const datap_desc_type *dp, 
      /* in     */ const uint32_t idx
      )
{
   object_time_type pair;
   if (c_assert(idx < dp->queue_length) != 0) {
      log_err(get_kernel_log(), "Error: exceeded array bounds in %s (%s)",
            dp->td->obj_name, dp->td->class_name);
      log_err(get_kernel_log(), "Accessed element %d of queue length %d\n",
            idx, dp->queue_length);
      hard_exit("dp_get_object_and_time_at", 1);
   }
   pair.obj = (dp->get_object_at)(dp, idx);
   pair.t = dp->ts[idx];
   return pair;
}


////////////////////////////////////////////////////////////////////////
// global (all-processor) requests

void request_config_reload(void)
{
   for (uint32_t i=0; i<num_threads_g; i++) {
      processor_list_g[i]->reload_flag = 1;
   }
}

////////////////////////////////////////////////////////////////////////
// 

void report_thread_id_by_name(
      /* in     */ const char *label,
      /* in out */       log_info_type *log
      )
{
   long tid = syscall(SYS_gettid);
   log_info(log, "Process %s is TID %ld", label, tid);
}


void report_thread_id(
      /* in out */       datap_desc_type *dp,
      /* in out */       log_info_type *log
      )
{
   report_thread_id_by_name(dp->td->obj_name, log);
}

