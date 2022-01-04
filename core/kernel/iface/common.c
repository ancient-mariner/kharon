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
#if !defined(COMMON_C)
#define COMMON_C
#include "script_iface.h"
#include "routing/driver.h"
#include "dev_info.h"

struct datap_desc * find_source(const char *str)
{
   struct datap_desc * prod = NULL;
   for (uint32_t i=0; i<num_threads_g; i++)
   {
      if (strcmp(str, thread_table_g[i].obj_name) == 0)
      {
         prod = thread_table_g[i].dp;
         break;
      }
   }
   if (prod == NULL)
   {
      fprintf(stderr, "Unable to locate source with name '%s'\n", str);
      errs_++;
   }
   return prod;
}

const char * get_string(lua_State *L, const char *func, int32_t param)
{
   const char * str = lua_tostring(L, param);
   if (!str)
   {
      fprintf(stderr, "NULL string in '%s', param %d\n", func, param);
      errs_++;
   }
   return str;
}

// reads from an int-filled array (table) at stack index idx
// returns the array of values. the array length is stored in *count
const int32_t * get_int_array(
      /* in out */       lua_State *L,
      /* in     */ const char *func,
      /* in     */ const int32_t idx,
      /*    out */       int32_t *count
      )
{
   *count = 0;
   if (lua_istable(L, idx) != 1) {
      fprintf(stderr, "Value supplied to %s was not a table\n", func);
      return NULL;
   }
   size_t len = lua_rawlen(L, idx);
   int32_t *array = malloc(len * sizeof *array);
   for (uint32_t i=0; i<len; i++) {
      lua_rawgeti(L, idx, i+1);
      lua_Integer val = lua_tointeger(L, -1);
      lua_pop(L, 1);
      array[i] = (int32_t) val;
   }
   *count = (int32_t) len;
   return array;
}


// out-of-place documentation
// to see CPU load per thread, run 'top -H -p <pid>'
// TODO figure out how to map pthread ID to system process ID
void launch_thread(const char *name, const char *class,
      void *(*cb)(void*), void *arg)
{
   uint32_t num_threads = num_threads_g;
   if (num_threads >= MAX_PROCESSORS) {
      fprintf(stderr, "Maximum number of data processors produced\n");
      fprintf(stderr, "Modify MAX_PROCESSORS and recompile to increase\n");
      errs_++;
   }
   // store object and class name
   size_t len1 = strlen(name);
   size_t len2 = strlen(class);
   if ((len1 >= MAX_NAME_LEN) || (len2 >= MAX_NAME_LEN)) {
      fprintf(stderr, "Name too long for %s (%s). "
            "Max of %d characters allowed\n", name, class, (MAX_NAME_LEN-1));
      errs_++;
   }
   strncpy(thread_table_g[num_threads].obj_name, name, MAX_NAME_LEN);
   strncpy(thread_table_g[num_threads].class_name, class, MAX_NAME_LEN);
   // create thread
   pthread_create(&thread_table_g[num_threads].thread_id, NULL, cb, arg);
   // sleep and poll until thread creation complete
   // if it takes over a second, signal an error (something is very likely
   //    wrong)
   uint32_t cnt = 0;
   while (num_threads == num_threads_g) {
      if (cnt++ > 2000) {
         fprintf(stderr, "Thread failed to register launch within "
               "timeout window\n");
         fprintf(stderr, "Offending thread: %s (%s)\n",
               thread_table_g[num_threads].obj_name,
               thread_table_g[num_threads].class_name);
         errs_++;
         break;
      }
      usleep(2000);
   }
}

// returns 1 if logging requested, 0 if not
// logging requested if string is "log"
// no logging requested if string is "no-log"
static uint32_t determine_color_state(
      /* in     */ const char *arg
      )
{
   if (strcmp(arg, "gray") == 0) {
      return 0;
   } else if (strcmp(arg, "grey") == 0) {
      return 0;
   } else if (strcmp(arg, "color") == 0) {
      return 1;
   }
   // else...
   log_err(get_kernel_log(), "Unrecognized color indicator. "
         "Need 'gray' or 'color'. Got '%s'", arg);
   errs_++;
   return 0;
}

// returns 1 if logging requested, 0 if not
// logging requested if string is "log"
// no logging requested if string is "no-log"
uint32_t determine_logging_state(
      /* in     */ const char *arg
      )
{
   if (strcmp(arg, "log") == 0) {
      return 1;
   } else if (strcmp(arg, "no-log") == 0) {
      return 0;
   }
   // else...
   log_err(get_kernel_log(), "Unrecognized logging indicator. "
         "Need 'log' or 'no-log'. Got '%s'", arg);
   errs_++;
   return 0;
}

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

static int32_t create_udp_sync(lua_State *L)
{
   int32_t argc = lua_gettop(L);
   if (argc != 1)
   {
      fprintf(stderr, "Lua syntax error\n");
      fprintf(stderr, "%s requires 1 argument\n", __func__);
      fprintf(stderr, "arg1 is synchronizer name (e.g., 'udp_sync'\n");
      fprintf(stderr, "encountered: %s(", __func__);
      for (int32_t i=1; i<=argc; i++)
         fprintf(stderr, "%s%s", lua_tostring(L, i), i==argc?"":", ");
      fprintf(stderr, ")\n");
      errs_++;
      return 1;
   }
   const char * str1 = get_string(L, __func__, 1);
   launch_thread(str1, UDP_SYNC_CLASS_NAME, udp_sync_class_init, NULL);
   return 0;
}

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

static int32_t set_logging_level(lua_State *L)
{
   int32_t argc = lua_gettop(L);
   if (argc != 1)
   {
      fprintf(stderr, "Lua syntax error\n");
      fprintf(stderr, "%s requires 1 argument\n", __func__);
      fprintf(stderr, "arg1 is logging level:\n");
      fprintf(stderr, "    ALL   %d\n", LOG_LEVEL_ALL);
      fprintf(stderr, "    INFO  %d\n", LOG_LEVEL_INFO);
      fprintf(stderr, "    WARN  %d\n", LOG_LEVEL_WARN);
      fprintf(stderr, "    ERR   %d\n", LOG_LEVEL_ERR);
      fprintf(stderr, "    NONE  %d\n", LOG_LEVEL_NONE);
      fprintf(stderr, "encountered: %s(", __func__);
      for (int32_t i=1; i<=argc; i++)
         fprintf(stderr, "%s%s", lua_tostring(L, i), i==argc?"":", ");
      fprintf(stderr, ")\n");
      errs_++;
      return 1;
   }
   const char * str1 = get_string(L, __func__, 1);
   int32_t level = atoi(str1);
   set_default_log_level(level);
   return 0;
}

static int32_t set_environment(lua_State *L)
{
   int32_t argc = lua_gettop(L);
   if (argc != 1)
   {
      fprintf(stderr, "Lua syntax error\n");
      fprintf(stderr, "%s requires 1 argument\n", __func__);
      fprintf(stderr, "arg1 is path of environment directory "
            "(e.g., '/opt/kharon/data/env/elan_2021may')\n");
      fprintf(stderr, "encountered: %s(", __func__);
      for (int32_t i=1; i<=argc; i++)
         fprintf(stderr, "%s%s", lua_tostring(L, i), i==argc?"":", ");
      fprintf(stderr, ")\n");
      errs_++;
      return 1;
   }
   const char * str1 = get_string(L, __func__, 1);
   set_environment_string(str1);
   return 0;
}

static int32_t set_world_map_folder(lua_State *L)
{
   int32_t argc = lua_gettop(L);
   if (argc != 1)
   {
      fprintf(stderr, "Lua syntax error\n");
      fprintf(stderr, "%s requires 1 argument\n", __func__);
      fprintf(stderr, "arg1 is path of directory storing world map "
            "(e.g., '/opt/kharon/mapping')\n");
      fprintf(stderr, "encountered: %s(", __func__);
      for (int32_t i=1; i<=argc; i++)
         fprintf(stderr, "%s%s", lua_tostring(L, i), i==argc?"":", ");
      fprintf(stderr, ")\n");
      errs_++;
      return 1;
   }
   const char * str1 = get_string(L, __func__, 1);
   set_world_map_folder_name(str1);
   return 0;
}

static int32_t set_device_dir(lua_State *L)
{
   int32_t argc = lua_gettop(L);
   if (argc != 1)
   {
      fprintf(stderr, "Lua syntax error\n");
      fprintf(stderr, "%s requires 1 argument\n", __func__);
      fprintf(stderr, "arg1 is path to device directory "
            "(e.g., '/opt/kharon/data/dev/')\n");
      fprintf(stderr, "encountered: %s(", __func__);
      for (int32_t i=1; i<=argc; i++)
         fprintf(stderr, "%s%s", lua_tostring(L, i), i==argc?"":", ");
      fprintf(stderr, ")\n");
      errs_++;
      return 1;
   }
   const char * str1 = get_string(L, __func__, 1);
   set_device_dir_path(str1);
   return 0;
}


static int32_t set_pixels_per_degree(lua_State *L)
{
   int32_t argc = lua_gettop(L);
   if (argc != 1)
   {
      fprintf(stderr, "Lua syntax error\n");
      fprintf(stderr, "%s requires 1 arguments\n", __func__);
      fprintf(stderr, "arg1 is pixels per degree (float)\n");
      fprintf(stderr, "encountered: %s(", __func__);
      for (int32_t i=1; i<=argc; i++)
         fprintf(stderr, "%s%s", lua_tostring(L, i), i==argc?"":", ");
      fprintf(stderr, ")\n");
      errs_++;
      return 1;
   }
   const char * str1 = get_string(L, __func__, 1);
   set_ppd(atof(str1));
   return 0;
}

static int32_t verify_pyramid_levels(lua_State *L)
{
   int32_t argc = lua_gettop(L);
   if (argc != 1)
   {
      fprintf(stderr, "Lua syntax error\n");
      fprintf(stderr, "%s requires 1 arguments\n", __func__);
      fprintf(stderr, "arg1 is number of pyramid levels (int)\n");
      fprintf(stderr, "encountered: %s(", __func__);
      for (int32_t i=1; i<=argc; i++)
         fprintf(stderr, "%s%s", lua_tostring(L, i), i==argc?"":", ");
      fprintf(stderr, ")\n");
      errs_++;
      return 1;
   }
   const char * str1 = get_string(L, __func__, 1);
   const int num_levels = atoi(str1);
   if (num_levels != NUM_PYRAMID_LEVELS) {
      fprintf(stderr, "Config file calls for %d pyramid levels while core "
            "was compiled for %d. Fatal error\n", num_levels,
            NUM_PYRAMID_LEVELS);
      errs_++;
      return 1;
   }
   return 0;
}

static int32_t set_view_above_horizon(lua_State *L)
{
   int32_t argc = lua_gettop(L);
   if (argc != 1)
   {
      fprintf(stderr, "Lua syntax error\n");
      fprintf(stderr, "%s requires 1 arguments\n", __func__);
      fprintf(stderr, "arg1 is degrees visible above horizon\n");
      fprintf(stderr, "encountered: %s(", __func__);
      for (int32_t i=1; i<=argc; i++)
         fprintf(stderr, "%s%s", lua_tostring(L, i), i==argc?"":", ");
      fprintf(stderr, ")\n");
      errs_++;
      return 1;
   }
   const char * str1 = get_string(L, __func__, 1);
   set_world_height(atof(str1), WORLD_HEIGHT_BELOW_HORIZ_DEGS);
   return 0;
}

static int32_t set_view_below_horizon(lua_State *L)
{
   int32_t argc = lua_gettop(L);
   if (argc != 1)
   {
      fprintf(stderr, "Lua syntax error\n");
      fprintf(stderr, "%s requires 1 arguments\n", __func__);
      fprintf(stderr, "arg1 is degrees visible below horizon\n");
      fprintf(stderr, "encountered: %s(", __func__);
      for (int32_t i=1; i<=argc; i++)
         fprintf(stderr, "%s%s", lua_tostring(L, i), i==argc?"":", ");
      fprintf(stderr, ")\n");
      errs_++;
      return 1;
   }
   const char * str1 = get_string(L, __func__, 1);
   set_world_height(WORLD_HEIGHT_ABOVE_HORIZ_DEGS, atof(str1));
   return 0;
}

static int32_t set_cruise_speed_kts(lua_State *L)
{
   int32_t argc = lua_gettop(L);
   if (argc != 1)
   {
      fprintf(stderr, "Lua syntax error\n");
      fprintf(stderr, "%s requires 1 arguments\n", __func__);
      fprintf(stderr, "arg1 is standard cruise speed in kts\n");
      fprintf(stderr, "encountered: %s(", __func__);
      for (int32_t i=1; i<=argc; i++)
         fprintf(stderr, "%s%s", lua_tostring(L, i), i==argc?"":", ");
      fprintf(stderr, ")\n");
      errs_++;
      return 1;
   }
   const char * str1 = get_string(L, __func__, 1);
   set_default_cruise_speed_kts(atof(str1));
   return 0;
}


////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////


// for Lua interface, allow variable args, with argv[>1] subscribing to
//   argv[1]
static int32_t link_b_to_a(lua_State *L)
{
   int32_t argc = lua_gettop(L);
   if (argc < 2)
   {
      fprintf(stderr, "Lua syntax error\n");
      fprintf(stderr, "link_b_to_a requires at least 2 arguments\n");
      fprintf(stderr, "arg0 is 'producer' name, argN is/are processes "
            "provided pointers to it\n");
      fprintf(stderr, "encountered: link_b_to_a(");
      for (int32_t i=1; i<=argc; i++)
         fprintf(stderr, "%s%s", lua_tostring(L, i), i==argc?"":", ");
      fprintf(stderr, ")\n");
      errs_++;
      return 1;
   }
   const char * prod_name = get_string(L, "link_b_to_a", 1);
   struct datap_desc * prod = find_source(prod_name);
   for (int32_t i=2; i<=argc; i++)
   {
      const char * cons_name = get_string(L, "link_b_to_a", i);
      struct datap_desc * cons = find_source(cons_name);
      if (!cons || !prod)
      {
         fprintf(stderr, "Error in %s: %s and/or %s is not defined\n",
               __func__, prod_name, cons_name);
         errs_++;
         return 1;
      }
      if (cons == prod)
      {
         fprintf(stderr, "Error - attempted to link object "
               "to itself\n");
         fprintf(stderr, "Linkin '%s' to '%s'\n",
               cons->td->obj_name, prod->td->obj_name);
         errs_++;
         return 1;
      }
      if (cons->add_link == NULL) {
         fprintf(stderr, "Attempted to provide link to module not "
               "expecting one\n");
         fprintf(stderr, "Linking '%s' to '%s'\n",
               cons->td->obj_name, prod->td->obj_name);
         fprintf(stderr, "The former does not have add_link defined\n");
         errs_++;
         return 1;
      }
      cons->add_link(cons, prod);
      log_info(get_kernel_log(), "Linked %s to %s", cons_name, prod_name);
   }
   return 0;
}


// for Lua interface, allow variable args, with argv[>1] subscribing to
//   argv[1]
static int32_t subscribe_b_to_a(lua_State *L)
{
   int32_t argc = lua_gettop(L);
   if (argc < 2)
   {
      fprintf(stderr, "Lua syntax error\n");
      fprintf(stderr, "subscribe_b_to_a requires at least 2 arguments\n");
      fprintf(stderr, "arg0 is producer name, argN is/are subscribing "
            "consumers\n");
      fprintf(stderr, "encountered: subscribe_b_to_a(");
      for (int32_t i=1; i<=argc; i++)
         fprintf(stderr, "%s%s", lua_tostring(L, i), i==argc?"":", ");
      fprintf(stderr, ")\n");
      errs_++;
      return 1;
   }
   const char * prod_name = get_string(L, "subscribe_b_to_a", 1);
   struct datap_desc * prod = find_source(prod_name);
   for (int32_t i=2; i<=argc; i++)
   {
      const char * cons_name = get_string(L, "subscribe_b_to_a", i);
      struct datap_desc * cons = find_source(cons_name);
      if (!cons || !prod)
      {
         fprintf(stderr, "Error in %s: %s and/or %s is not defined\n",
               __func__, prod_name, cons_name);
         errs_++;
         return 1;
      }
      if (cons == prod)
      {
         fprintf(stderr, "Error - attempted to subscribe object "
               "to itself\n");
         fprintf(stderr, "Subscribing '%s' to '%s'\n",
               cons->td->obj_name, prod->td->obj_name);
         errs_++;
         errs_++;
         return 1;
      }
      if (prod->add_consumer == NULL) {
         fprintf(stderr, "Attempted to subscribe to a non-producer\n");
         fprintf(stderr, "Subscribing '%s' to '%s'\n",
               cons->td->obj_name, prod->td->obj_name);
         fprintf(stderr, "The latter does not have add_consumer defined\n");
         errs_++;
         errs_++;
         return 1;
      }
      if (cons->add_producer == NULL) {
         fprintf(stderr, "Attempted to subscribe a non-consumer\n");
         fprintf(stderr, "Subscribing '%s' to '%s'\n",
               cons->td->obj_name, prod->td->obj_name);
         fprintf(stderr, "The former does not have add_producer defined\n");
         errs_++;
         return 1;
      }
      cons->add_producer(cons, prod);
      prod->add_consumer(prod, cons);
      log_info(get_kernel_log(), "Subscribed %s to %s", cons_name, prod_name);
   }
   return 0;
}

#endif   // COMMON_C

