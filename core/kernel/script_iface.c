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
#include "pin_types.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>
#include "lauxlib.h"
#include "lualib.h"
#include "logger.h"
#include "script_iface.h"
#include "kernel.h"
#include "datap.h"
#include "udp_sync.h"

static uint32_t errs_ = 0;

#if !defined(STR_LEN)
#define STR_LEN  256u
#endif   // STR_LEN

#include "iface/common.c"
#include "iface/sensor.c"
#include "iface/routing.c"

// on start:
//    init globals
//    exec script
//    free globals
//
// on thread creation
//    copy class and obj into next thread_table_g slot
//    get number of threads
//    create thread, using class name to select init function
//    sleep-poll-repeat until thread counter incremented
//
// when all processors connected
//    call run_all_processors()
//    on procedure return, call clean_up_globals()


// run lua scripts to create and customize producers and consumers,
//   and link them together
int32_t configure_environment(const char* config_file)
{
   int32_t rc = -1;
   // make backup copy of config file
   copy_file_to_logging_dir(config_file, "config.lua");
   // create lua instance and initialize environment
   lua_State *L = NULL;
   L = luaL_newstate();
   luaL_openlibs(L);
   ///////////////////////////////////////////////////////////////////
   // register C API
   // general functions
   // subscribe a consumer to a producer
   lua_register(L, "subscribe_b_to_a", subscribe_b_to_a);
   lua_register(L, "link_b_to_a", link_b_to_a);
   /////////////////////////////////////////////
   // object creation
   /////////////////////
   // core modules (sensor)
   lua_register(L, "create_attitude", create_attitude);
   lua_register(L, "create_frame_sync", create_frame_sync);
   lua_register(L, "create_gps_receiver", create_gps_receiver);
   lua_register(L, "create_imu_receiver", create_imu_receiver);
   lua_register(L, "create_optical_up", create_optical_up);
   lua_register(L, "create_panorama", create_panorama);
   lua_register(L, "create_vy_receiver", create_vy_receiver);
   /////////////////////
   // tracking
#if defined(USE_TRACKING)
   errs_ += init_tracking_interface(L);
#endif // USE_TRACKING
   /////////////////////
   // routing
   lua_register(L, "create_beeper", create_beeper);
   lua_register(L, "create_driver", create_driver);
   lua_register(L, "set_destination", set_destination_);
   /////////////////////
   // other
   lua_register(L, "create_udp_sync", create_udp_sync);
   /////////////////////////////////////////////////////////////////////
   // IMU
   lua_register(L, "set_imu_priority", set_imu_priority_);
   // panorama
   //lua_register(L, "define_phantom_image", define_phantom_image);
   /////////////////////////////////////////////////////////////////////
   //
   lua_register(L, "set_environment", set_environment);
   //
   lua_register(L, "set_device_dir", set_device_dir);
   //
   lua_register(L, "set_world_map_folder", set_world_map_folder);
   //
   lua_register(L, "set_logging_level", set_logging_level);
   //
   lua_register(L, "set_pixels_per_degree", set_pixels_per_degree);
   //
   lua_register(L, "set_view_above_horizon", set_view_above_horizon);
   //
   lua_register(L, "set_view_below_horizon", set_view_below_horizon);
   //
   lua_register(L, "set_cruise_speed_kts", set_cruise_speed_kts);
   //
   lua_register(L, "verify_pyramid_levels", verify_pyramid_levels);
   //
   lua_register(L, "set_autotracking", set_autotracking_);
   /////////////////////////////////////////////////////////////////////
   // copy config file to log folder

   // run general configuration file
   printf("Loading config file %s\n", config_file);
   if (luaL_loadfile(L, config_file) != 0)
   {
      char str[512];
      snprintf(str, 512, "%sluac -p %s", LUA_PATH, config_file);
      system(str);
      fprintf(stderr, "Error loading '%s'\n", config_file);
      goto done;
   }
   if (lua_pcall(L, 0, LUA_MULTRET, 0) != 0)
   {
      fprintf(stderr, "Error running '%s'\n", config_file);
      fprintf(stderr, "%s\n", lua_tostring(L, -1));
      goto done;
   }
   if (errs_ > 0) {
      fprintf(stderr, "Encountered %d errors executing lua file\n", errs_);
      goto done;
   }
   // done reading files - close lua environment
   rc = 0;
done:
   lua_close(L);
   return rc;
}

