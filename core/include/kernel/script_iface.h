#if !defined(SCRIPT_IFACE_H)
#define SCRIPT_IFACE_H
#include <stdint.h>
#include "lauxlib.h"
#include "lualib.h"

#define LUA_PATH "/pinet/ext/bin/"

// run lua scripts to create and customize producers and consumers, 
//   and link them together
int32_t configure_environment(const char* config);

////////////////////////////////////////////////////////////////////////
// utility functions used by external libs that are linked in (e.g.,
//    tracking)

// returns parameter 'param' on lua call stack. 'func' is name of
//    lua function call (this is used for reporting errors)
const char * get_string(lua_State *L, const char *func, int32_t param);

// input param is 'log' or 'no-log' parameter passed in to create module
//    function
// returns 1 if logging requested, 0 if not
// logging requested if string is "log"
// no logging requested if string is "no-log"
uint32_t determine_logging_state(
      /* in     */ const char *arg
      );

// launch thread for module with specified name
// out-of-place documentation
// to see CPU load per thread, run 'top -H -p <pid>'
// TODO figure out how to map pthread ID to system process ID
void launch_thread(const char *name, const char *class, 
      void *(*cb)(void*), void *arg);

// reads from an int-filled array (table) at stack index idx
// returns the array of values. the array length is stored in *count
const int32_t * get_int_array(
      /* in out */       lua_State *L,
      /* in     */ const char *func,
      /* in     */ const int32_t idx,
      /*    out */       int32_t *count
      );

// returns pointer data producer having specified name and NULL otherwise
struct datap_desc * find_source(const char *str);

#if defined(USE_TRACKING)
uint32_t init_tracking_interface(lua_State *L);
#endif  // USE_TRACKING

#endif	// SCRIPT_IFACE_H

