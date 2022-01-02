
########################################################################
# these values should be set through a configure script

ROOT = /opt/kharon/

# set to 1 if tracking is enabled and tracking library or source is available
USE_TRACKING = 1


########################################################################

ifeq ($(USE_TRACKING), 1)
	TRACKING_FLAGS = -DUSE_TRACKING
endif


# get architecture
arch = $(shell arch)

# set variables according to which architecture we're compiling on
# from JPL C-coding standards, with -Wtraditional removed, and 
#     -Wdouble-promotion & others added
# not using -pedantic as some gcc extensions are frequently used, 
#     such as #warning
ifeq ($(arch), armv7l)
	PLATFORM = RPI # pi3, stretch(?) 32-bit
	#march native crashes pi3 -- leave it blank
	MARCH = 
   WARN = -Wall -Wextra -Wshadow -Wpointer-arith -Wcast-qual \
          -Wcast-align -Wstrict-prototypes -Wmissing-prototypes \
          -Wconversion -Wdouble-promotion -Wlogical-op   \
          -Wunused-function -Wunused-parameter  
   TEST_WARN = -Wall -Wextra -Wshadow -Wpointer-arith -Wcast-qual \
          -Wcast-align -Wconversion -Wdouble-promotion -Wlogical-op
endif

ifeq ($(arch), aarch64)
	PLATFORM = RPI4    # pi4, buster 64-bit
	MARCH = -march=native
   WARN = -Wall -Wextra -Wshadow -Wpointer-arith -Wcast-qual \
          -Wcast-align -Wstrict-prototypes -Wmissing-prototypes \
          -Wconversion -Wdouble-promotion -Wlogical-op   \
          -Wunused-function -Wunused-parameter  \
          -Wnull-dereference -Wduplicated-cond 
   TEST_WARN = -Wall -Wextra -Wshadow -Wpointer-arith -Wcast-qual \
          -Wcast-align -Wconversion -Wdouble-promotion -Wlogical-op \
          -Wduplicated-cond -Wnull-dereference
endif

ifeq ($(arch), x86_64)
	PLATFORM = INTEL  # stretch (or ??)
	MARCH = -march=native
   WARN = -Wall -Wextra -Wshadow -Wpointer-arith -Wcast-qual \
          -Wcast-align -Wstrict-prototypes -Wmissing-prototypes \
          -Wconversion -Wdouble-promotion -Wlogical-op \
          -Wunused-function -Wunused-parameter \
          -Wnull-dereference -Wduplicated-cond 
   TEST_WARN = -Wall -Wextra -Wshadow -Wpointer-arith -Wcast-qual \
          -Wcast-align -Wconversion -Wdouble-promotion -Wlogical-op
endif

ifndef PLATFORM
# if PLATFORM is not set then makefile needs to be updated to recognize
#  arch output and set variables as appropriate
$(error Platform not detected. arch is '$(arch)'. please update set_env_base.make)
endif

LOCAL_LIB_DIR = $(ROOT)local/lib/
LOCAL_INCLUDE = $(ROOT)local/include/
LOCAL_LIB = $(ROOT)local/lib/liblocal.a
LOCAL_BIN_DIR = $(ROOT)local/bin/

MAPPING_LIB = $(ROOT)mapping/lib/libmapping.a
MAPPING_INCLUDE_DIR = $(ROOT)mapping/include/

EXTERNAL_LIB_DIR = $(ROOT)external/lib/
EXTERNAL_INCLUDE_DIR = $(ROOT)external/include/

# _GNU_SOURCE used for sincos and in udp_sync's use of sendto(), where
#     sockaddr_in is an acceptable parameter, which stores port number.
#     without _GNU_SOURCE, param 5 is struct sockaddr, which doesn't.
# TODO migrate away from sincos, and alter udp_sync to use standard approach
CORE_FLAGS = -D_GNU_SOURCE -D$(PLATFORM) -std=gnu11 $(MARCH) $(TRACKING_FLAGS)

CC = gcc

