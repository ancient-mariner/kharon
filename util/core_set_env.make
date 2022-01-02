# set_env_base.make must be included before this file

# for a full compile OPT will be set. if we're compiling locally or
#     regionally then default to modest optimization, with debugging
ifndef OPT
#OPT = -ffast-math -O0 -g
OPT = -ffast-math -O2 -g
TEST_OPT = -ffast-math -g
endif

# repo has tracking as optionally includable content, complicating
#  its organization and logic. all tracking content has been moved 
#  into core/tracking, including what used to be core/include/tracking,
#  so we need an additional -I flag if tracking enabled
# around build 376, tracking has been removed from this repo
ifeq ($(USE_TRACKING), 1)
   TRACKING_INCLUDE = -I$(ROOT)core/tracking/include/
endif

CORE_INC = -I$(ROOT)core/include/kernel/ \
      -I$(ROOT)core/include/ \
      -I$(LOCAL_INCLUDE)      \
      -I$(MAPPING_INCLUDE_DIR)   \
      $(TRACKING_INCLUDE)

CFLAGS = $(CORE_FLAGS) $(WARN) $(OPT) $(CORE_INC)
TEST_CFLAGS = $(CORE_FLAGS) $(TEST_WARN) $(TEST_OPT) $(CORE_INC) -g

################################
# libraries

MAPPING_LIB = $(ROOT)/mapping/lib/libmapping.a

MOD_CORE_LIB = $(ROOT)core/lib/libmod_core.a
MOD_ROUTING_LIB = $(ROOT)core/lib/libmod_routing.a

# these libs may not be present if tracking disabled, but permit definition
#  to keep make system happy and less complicated
MOD_TRACKING_LIB = $(ROOT)core/lib/libmod_tracking.a
MOD_OPT_TRACKING_LIB = $(ROOT)core/lib/libmod_opttracking.a



