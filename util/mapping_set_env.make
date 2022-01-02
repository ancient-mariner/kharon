# set_env_base.make must be included before this file

# for a full compile OPT will be set. if we're compiling locally or
#     regionally then default to modest optimization, with debugging
ifndef OPT
#OPT = -ffast-math -O0 -g -gstabs
OPT = -ffast-math -O2 -g
TEST_OPT = -ffast-math -g
endif

CORE_INC = -I$(ROOT)local/include/  \
      -I$(ROOT)mapping/include/     \
      -I$(ROOT)core/include/

#LIB = $(LOCAL_LIB) -lm

CFLAGS = -std=gnu11 $(CORE_FLAGS) $(WARN) $(OPT) $(CORE_INC)
TEST_CFLAGS = -std=gnu11 $(CORE_FLAGS) $(TEST_WARN) $(OPT) $(CORE_INC) -g

################################
# libraries

MAPPING_LIB = $(ROOT)/mapping/lib/libmapping.a

MOD_CORE_LIB = $(ROOT)core/lib/libmod_core.a
MOD_TRACKING_LIB = $(ROOT)core/lib/libmod_tracking.a
MOD_ROUTING_LIB = $(ROOT)core/lib/libmod_routing.a

