# set_env_base.make must be included before this file

# for a full compile OPT will be set. if we're compiling locally or
#     regionally then default to modest optimization, with debugging
ifndef OPT
OPT = -ffast-math -O2 -g
endif

CORE_INC = -I$(ROOT)local/include/ -I$(ROOT)external/include/

CFLAGS = -std=gnu11 $(CORE_FLAGS) $(WARN) $(OPT) $(CORE_INC)
TEST_CFLAGS = -std=gnu11 $(CORE_FLAGS) $(TEST_WARN) $(OPT) $(CORE_INC) -g

