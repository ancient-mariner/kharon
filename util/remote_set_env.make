# set_env_base.make must be included before this file

# for a full compile OPT will be set. if we're compiling locally or
#     regionally then default to modest optimization, with debugging
OPT = -ffast-math -O2 -g

INC = -I$(ROOT)core/include/external/ \
            -I$(ROOT)local/include/ 

CFLAGS = -std=gnu11 $(CORE_FLAGS) $(WARN) $(OPT) $(INC)
TEST_CFLAGS = -std=gnu11 $(CORE_FLAGS) $(TEST_WARN) $(OPT) $(INC) -g

SENS_I2C_LIB = libsensi2c.a
