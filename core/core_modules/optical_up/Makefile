include ../../../util/set_env_base.make
include ../../../util/core_set_env.make

LIB_DIR = $(ROOT)local/lib/

LIB = -L$(LIB_DIR) -lm -lpthread -ldl $(LIB_DIR)liblocal.a

OBJS = optical_up.o


########################################################################
#

all: $(OBJS) testing


%.o: %.c $(HDRS)
	$(CC) $< -c -o $@ $(CFLAGS)


testing: test_accumulator


test_accumulator: accumulator.c accumulator.h
	$(CC) accumulator.c -o test_accumulator $(CFLAGS) $(LIB) -DTEST_ACCUMULATOR


clean:
	rm -f *.o test_* 

refresh: clean all

