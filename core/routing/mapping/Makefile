include ../../../util/set_env_base.make
include ../../../util/core_set_env.make

INC = -I$(ROOT)mapping/include/

OBJS = world_map.o

all: clean $(OBJS)

%.o: %.c $(HDRS)
	$(CC) $< -c -o $@ $(TEST_CFLAGS) $(INC)

refresh: all

clean:
	rm -f *.o test_* view_gaze_out

