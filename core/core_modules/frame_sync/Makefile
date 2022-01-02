include ../../../util/set_env_base.make
include ../../../util/core_set_env.make

OBJS = frame_sync.o

########################################################################
#

all: $(OBJS)

%.o: %.c $(HDRS)
	$(CC) $< -c -o $@ $(CFLAGS)

clean:
	rm -f *.o test_* 

refresh: clean all

