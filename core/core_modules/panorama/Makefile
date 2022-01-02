include ../../../util/set_env_base.make
include ../../../util/core_set_env.make

OBJS = panorama.o

########################################################################
#

all: $(OBJS)

%.o: %.c $(HDRS)
	$(CC) $< -c -o $@ $(CFLAGS)

clean:
	rm -f *.o

refresh: clean all

