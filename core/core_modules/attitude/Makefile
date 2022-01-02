include ../../../util/set_env_base.make
include ../../../util/core_set_env.make

OBJS = attitude.o get_attitude.o
HDRS = $(ROOT)core/include/core_modules/attitude.h 

all: $(OBJS)

%.o: %.c $(HDRS)
	$(CC) $< -c -o $@ $(CFLAGS)

refresh: clean all

clean:
	rm -f *.o

