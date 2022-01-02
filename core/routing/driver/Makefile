include ../../../util/set_env_base.make
include ../../../util/core_set_env.make

LIB = $(LOCAL_LIB) -lm -ldl -lpthread

OBJS = driver.o test_comm

all: clean $(OBJS)

test_comm: comm.c
	$(CC) comm.c -o test_comm $(CFLAGS) -DSIMULATE_DRIVER -lm -lpthread

%.o: %.c $(HDRS)
	$(CC) $< -c -o $@ $(CFLAGS)

refresh: all
	#cd tests; make all

clean:
	rm -f *.o $(OBJS)

