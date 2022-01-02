include ../../../util/set_env_base.make
include ../../../util/core_set_env.make

LIB = $(CORE_LIBS) $(EXT_LIBS) -lm -ldl -lpthread

OBJS = beeper.o 
HDRS = $(ROOT)brain/include/routing/beeper.h


all: $(OBJS)

%.o: %.c $(HDRS)
	$(CC) $< -c -o $@ $(TEST_CFLAGS)

refresh: clean all

clean:
	rm -f *.o

