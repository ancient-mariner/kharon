include ../../../util/set_env_base.make
include ../../../util/core_set_env.make

CORE_LIBS = $(ROOT)local/lib/libpinet.a
EXT_LIBS = $(ROOT)ext/lib/liblua.a 

LIB = $(CORE_LIBS) $(EXT_LIBS) -lm -ldl -lpthread

OBJS = vy_receiver.o 
HDRS = $(ROOT)brain/include/modules/vy_receiver.h


all: $(OBJS)

%.o: %.c $(HDRS)
	$(CC) $< -c -o $@ $(CFLAGS)

refresh: clean all

clean:
	rm -f *.o

