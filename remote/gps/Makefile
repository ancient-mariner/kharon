include ../../util/set_env_base.make
include ../../util/remote_set_env.make

LIB =  -ldl -lm $(LOCAL_LIB) -lpthread 

all: gps install

gps: gps.c 
	./buildnum.py
	$(CC) $(CFLAGS) gps.c -o gps $(LIB)

install: $(cpu)

INTEL:

ARM:
	cp gps $(BIN_DIR)

refresh: clean all

clean:
	rm -f *.o gps

