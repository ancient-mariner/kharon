include ../../util/set_env_base.make
include ../../util/remote_set_env.make

LIB =  $(LOCAL_LIB) -ldl -lm -lprocps


all: pi_super install

only: pi_super 

pi_super: pi_super.c 
	./buildnum.py
	$(CC) $(CFLAGS) pi_super.c -o pi_super $(LIB)

install: $(cpu)

launch:
	bash -c 'sudo killall -q pi_super' || echo pi_super not running
	bash -c 'sudo $(BIN_DIR)pi_super' || echo Failed to start pi_super

INTEL:

ARM:
	bash -c 'sudo killall -q pi_super' || echo pi_super not running
	cp pi_super $(BIN_DIR)
	bash -c 'sudo $(BIN_DIR)pi_super' || echo Failed to start pi_super

refresh: clean all

clean:
	rm -f *.o pi_super

