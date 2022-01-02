include ../../util/set_env_base.make
include ../../util/remote_set_env.make

LIB =  -ldl -lm $(LOCAL_LIB) -lpthread 

all: kill_monitor send_kill install
	./buildnum.py

kill_monitor: kill_monitor.c 
	$(CC) $(CFLAGS) kill_monitor.c -o kill_monitor $(LIB)

send_kill: send_kill.c 
	$(CC) $(CFLAGS) send_kill.c -o send_kill $(LIB)

install: $(PLATFORM)

INTEL:
	cp send_kill $(LOCAL_BIN_DIR)

ARM:
	bash -c 'sudo killall -q kill_monitor' || echo kill_monitor not running 
	cp kill_monitor $(LOCAL_BIN_DIR)
	bash -c 'sudo $(LOCAL_BIN_DIR)kill_monitor' || echo Failed to start kill_monitor
	cp send_kill $(LOCAL_BIN_DIR)


refresh: clean all

clean:
	rm -f *.o kill_monitor send_kill log_*

