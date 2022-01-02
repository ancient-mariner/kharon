include ../../util/set_env_base.make
include ../../util/mapping_set_env.make

TARGETS = default_beacons list_beacons beacon_path view_beacon eval_beacon merge_bins

LIB = $(MOD_ROUTING_LIB) $(MAPPING_LIB) $(LOCAL_LIB) -lm


all: $(TARGETS)

list_beacons: list_beacons.c  
	$(CC) $(CFLAGS) -o list_beacons list_beacons.c $(LIB)

beacon_path: beacon_path.c  
	$(CC) $(CFLAGS) -o beacon_path beacon_path.c $(LIB)

eval_beacon: eval_beacon.c  
	$(CC) $(CFLAGS) -o eval_beacon eval_beacon.c $(LIB)

merge_bins: merge_bins.c  
	$(CC) $(CFLAGS) -o merge_bins merge_bins.c $(LIB)

view_beacon: view_beacon.c  
	$(CC) $(CFLAGS) -o view_beacon view_beacon.c $(LIB)
	
default_beacons: default_beacons.c  
	$(CC) -o default_beacons default_beacons.c $(CFLAGS) $(LIB) $(MAPPING_LIB)


clean:
	rm -f *.o $(TARGETS)

refresh: clean all

