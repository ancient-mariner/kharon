include ../../util/set_env_base.make
include ../../util/core_set_env.make

OBJS = beeper/beeper.o 	\
		 driver/driver.o 		\
		 mapping/world_map.o        

all: subdirs
	rm -f $(MOD_ROUTING_LIB)
	gcc-ar rcsv $(MOD_ROUTING_LIB) $(OBJS)

subdirs:
	cd mapping; make
	cd beeper; make
	cd driver; make

refresh: clean all

clean:
	cd mapping; make clean
	cd beeper; make clean
	cd driver; make clean

