include ../../util/set_env_base.make
include ../../util/core_set_env.make

OBJS =   attitude/attitude.o	\
         attitude/get_attitude.o	\
         gps_receiver/gps_receiver.o	\
         imu_receiver/imu_receiver.o \
         optical_up/optical_up.o \
         vy_receiver/vy_receiver.o \
         frame_sync/frame_sync.o \
         panorama/panorama.o 

all: subdirs
	rm -f $(MOD_CORE_LIB)
	gcc-ar rcsv $(MOD_CORE_LIB) $(OBJS)

subdirs:
	cd attitude; make
	cd gps_receiver; make
	cd imu_receiver; make
	cd optical_up; make
	cd vy_receiver; make
	cd frame_sync; make
	cd panorama; make

refresh: clean all

clean:
	cd attitude; make clean
	cd gps_receiver; make clean
	cd imu_receiver; make clean
	cd optical_up; make clean
	cd vy_receiver; make clean
	cd frame_sync; make clean
	cd panorama; make clean

