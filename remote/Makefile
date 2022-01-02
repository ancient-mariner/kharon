include ../util/set_env_base.make
include ../util/remote_set_env.make

ifeq ($(PLATFORM), INTEL)
   CAM_TGT = camera_null
else
   CAM_TGT = warn_camera_pi3
endif

all: $(PLATFORM)

RPI: $(CAM_TGT)
	cd sensors; make
	cd pi_supervisor; make
	cd kill; make
	cd gps; make

INTEL: 
	cd kill; make

camera_null:


warn_camera_pi3:
	@echo "#### camera_vy is not being built. type 'make camera_pi3' to build it"

camera_pi3:
	touch /home/pi/dvl/userland-master/host_applications/linux/apps/raspicam/camera_stream.c
	cd /home/pi/dvl/userland-master; ./buildme; cp build/bin/camera_vy /pinet/local/bin_rpi/

clean_INTEL:
	cd kill; make clean

clean_RPI:
	cd sensors; make clean
	cd pi_supervisor; make clean
	cd kill; make clean
	cd gps; make clean

refresh: clean_$(PLATFORM) $(PLATFORM)

