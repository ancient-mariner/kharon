include ../util/set_env_base.make

ifeq ($(USE_TRACKING), 1)
   TRACKING_ALL = tracking_all
   TRACKING_CLEAN = tracking_clean
endif 

all: $(TRACKING_ALL)
	./buildnum.py
	cd core_modules; make
	cd routing; make
	cd kernel; make

tracking_all:
	cd tracking; make

refresh: clean $(TRACKING_CLEAN) all $(TRACKING_ALL)

clean: $(TRACKING_CLEAN)
	cd core_modules; make clean
	cd routing; make clean
	cd kernel; make clean

tracking_clean:
	cd tracking; make clean

