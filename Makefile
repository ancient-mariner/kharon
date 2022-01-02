# builds entire project

########################################################################
# random note for running callgrind
# callgrind_control -d [hint [PID/Name]]

# random note for investigating coredump on process outside of debugger
# get stack trace from failed process (in logs or printed to stdout)
# decompile
#    objdump -S -r -D ./bob > out
# look for offset to fault line in object file
# this works best with debug symbols included and linked w/ -rdynamic, and
#    compiled at low optimization level
########################################################################

# set global compile options
#export OPT = -ffast-math -march=native -g  -flto -rdynamic
#export OPT = -ffast-math -march=native -g -O1 -rdynamic
#export OPT = -ffast-math -march=native -g -O3  -rdynamic
export OPT = -ffast-math -march=native -O3 -flto -rdynamic

include util/set_env_base.make

core: $(PLATFORM)
   
all: external $(PLATFORM)

external:
	cd external/src/; make

RPI:
	cd local/; make
	cd mapping/create/; make
	cd core; make
	cd remote; make

INTEL:
	cd local/; make
	cd mapping/create/; make
	cd core; make


refresh: clean_$(PLATFORM) $(PLATFORM)


clean_RPI:
	cd local/src/; make clean
	cd mapping/create/; make clean
	cd core; make clean
	cd remote; make clean

clean_INTEL:
	cd local/src/; make clean
	cd mapping/create/; make clean
	cd core; make clean

