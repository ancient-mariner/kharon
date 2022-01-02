include ../../util/set_env_base.make
include ../../util/mapping_set_env.make


LIB = -lm
OBJS = common.o declination.o 
TARGETS = init_map1_gebco load_gebco_map2 view_level2 see_level1 \
      read_noaa observe_level3 build_composite horeline

all: $(OBJS) $(TARGETS) lib

lib:
	gcc-ar rcsv $(MAPPING_LIB) $(OBJS)
	

%.o: %.c $(HDRS)
	$(CC) $< -c -o $@ $(CFLAGS)

build_composite: build_composite.c common.o
	$(CC) -o build_composite build_composite.c common.o $(CFLAGS) $(LIB)

horeline: horeline.c common.o
	$(CC) -o horeline horeline.c common.o $(CFLAGS) $(LIB)

init_map1_gebco: init_map1_gebco.c common.o
	$(CC) -o init_map1_gebco init_map1_gebco.c common.o $(CFLAGS) $(LIB)

load_gebco_map2: load_gebco_map2.c common.o
	$(CC) -o load_gebco_map2 load_gebco_map2.c common.o $(CFLAGS) $(LIB)

observe_level3: observe_level3.c common.o
	$(CC) -o observe_level3 observe_level3.c common.o $(CFLAGS) $(LIB)

read_noaa: read_noaa.c common.o
	$(CC) -o read_noaa read_noaa.c common.o $(CFLAGS) $(LIB) -lz

see_level1: see_level1.c common.o
	$(CC) -o see_level1 see_level1.c common.o $(CFLAGS) $(LIB)

view_level2: view_level2.c common.o
	$(CC) -o view_level2 view_level2.c common.o $(CFLAGS) $(LIB)

clean:
	rm -f *.o test_* $(TARGETS)

refresh: clean all

