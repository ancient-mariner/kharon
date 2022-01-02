include ../../util/set_env_base.make
include ../../util/mapping_set_env.make

########################################################################
#

LIB = $(MOD_ROUTING_LIB) $(MAPPING_LIB) $(LOCAL_LIB) -lm

TARGETS = view_octant see_box convert_ascii2bin

all: $(TARGETS)
	

%.o: %.c $(HDRS)
	$(CC) $< -c -o $@ $(CFLAGS)
	@touch $(COMPILE_FLAG)

view_octant: view_octant.c  
	$(CC) -o view_octant view_octant.c $(CFLAGS) $(LIB)

see_box: see_box.c  
	$(CC) -o see_box see_box.c $(CFLAGS) $(LIB)

convert_ascii2bin: convert_ascii2bin.c  
	$(CC) -o convert_ascii2bin convert_ascii2bin.c $(CFLAGS) $(LIB)


clean:
	rm -f *.o $(TARGETS)

refresh: clean all

