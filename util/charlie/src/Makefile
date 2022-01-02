include ../../set_env_base.make

OPT = -ffast-math -O2 -g

INC = -I$(LOCAL_INCLUDE)     \
      -I$(ROOT)util/charlie/include/     \
      -I$(ROOT)core/include/external/   \
      -I$(ROOT)core/include/kernel/

CFLAGS = $(INC) $(CORE_FLAGS) $(WARN) $(OPT)
LIB = $(LOCAL_LIB) -lm -ldl

OBJS = charlie.o 

TARGETS = aim go otto module

########################################################################
#
all: $(OBJS) $(TARGETS)


charlie.o: charlie.c
	$(CC) charlie.c -c $(CFLAGS) 

aim: aim.c
	$(CC) aim.c -o aim $(CFLAGS) $(OBJS) $(LIB)
	cp aim ../bin

go: go.c
	$(CC) go.c -o go $(CFLAGS) $(OBJS) $(LIB)
	cp go ../bin

module: module.c
	$(CC) module.c -o module $(CFLAGS) $(OBJS) $(LIB)
	cp module ../bin

otto: otto.c
	$(CC) otto.c -o otto $(CFLAGS) $(OBJS) $(LIB)
	cp otto ../bin


clean:
	rm -f *.o test_* $(TARGETS)

refresh: clean all

