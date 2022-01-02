
CC = g++


WARN = -Wall -Wextra -Wshadow -Wpointer-arith -Wcast-qual \
		 -Wcast-align -Wconversion -Wdouble-promotion \
       -Wduplicated-cond -Wlogical-op -Wnull-dereference

DEF = -D_GNU_SOURCE

CFLAGS = -Wall $(DEF) $(WARN) 

OBJS = test_arduino_otto

all: clean $(OBJS)

test_arduino_otto: test.cpp
	$(CC) test.cpp -o test_arduino_otto $(CFLAGS)

refresh: all

clean:
	rm -f $(OBJS)

