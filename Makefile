TARGETS = robot 

CC = gcc
CFLAGS = -Wall -g -O2 -Iutil

SRC_ROBOT = robot.c \
            util/util_mc.c \
            util/util_misc.c 

#
# build rules
#

all: $(TARGETS)

robot: $(SRC_ROBOT:.c=.o)
	$(CC) -o $@ $(SRC_ROBOT:.c=.o) -lpthread

#
# clean rule
#

clean:
	rm -f $(TARGETS) *.o util/*.o
