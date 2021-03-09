all: relay sensors

relay: relay.c
	gcc -o relay -Wall -g -O2 relay.c 

sensors: sensors.c ../../util/util_sensors.c ../../util/util_misc.c ../../util/bme680.cpp
	gcc -o sensors -Wall -g -O2 -li2c sensors.c \
            ../../util/util_sensors.c \
            ../../util/util_misc.c \
            ../../util/bme680.cpp \
            ../../util/u8g2/libu8g2.a

clean:
	rm -f relay sensors
