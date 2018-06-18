CFLAGS = -Os -Wall -Werror -Wno-unused-result -Wno-strict-aliasing -fPIC
BINARIES = rampost animfail

all: $(BINARIES)

clean:
	rm $(BINARIES)

rampost: rampost.c gpio.c lcd.c spine_hal.c imu.c
	$(CC) $(CFLAGS) rampost.c gpio.c lcd.c spine_hal.c imu.c -o $@

animfail: animfail.c gpio.c lcd.c
	$(CC) $(CFLAGS) animfail.c gpio.c lcd.c -o $@
