CFLAGS = \
	-Os \
	-Wall \
	-Werror \
	-Wno-unused-result \
	-Wno-strict-aliasing \
	-fPIC \
	-fvisibility=hidden \
	-ffunction-sections \
	-fdata-sections \
	-fstack-protector-strong \
	-D_FORTIFY_SOURCE=2 \
	-Wformat \
	-Werror=format-security \
	-Wl,-z,relro \
	-Wl,-z,now \
	-Wl,-z,noexecstack \
	-Wl,-z,nocopyreloc \
	-pie \
	-fPIE

BINARIES = rampost animfail

all: $(BINARIES)

clean:
	rm $(BINARIES)

rampost: rampost.c gpio.c lcd.c spine_hal.c imu.c
	$(CC) $(CFLAGS) rampost.c gpio.c lcd.c spine_hal.c imu.c -o $@

animfail: animfail.c gpio.c lcd.c
	$(CC) $(CFLAGS) animfail.c gpio.c lcd.c -o $@
