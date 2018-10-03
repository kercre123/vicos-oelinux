FLAGS = \
	-std=gnu++14 \
	-D_GNU_SOURCE \
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

user-data-locker: user-data-locker.cpp
	${CXX} ${CXXFLAGS} $(FLAGS) -o $@ $^

clean:
	rm user-data-locker
