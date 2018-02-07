OBJS=neutrino
INCL=neutrino.h
AVBLIB=libneutrino.a
#CFLAGS=-ggdb

CC?=gcc
RANLIB?=ranlib

all: $(AVBLIB)

$(AVBLIB): $(addsuffix .o,$(OBJS))
	$(RM) $@
	$(AR) rcs $@ $^
	$(RANLIB) $@

neutrino.o: neutrino.c $(INCL)
	$(CC) -c $(INCFLAGS) $(CFLAGS) neutrino.c

clean:
	$(RM) `find . -name "*~" -o -name "*.[oa]" -o -name "\#*\#" -o -name TAGS -o -name core -o -name "*.orig"`


