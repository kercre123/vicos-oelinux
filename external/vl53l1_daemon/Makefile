

#######################################################################
# Generic Makefile for Linux/Ubuntu to compile/build
# Usage: from terminal use "make clean; make" to build vl53l1_daemon.so
#######################################################################

exe =vl53l1_daemon

BIN_DIR ?= $(INSTALL_ROOT)/usr/bin

#------------------------------------------------------------------------------
# Source Files groups
#------------------------------------------------------------------------------
LIB_SOURCES =\
		vl53l1_daemon.c\
		core/src/vl53l1_core_support.c\
		protected/src/vl53l1_hist_funcs.c\
		protected/src/vl53l1_xtalk.c\
		protected/src/vl53l1_hist_algos_gen2.c\
		protected/src/vl53l1_hist_algos_gen3.c\
		protected/src/vl53l1_hist_core.c\
		protected/src/vl53l1_sigma_estimate.c\
		protected/src/vl53l1_dmax.c\
		protected/src/vl53l1_hist_algos_gen4.c

INCLUDE_PATH = -Iprotected/inc -I.

#------------------------------------------------------------------------------
# COmpiler flags
#------------------------------------------------------------------------------

#WARNINGS_ON_FLAGS = -Wall -Wextra -fmessage-length=0 -Wtype-limits -Wdeclaration-after-statement -fmax-errors=4
WARNINGS_ON_FLAGS = -Wall

BASE_CFLAGS = -fPIC $(WARNINGS_ON_FLAGS) -O2

CFLAGS = $(BASE_CFLAGS) $(INCLUDE_PATH)

#------------------------------------------------------------------------------
# Main Targets
#------------------------------------------------------------------------------

all: $(exe)

#------------------------------------------------------------------------------
#  Target Builds
#------------------------------------------------------------------------------

$(BIN_DIR): $(INSTALL_ROOT)
	mkdir -p $@

$(exe):
	$(CC) $(CFLAGS) -lpthread vl53l1_daemon_main.c $(LIB_SOURCES) -o $@

install: $(exe) $(BIN_DIR)
	cp -v $(exe) $(BIN_DIR)/$(exe)

clean:
	rm -f ./core/src/*.o
	rm -f ./protected/src/*.o
	rm -f ./*.o *.so
	rm -f $(exe)
