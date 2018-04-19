IMG_DIR=../poky/build/tmp-glibc/deploy/images/apq8009-robot-robot-perf
BUILD=../_build
BOOT_STEM=apq8009-robot-boot
SYS_STEM=apq8009-robot-sysfs
MANIFEST=$(BUILD)/manifest

BOOT_GZ=$(BUILD)/$(BOOT_STEM).img.gz
BOOT_STATS=$(BUILD)/$(BOOT_STEM).stats
SYS_GZ=$(BUILD)/$(SYS_STEM).img.gz
SYS_STATS=$(BUILD)/$(SYS_STEM).stats

GZIP_MODE=--best
GZIP_WBITS=31

OTAKEY ?= ota_test.key

BASE_VERSION=$(shell cat ../ANKI_VERSION)
ANKI_BUILD_VERSION ?= 0
UPDATE_VERSION ?= $(BASE_VERSION).$(ANKI_BUILD_VERSION)

OTA_FILE=$(BUILD)/vicos-$(UPDATE_VERSION).ota

all: $(BUILD) $(OTA_FILE)

clean:
	rm -rf $(BUILD)

deepclean: clean
	rm -f $(IMG_DIR)/*

.PHONY: clean deepclean

.PRECIOUS: $(BUILD)/%.img

$(BUILD):
	mkdir $(BUILD)

$(BUILD)/%.img: $(IMG_DIR)/%.img
	cp $< $@

%.img.gz: %.img
	gzip $(GZIP_MODE) --force --keep $<

$(BUILD)/%.img: $(IMG_DIR)/%.ext4
	simg2img $< $@

%.stats: %.img
	echo "bytes=`wc -c $< | awk '{ print $$1 }'`"            > $@
	echo "sha256=`shasum -a256 -b $< | awk '{ print $$1 }'`" >> $@

%.sha256: %.ini
	openssl dgst -sha256 -sign $(OTAKEY) -out $@ $<

$(MANIFEST).ini: $(BUILD) $(BOOT_STATS) $(SYS_STATS)
	echo "[META]"                           > $@
	echo "manifest_version=0.9.2"           >> $@
	echo "update_version=$(UPDATE_VERSION)" >> $@
	echo "num_images=2"                     >> $@
	echo "[BOOT]"                           >> $@
	echo "delta=0"                          >> $@
	echo "compression=gz"                   >> $@
	echo "wbits=$(GZIP_WBITS)"              >> $@
	cat $(BOOT_STATS)                       >> $@
	echo "[SYSTEM]"                         >> $@
	echo "delta=0"                          >> $@
	echo "compression=gz"                   >> $@
	echo "wbits=$(GZIP_WBITS)"              >> $@
	cat $(SYS_STATS)                        >> $@

# COPYFILE_DISABLE=1 causes OSX not to put '._' junk in our TAR file when files have extended attributes
$(OTA_FILE): $(MANIFEST).sha256 $(BOOT_GZ) $(SYS_GZ)
	COPYFILE_DISABLE=1 tar cf $@ -C $(BUILD) manifest.ini manifest.sha256 apq8009-robot-boot.img.gz apq8009-robot-sysfs.img.gz

serve: $(OTA_FILE)
	cd $(BUILD); python -m SimpleHTTPServer 5555
