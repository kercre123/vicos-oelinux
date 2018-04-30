IMG_DIR=../poky/build/tmp-glibc/deploy/images/apq8009-robot-robot-perf
BUILD=../_build
BOOT_STEM=apq8009-robot-boot
SYS_STEM=apq8009-robot-sysfs
MANIFEST=$(BUILD)/manifest

BOOT_GZ=$(BUILD)/$(BOOT_STEM).img.gz
BOOT_GZ_ENC=$(BOOT_GZ).enc
BOOT_STATS=$(BUILD)/$(BOOT_STEM).stats
SYS_GZ=$(BUILD)/$(SYS_STEM).img.gz
SYS_STATS=$(BUILD)/$(SYS_STEM).stats
SYS_GZ_ENC=$(SYS_GZ).enc

GZIP_MODE=--best
GZIP_WBITS=31

OTAKEY ?= ota_test.key
OTAPASS ?= ota_test.pass
OTAENC ?= 1

BASE_VERSION=$(shell cat ../ANKI_VERSION)
ANKI_BUILD_VERSION ?= 0
UPDATE_VERSION ?= $(BASE_VERSION).$(ANKI_BUILD_VERSION)

OTA_FILE=$(BUILD)/vicos-$(UPDATE_VERSION).ota
ENC_OTA_FILE=$(BUILD)/vicos-$(UPDATE_VERSION).enc.ota

all: $(BUILD) $(OTA_FILE) $(ENC_OTA_FILE)

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

%.img.gz.enc: %.img.gz
	openssl aes-256-ctr -pass file:$(OTAPASS) -in $< -out $@

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
	echo "encryption=0" 	                >> $@
	echo "delta=0"                          >> $@
	echo "compression=gz"                   >> $@
	echo "wbits=$(GZIP_WBITS)"              >> $@
	cat $(BOOT_STATS)                       >> $@
	echo "[SYSTEM]"                         >> $@
	echo "encryption=0"                     >> $@
	echo "delta=0"                          >> $@
	echo "compression=gz"                   >> $@
	echo "wbits=$(GZIP_WBITS)"              >> $@
	cat $(SYS_STATS)                        >> $@

$(MANIFEST).enc.ini: $(MANIFEST).ini
	cat $< | sed -e "s#encryption=0#encryption=$(OTAENC)##g" \
		     -e "s#manifest_version=0.9.2#manifest_version=0.9.5##g"> $@

# COPYFILE_DISABLE=1 causes OSX not to put '._' junk in our TAR file when files have extended attributes
$(OTA_FILE): $(MANIFEST).sha256 $(BOOT_GZ) $(SYS_GZ)
	COPYFILE_DISABLE=1 tar cf $@ -C $(BUILD) manifest.ini manifest.sha256 apq8009-robot-boot.img.gz apq8009-robot-sysfs.img.gz

$(ENC_OTA_FILE): $(MANIFEST).enc.sha256 $(BOOT_GZ_ENC) $(SYS_GZ_ENC)
	COPYFILE_DISABLE=1 tar --transform='flags=r;s|.enc||' -cf $@ -C $(BUILD) manifest.enc.ini manifest.enc.sha256 apq8009-robot-boot.img.gz.enc apq8009-robot-sysfs.img.gz.enc

serve: $(OTA_FILE) $(ENC_OTA_FILE)
	cd $(BUILD); python -m SimpleHTTPServer 5555
