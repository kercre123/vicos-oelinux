IMG_DIR=../poky/build/tmp-glibc/deploy/images/apq8009-robot-robot-perf
OS_VERSION_FILE=../poky/build/tmp-glibc/work/apq8009_robot-oe-linux-gnueabi/machine-robot-image/1.0-r0/rootfs/etc/os-version
BUILD=../_build
BOOT_STEM=apq8009-robot-boot
SYS_STEM=apq8009-robot-sysfs
ABOOT_STEM=emmc_appsboot
MANIFEST=$(BUILD)/manifest

ABOOT_GZ=$(BUILD)/$(ABOOT_STEM).img.gz
ABOOT_GZ_ENC=$(ABOOT_GZ).enc
ABOOT_STATS=$(BUILD)/$(ABOOT_STEM).stats
BOOT_GZ=$(BUILD)/$(BOOT_STEM).img.gz
BOOT_GZ_ENC=$(BOOT_GZ).enc
BOOT_STATS=$(BUILD)/$(BOOT_STEM).stats
SYS_GZ=$(BUILD)/$(SYS_STEM).img.gz
SYS_STATS=$(BUILD)/$(SYS_STEM).stats
SYS_GZ_ENC=$(SYS_GZ).enc

GZIP_MODE=--best
GZIP_WBITS=31

UNAME_S := $(shell uname -s)
ifeq ($(UNAME_S),Darwin)
  TAR=gtar
  export COPYFILE_DISABLE=1
  # COPYFILE_DISABLE=1 causes OSX not to put '._' junk in our TAR file when files have extended attributes
else
  TAR=tar
endif

OTAKEY ?= ota_prod.key
OTAPASS ?= file:ota_test.pass
OTAENC ?= 1
ANKIDEV ?= 1

UPDATE_VERSION ?= $(shell cat $(OS_VERSION_FILE))

OTA_FILE=$(BUILD)/vicos-$(UPDATE_VERSION)-$(QSN).ota

all: $(BUILD) $(OTA_FILE)

clean:
	rm -rf $(BUILD)

deepclean: clean
	rm -rf $(IMG_DIR)/*

.PHONY: clean deepclean

.PRECIOUS: $(BUILD)/%.img

$(BUILD):
	mkdir $(BUILD)

$(BUILD)/%.img: $(IMG_DIR)/%.img
	cp $< $@

%.img.gz: %.img
	gzip $(GZIP_MODE) --force --keep $<

%.img.gz.enc: %.img.gz
	openssl aes-256-ctr -pass $(OTAPASS) -in $< -out $@

$(BUILD)/%.img: $(IMG_DIR)/%.ext4
	simg2img $< $@

$(BUILD)/%.img: $(IMG_DIR)/%.mbn
	cp $< $@

%.stats: %.img
	echo "bytes=`wc -c $< | awk '{ print $$1 }'`"            > $@
	echo "sha256=`shasum -a256 -b $< | awk '{ print $$1 }'`" >> $@

%.sha256: %.ini
	openssl dgst -sha256 -sign $(OTAKEY) -passin pass:$(OTA_KEY_PASS) -out $@ $<

$(MANIFEST).ini: $(BUILD) $(ABOOT_STATS) $(BOOT_STATS) $(SYS_STATS)
	echo "[META]"                           > $@
	echo "manifest_version=1.0.0"           >> $@
	echo "update_version=$(UPDATE_VERSION)" >> $@
	echo "ankidev=$(ANKIDEV)"               >> $@
	echo "qsn=$(QSN)"                       >> $@
	echo "num_images=3"                     >> $@
	echo "[ABOOT]"                          >> $@
	echo "encryption=$(OTAENC)"             >> $@
	echo "delta=0"                          >> $@
	echo "compression=gz"                   >> $@
	echo "wbits=$(GZIP_WBITS)"              >> $@
	cat $(ABOOT_STATS)                      >> $@
	echo "[RECOVERY]"                       >> $@
	echo "encryption=$(OTAENC)"             >> $@
	echo "delta=0"                          >> $@
	echo "compression=gz"                   >> $@
	echo "wbits=$(GZIP_WBITS)"              >> $@
	cat $(BOOT_STATS)                       >> $@
	echo "[RECOVERYFS]"                     >> $@
	echo "encryption=$(OTAENC)"             >> $@
	echo "delta=0"                          >> $@
	echo "compression=gz"                   >> $@
	echo "wbits=$(GZIP_WBITS)"              >> $@
	cat $(SYS_STATS)                        >> $@

PERMS="0400"
OWNER="root:0"
GROUP="root:0"

$(OTA_FILE): $(MANIFEST).sha256 $(ABOOT_GZ_ENC) $(BOOT_GZ_ENC) $(SYS_GZ_ENC)
	$(TAR) --transform='flags=r;s|.enc||' -cf $@ \
	--mode=$(PERMS) \
	--owner=$(OWNER) \
	--group=$(GROUP) \
	-C $(BUILD) \
	manifest.ini \
	manifest.sha256 \
	emmc_appsboot.img.gz.enc \
	apq8009-robot-boot.img.gz.enc \
	apq8009-robot-sysfs.img.gz.enc

serve: $(OTA_FILE)
	cd $(BUILD); python -m SimpleHTTPServer 5555
