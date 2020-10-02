IMG_DIR=../poky/build/tmp-glibc/deploy/images/apq8009-robot-robot-perf
OS_VERSION_FILE=../poky/build/tmp-glibc/work/apq8009_robot-oe-linux-gnueabi/machine-robot-image/1.0-r0/rootfs/etc/os-version
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

UNAME_S := $(shell uname -s)
ifeq ($(UNAME_S),Darwin)
  TAR=gtar
  export COPYFILE_DISABLE=1
  # COPYFILE_DISABLE=1 causes OSX not to put '._' junk in our TAR file when files have extended attributes
else
  TAR=tar
endif

OTAKEY ?= ota_test.key
OTAPASS ?= file:ota_test.pass
OTAENC ?= 1
ANKIDEV ?= 1

ifeq ($(ANKIDEV), 0)
 MANIFEST_VERSION = 0.9.2
else
 MANIFEST_VERSION = 1.0.0
endif

UPDATE_VERSION ?= $(shell cat $(OS_VERSION_FILE))

OTA_FILE=$(BUILD)/vicos-$(UPDATE_VERSION).ota

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

%.stats: %.img
	echo "bytes=`wc -c $< | awk '{ print $$1 }'`"            > $@
	echo "sha256=`shasum -a256 -b $< | awk '{ print $$1 }'`" >> $@

%.sha256: %.ini
	openssl dgst -sha256 -sign $(OTAKEY) -passin env:OTA_KEY_PASS -out $@ $<

$(MANIFEST).ini: $(BUILD) $(BOOT_STATS) $(SYS_STATS)
	echo "[META]"                                > $@
	echo "manifest_version=$(MANIFEST_VERSION)" >> $@
	echo "update_version=$(UPDATE_VERSION)"     >> $@
	echo "ankidev=$(ANKIDEV)"                   >> $@
	echo "num_images=2"                         >> $@
	echo "[BOOT]"                               >> $@
	echo "encryption=$(OTAENC)"                 >> $@
	echo "delta=0"                              >> $@
	echo "compression=gz"                       >> $@
	echo "wbits=$(GZIP_WBITS)"                  >> $@
	cat $(BOOT_STATS)                           >> $@
	echo "[SYSTEM]"                             >> $@
	echo "encryption=$(OTAENC)"                 >> $@
	echo "delta=0"                              >> $@
	echo "compression=gz"                       >> $@
	echo "wbits=$(GZIP_WBITS)"                  >> $@
	cat $(SYS_STATS)                            >> $@

PERMS="0400"
OWNER="root:0"
GROUP="root:0"

$(OTA_FILE): $(MANIFEST).sha256 $(BOOT_GZ_ENC) $(SYS_GZ_ENC)
	$(TAR) --transform='flags=r;s|.enc||' -cf $@ \
	--mode=$(PERMS) \
	--owner=$(OWNER) \
	--group=$(GROUP) \
	-C $(BUILD) \
	manifest.ini \
	manifest.sha256 \
	apq8009-robot-boot.img.gz.enc \
	apq8009-robot-sysfs.img.gz.enc

serve: $(OTA_FILE)
	cd $(BUILD); python -m SimpleHTTPServer 5555


###############################################################################

NON_SECURE_KERNEL=../poky/build/tmp-glibc/deploy/images/apq8009-robot-robot-perf/apq8009-robot-boot.img.nonsecure
PROD_KERNEL=../poky/build/tmp-glibc/deploy/images/apq8009-robot-robot-perf/apq8009-robot-boot.img
BOOT_KEY ?= qtipri.encrypted.key
DEV_BOOT_KEY = ../poky/meta-qti-bsp-prop/recipes-products/images/kernel/vble-qti.key
OSKR_BOOT_KEY ?= qtioskrpri.encrypted.key

prodsign:
	openssl dgst -sha256 -binary $(NON_SECURE_KERNEL) > $(NON_SECURE_KERNEL).sha256
	openssl pkeyutl -sign -in $(NON_SECURE_KERNEL).sha256 -inkey $(BOOT_KEY) -out $(NON_SECURE_KERNEL).sig -pkeyopt digest:sha256 -pkeyopt rsa_padding_mode:pkcs1
	dd if=/dev/zero of=$(NON_SECURE_KERNEL).sig.padded bs=2048 count=1
	dd if=$(NON_SECURE_KERNEL).sig of=$(NON_SECURE_KERNEL).sig.padded conv=notrunc
	cat $(NON_SECURE_KERNEL) $(NON_SECURE_KERNEL).sig.padded > $(PROD_KERNEL)
	rm -rf $(NON_SECURE_KERNEL).sha256 $(NON_SECURE_KERNEL).sig $(NON_SECURE_KERNEL).sig.padded

oskrsign:
	openssl dgst -sha256 -binary $(NON_SECURE_KERNEL) > $(NON_SECURE_KERNEL).sha256
	openssl pkeyutl -sign -in $(NON_SECURE_KERNEL).sha256 -inkey $(OSKR_BOOT_KEY) -out $(NON_SECURE_KERNEL).sig -pkeyopt digest:sha256 -pkeyopt rsa_padding_mode:pkcs1
	dd if=/dev/zero of=$(NON_SECURE_KERNEL).sig.padded bs=2048 count=1
	dd if=$(NON_SECURE_KERNEL).sig of=$(NON_SECURE_KERNEL).sig.padded conv=notrunc
	cat $(NON_SECURE_KERNEL) $(NON_SECURE_KERNEL).sig.padded > $(PROD_KERNEL)
#	rm -rf $(NON_SECURE_KERNEL).sha256 $(NON_SECURE_KERNEL).sig $(NON_SECURE_KERNEL).sig.padded

devsign:
	openssl dgst -sha256 -binary $(NON_SECURE_KERNEL) > $(NON_SECURE_KERNEL).sha256
	openssl pkeyutl -sign -in $(NON_SECURE_KERNEL).sha256 -inkey $(DEV_BOOT_KEY) -out $(NON_SECURE_KERNEL).sig -pkeyopt digest:sha256 -pkeyopt rsa_padding_mode:pkcs1
	dd if=/dev/zero of=$(NON_SECURE_KERNEL).sig.padded bs=2048 count=1
	dd if=$(NON_SECURE_KERNEL).sig of=$(NON_SECURE_KERNEL).sig.padded conv=notrunc
	cat $(NON_SECURE_KERNEL) $(NON_SECURE_KERNEL).sig.padded > $(PROD_KERNEL)
	rm -rf $(NON_SECURE_KERNEL).sha256 $(NON_SECURE_KERNEL).sig $(NON_SECURE_KERNEL).sig.padded


resign-dev: $(BUILD)/$(BOOT_STEM).img
	truncate --size=-2048 $<
	openssl dgst -sha256 -binary $< > $<.sha256
	openssl pkeyutl -sign -in $<.sha256 -inkey $(DEV_BOOT_KEY) -out $<.sig -pkeyopt digest:sha256 -pkeyopt rsa_padding_mode:pkcs1
	dd if=/dev/zero of=$<.sig.padded bs=2048 count=1
	dd if=$<.sig of=$<.sig.padded conv=notrunc
	cat $< $<.sig.padded > $<.signed
	mv $<.signed $<
	rm -rf $<.sha256 $<.sig $<.sig.padded

#
# Because making new boot images is delicate, these can be used to test.
# Both the ABOOT and BOOT must match, for example passing both prod tests
# 

EMMC_FILE=../poky/build/tmp-glibc/work/apq8009_robot-oe-linux-gnueabi/lk/git-r1signed_robot_perf_perf/package/boot/integrity/emmc_appsboot.mbn

verify-aboot-oskr:
	grep "Digital Dream Labs" $(EMMC_FILE)
	echo "ABOOT CHECK PASSED. CORRECT x509."

verify-aboot-prod:
	grep "Anki" $(EMMC_FILE)
	grep "San Francisco" $(EMMC_FILE)
	echo "ABOOT CHECK PASSED. CORRECT x509."

verify-aboot-dev:
	grep "Widgit" $(EMMC_FILE)
	echo "ABOOT CHECK PASSED. CORRECT x509."

verify-boot-oskr: verify-aboot-oskr
	openssl pkeyutl -verify -in  $(NON_SECURE_KERNEL).sha256  -sigfile $(NON_SECURE_KERNEL).sig -inkey $(OSKR_BOOT_KEY) -pkeyopt digest:sha256 -pkeyopt rsa_padding_mode:pkcs1 | cat

verify-boot-prod: verify-aboot-prod
	openssl pkeyutl -verify -in  $(NON_SECURE_KERNEL).sha256  -sigfile $(NON_SECURE_KERNEL).sig -inkey $(BOOT_KEY) -pkeyopt digest:sha256 -pkeyopt rsa_padding_mode:pkcs1 | cat

verify-boot-dev: verify-aboot-dev
	openssl pkeyutl -verify -in  $(NON_SECURE_KERNEL).sha256  -sigfile $(NON_SECURE_KERNEL).sig -inkey $(DEV_BOOT_KEY) -pkeyopt digest:sha256 -pkeyopt rsa_padding_mode:pkcs1 | cat

