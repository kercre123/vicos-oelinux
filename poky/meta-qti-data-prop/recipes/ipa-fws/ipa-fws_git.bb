inherit qcommon qlicense

DESCRIPTION = "IPA firmwares"
PR = "r3"

INSANE_SKIP_${PN} = "arch"
FILES_${PN} += "${base_libdir}/firmware/ipa/*"
FILES_${PN} += "${base_libdir}/firmware/ipa_fws*"
FILESPATH = "${WORKSPACE}:"
SRC_DIR = "${WORKSPACE}/data/ipa_fws"
S = "${WORKDIR}/ipa_fws"

IPA_ELF_FIRMWARE_PATH = "${D}${base_libdir}/firmware/ipa"
IPA_SPLIT_FIRMWARE_PATH = "${D}${base_libdir}/firmware"

EXTRA_OECONF += "${@base_conditional('BASEMACHINE', 'mdm9650', '--enable-target-mdm9650=yes', '', d)}"
EXTRA_OECONF += "${@base_conditional('BASEMACHINE', 'sdx20', '--enable-target-sdx20=yes', '', d)}"

do_install() {
   install -d ${IPA_ELF_FIRMWARE_PATH}
   install -m 0755 ${S}/ipa_fws.elf -D ${IPA_ELF_FIRMWARE_PATH}

   if [ -f ${S}/ipa_fws.mdt ]; then
      install -d ${IPA_SPLIT_FIRMWARE_PATH}
      install -m 0755 ${S}/ipa_fws.b* -D ${IPA_SPLIT_FIRMWARE_PATH}
      install -m 0755 ${S}/ipa_fws.mdt -D ${IPA_SPLIT_FIRMWARE_PATH}
   fi;
}
