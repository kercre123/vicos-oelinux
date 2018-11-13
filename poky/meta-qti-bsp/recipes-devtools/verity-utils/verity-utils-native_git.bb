inherit native autotools pkgconfig

DESCRIPTION = "Verity utilites"
HOMEPAGE = "http://developer.android.com/"
LICENSE = "Apache-2.0"
LIC_FILES_CHKSUM = "file://${COREBASE}/meta/files/common-licenses/\
${LICENSE};md5=89aea4e17d99a7cacdbeed46a0096b10"

PR = "r0"

DEPENDS = "libgcc libmincrypt-native libsparse-native zlib-native openssl-native bouncycastle-native"

#PACKAGES = "libstdc++"

FILESPATH =+ "${WORKSPACE}/system/extras/:"
SRC_URI = "file://verity"

S = "${WORKDIR}/verity"

EXTRA_OECONF = "--with-core-includes=${WORKSPACE}/system/core/include"

CPPFLAGS += "-I${STAGING_INCDIR}/libselinux"
CPPFLAGS += "-I${STAGING_INCDIR}/libsparse"
CPPFLAGS += "-I${STAGING_INCDIR}/zlib"

# including C flags for compile generate_verity_key
CFLAGS += " -I${STAGING_INCDIR}"
LDFLAGS += "-lcrypto"
LDFLAGS += "-lsparse"

# Including C flags to compile verify_boot_signature
CFLAGS += " -I${WORKSPACE}/system/extras/ext4_utils"
CFLAGS += " -I${WORKSPACE}/system/core/mkbootimg"

do_compile () {
    ${CC} ${CFLAGS} ${S}/generate_verity_key.c -o generate_verity_key ${LDFLAGS}
    ${CC} ${CFLAGS} ${S}/verify_boot_signature.c -o verify_boot_signature ${LDFLAGS}
    ${CXX} ${CFLAGS} ${CPPFLAGS} ${S}/build_verity_tree.cpp -o build_verity_tree ${LDFLAGS}
    # ${CXX} ${CPPFLAGS} ${S}/verity_verifier.cpp -o verity_verifier ${LDFLAGS}
}


editveritysigner () {
    sed -i -e '/^java/d' ${S}/verity_signer
    echo 'java -Xmx512M -jar ${STAGING_LIBDIR_NATIVE}/VeritSigner.jar "$@"' >> ${S}/verity_signer
}

do_install () {
    install -d ${D}/${bindir}
    install -m 755 ${B}/build_verity_tree ${D}/${bindir}/build_verity_tree
    install -m 755 ${B}/generate_verity_key ${D}/${bindir}/generate_verity_key
    install -m 755 ${B}/verify_boot_signature ${D}/${bindir}/verify_boot_signature
    editveritysigner
    install -m 755 ${S}/verity_signer ${D}/${bindir}/verity_signer
    install -m 755 ${S}/build_verity_metadata.py ${D}/${bindir}/build_verity_metadata.py
}

NATIVE_INSTALL_WORKS="1"
