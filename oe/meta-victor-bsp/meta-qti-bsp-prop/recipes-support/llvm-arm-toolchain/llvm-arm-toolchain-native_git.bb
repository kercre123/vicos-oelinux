inherit native

DESCRIPTION = "CLANG compiler"
LICENSE = "Apache-2.0"
LIC_FILES_CHKSUM = "file://${COREBASE}/meta/files/common-licenses/\
${LICENSE};md5=89aea4e17d99a7cacdbeed46a0096b10"

PR = "r0"

FILESPATH =+ "${WORKSPACE}/vendor/qcom/proprietary/:"
SRC_URI   = "file://llvm-arm-toolchain-ship/3.8/"

S = "${WORKDIR}/llvm-arm-toolchain-ship/3.8"

PREFERRED_PROVIDER_clang   ?= "llvm-arm-toolchain"
PREFERRED_PROVIDER_clang++ ?= "llvm-arm-toolchain"

do_install() {
    install -d ${bindir}/llvm-arm-toolchain/
    install -d ${bindir}/llvm-arm-toolchain/bin/
    cp -rf ${S}/bin/* ${bindir}/llvm-arm-toolchain/bin/
    install -d ${bindir}/llvm-arm-toolchain/lib/
    cp -rf ${S}/lib/* ${bindir}/llvm-arm-toolchain/lib/
}
