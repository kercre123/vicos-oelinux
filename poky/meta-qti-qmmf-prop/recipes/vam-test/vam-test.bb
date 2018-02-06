inherit autotools pkgconfig qlicense

DESCRIPTION = "VAM Test"

PR = "r0"

DEPENDS += "vam-lib freetype libsdl util-linux"

FILESPATH =+ "${WORKSPACE}/vendor/qcom/proprietary/:"
SRC_URI   = "file://vam"

S = "${WORKDIR}/vam"

do_install_append() {
       install -d ${D}${libdir}/VAMEngines/

       install -d ${D}/usr/data/VAM/
       install -m 0755 ${S}/vam_sim/VASim_test.sh -D ${D}/usr/data/VAM/VASim_test.sh
       install -m 0755 ${S}/vam_sim/VASim_test.config -D ${D}/usr/data/VAM/VASim_test.config
       install -d ${D}/usr/data/VAM/sample_input/
       install -m 0755 ${S}/vam_sim/sample_input/* -D ${D}/usr/data/VAM/sample_input/
       install -d ${D}/usr/data/VAM/engine_data
       install -m 0755 ${S}/vam_sim/engine_data/* -D ${D}/usr/data/VAM/engine_data/
       install -d ${D}/usr/data/VAM/engine_data


}

FILES_${PN} += "/usr"

PACKAGES =+ "${PN}-vasim-dbg ${PN}-vasim"
FILES_${PN}-vasim-dbg = "${bindir}/VAM/.debug/VASim"
FILES_${PN}-vasim     = "${bindir}/VAM/VASim"

PACKAGES =+ "${PN}-vamreport-dbg ${PN}-vamreport"
FILES_${PN}-vamreport-dbg = "${bindir}/VAM/.debug/VAMReport"
FILES_${PN}-vamreport     = "${bindir}/VAM/VAMReport"

PACKAGES =+ "${PN}-libjsonapis-dbg ${PN}-libjsonapis ${PN}-libjsonapis-dev"
FILES_${PN}-libjsonapis-dbg    = "${libdir}/VAM/.debug/libjson_apis.*"
FILES_${PN}-libjsonapis        = "${libdir}/VAM/libjson_apis.so.*"
FILES_${PN}-libjsonapis-dev    = "${libdir}/VAM/libjson_apis.so ${libdir}/VAM/libjson_apis.la ${includedir}"

PACKAGES =+ "${PN}-libgtrend-dbg ${PN}-libgtrend ${PN}-libgtrend-dev"
FILES_${PN}-libgtrend-dbg    = "${libdir}/VAMEngines/.debug/libEngine_GTRenderer.*"
FILES_${PN}-libgtrend        = "${libdir}/VAMEngines/libEngine_GTRenderer.so.*"
FILES_${PN}-libgtrend-dev    = "${libdir}/VAMEngines/libEngine_GTRenderer.so ${libdir}/VAM/libEngine_GTRenderer.la ${includedir}"


PACKAGES =+ "${PN}-libtest1-dbg ${PN}-libtest1 ${PN}-libtest1-dev"
FILES_${PN}-libtest1-dbg    = "${libdir}/VAM/VAMEngines/.debug/libTest1.*"
FILES_${PN}-libtest1        = "${libdir}/VAM/VAMEngines/libEngine_Test1.so.*"
FILES_${PN}-libtest1-dev    = "${libdir}/VAM/VAMEngines/libEngine_Test1.so ${libdir}/VAM/VAMEngines/libEngine_Test1.la ${includedir}"


PACKAGES =+ "${PN}-libtest2-dbg ${PN}-libtest2 ${PN}-libtest2-dev"
FILES_${PN}-libtest2-dbg    = "${libdir}/VAM/VAMEngines/.debug/libEngine_Test2.*"
FILES_${PN}-libtest2        = "${libdir}/VAM/VAMEngines/libEngine_Test2.so.*"
FILES_${PN}-libtest2-dev    = "${libdir}/VAM/VAMEngines/libEngine_Test2.so ${libdir}/VAM/VAMEngines/libEngine_Test2.la ${includedir}"

PACKAGES =+ "${PN}-customengine-dbg ${PN}-customengine ${PN}-customengine-dev"
FILES_${PN}-customengine-dbg    = "${libdir}/VAM/VAMEngines/.debug/libEngine_CustomEngineTemplate.*"
FILES_${PN}-customengine        = "${libdir}/VAM/VAMEngines/libEngine_CustomEngineTemplate.so.*"
FILES_${PN}-customengine-dev    = "${libdir}/VAM/VAMEngines/libEngine_CustomEngineTemplate.so ${libdir}/VAM/VAMEngines/libEngine_CustomEngineTemplate.la ${includedir}"

INSANE_SKIP_${PN}-vasim += "dev-deps"
INSANE_SKIP_${PN}-vamreport += "dev-deps"

