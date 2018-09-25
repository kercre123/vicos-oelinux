SUMMARY = "VL53L1 driver from ST Micro"
LICENSE = "GPLv2"
LIC_FILES_CHKSUM = "file://COPYING;md5=12f884d2ae1ff87c09e5b7ccc2c4ca7e"

inherit module

SRC_URI =  "file://vl53l1_platform_log.h \
            file://stmvl53l1_tunings.h \
            file://inc/vl53l1_error_exceptions.h \
            file://inc/vl53l1_register_settings.h \
            file://inc/vl53l1_zone_presets.h \
            file://inc/vl53l1_wait.h \
            file://inc/vl53l1_preset_setup.h \
            file://inc/vl53l1_ll_device.h \
            file://inc/vl53l1_api_calibration.h \
            file://inc/vl53l1_register_funcs.h \
            file://inc/vl53l1_hist_char.h \
            file://inc/vl53l1_hist_map.h \
            file://inc/vl53l1_register_map.h \
            file://inc/vl53l1_error_codes.h \
            file://inc/vl53l1_core.h \
            file://inc/vl53l1_register_structs.h \
            file://inc/vl53l1_api_preset_modes.h \
            file://inc/vl53l1_nvm.h \
            file://inc/vl53l1_nvm_map.h \
            file://inc/vl53l1_core_support.h \
            file://inc/vl53l1_error_strings.h \
            file://inc/vl53l1_ll_def.h \
            file://inc/vl53l1_api.h \
            file://inc/vl53l1_api_strings.h \
            file://inc/vl53l1_dmax_structs.h \
            file://inc/vl53l1_fpga_core.h \
            file://inc/vl53l1_api_debug.h \
            file://inc/vl53l1_silicon_core.h \
            file://inc/vl53l1_nvm_structs.h \
            file://inc/vl53l1_nvm_debug.h \
            file://inc/vl53l1_def.h \
            file://inc/vl53l1_api_core.h \
            file://inc/vl53l1_hist_structs.h \
            file://inc/vl53l1_tuning_parm_defaults.h \
            file://vl53l1_platform_ipp.h \
            file://stmvl53l1_ipp.h \
            file://stmvl53l1-i2c.h \
            file://stmvl53l1_module.c \
            file://Makefile \
            file://stmvl53l1_ipp_nl.c \
            file://stmvl53l1.h \
            file://vl53l1_types.h \
            file://vl53l1_platform_user_data.h \
            file://vl53l1_platform_user_config.h \
            file://stmvl53l1_module-i2c.c \
            file://vl53l1_platform_user_defines.h \
            file://stmvl53l1_if.h \
            file://stmvl53l1_log.c \
            file://ipp/ipp_linux.c \
            file://stmvl53l1_internal_if.h \
            file://st,stmvl53l1.txt \
            file://vl53l1_platform.h \
            file://COPYING \
            file://vl53l1_platform_ipp_imports.h \
            file://stmvl53l1_i2c.c \
            file://src/vl53l1_nvm.c \
            file://src/vl53l1_api.c \
            file://src/vl53l1_error_strings.c \
            file://src/vl53l1_core_support.c \
            file://src/vl53l1_silicon_core.c \
            file://src/vl53l1_api_debug.c \
            file://src/vl53l1_fpga_core.c \
            file://src/vl53l1_api_strings.c \
            file://src/vl53l1_api_core.c \
            file://src/vl53l1_nvm_debug.c \
            file://src/vl53l1_zone_presets.c \
            file://src/vl53l1_api_calibration.c \
            file://src/vl53l1_wait.c \
            file://src/vl53l1_register_funcs.c \
            file://src/vl53l1_hist_char.c \
            file://src/vl53l1_core.c \
            file://src/vl53l1_api_preset_modes.c \
          "

S = "${WORKDIR}"

# Install nessisary header files
do_install_append() {
  install -d ${D}${includedir}
  install ${S}/*.h     ${D}${includedir}
  install ${S}/inc/*.h ${D}${includedir}
}

# The inherit of module.bbclass will automatically name module packages with
# "kernel-module-" prefix as required by the oe-core build environment.
