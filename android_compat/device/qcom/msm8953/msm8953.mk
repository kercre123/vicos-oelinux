DEVICE_PACKAGE_OVERLAYS := device/qcom/msm8953_64/overlay

TARGET_USES_QCOM_BSP := true
BOARD_HAVE_QCOM_FM := true
# Add QC Video Enhancements flag
TARGET_ENABLE_QC_AV_ENHANCEMENTS := true
TARGET_USES_NQ_NFC := true
TARGET_KERNEL_VERSION := 3.18
#QTIC flag
-include $(QCPATH)/common/config/qtic-config.mk

# media_profiles and media_codecs xmls for msm8953
ifeq ($(TARGET_ENABLE_QC_AV_ENHANCEMENTS), true)
PRODUCT_COPY_FILES += device/qcom/msm8953_32/media/media_profiles_8953.xml:system/etc/media_profiles.xml \
                      device/qcom/msm8953_32/media/media_codecs_8953.xml:system/etc/media_codecs.xml \
                      device/qcom/msm8953_32/media/media_codecs_performance_8953.xml:system/etc/media_codecs_performance.xml
endif

PRODUCT_COPY_FILES += device/qcom/msm8953_64/whitelistedapps.xml:system/etc/whitelistedapps.xml

$(call inherit-product, $(DEVICE_DIR)/qcom/common/common64.mk)

PRODUCT_NAME := msm8953
PRODUCT_DEVICE := msm8953
PRODUCT_BRAND := Android
PRODUCT_MODEL := msm8953 for arm64

PRODUCT_BOOT_JARS += tcmiface

ifneq ($(strip $(QCPATH)),)
PRODUCT_BOOT_JARS += WfdCommon
PRODUCT_BOOT_JARS += com.qti.dpmframework
PRODUCT_BOOT_JARS += dpmapi
PRODUCT_BOOT_JARS += com.qti.location.sdk
endif

ifeq ($(strip $(BOARD_HAVE_QCOM_FM)),true)
PRODUCT_BOOT_JARS += qcom.fmradio
endif #BOARD_HAVE_QCOM_FM
PRODUCT_BOOT_JARS += qcmediaplayer

# default is nosdcard, S/W button enabled in resource
PRODUCT_CHARACTERISTICS := nosdcard

ifeq ($(strip $(TARGET_USES_QTIC)),true)
# font rendering engine feature switch
-include $(QCPATH)/common/config/rendering-engine.mk
ifneq (,$(strip $(wildcard $(PRODUCT_RENDERING_ENGINE_REVLIB))))
    MULTI_LANG_ENGINE := REVERIE
#    MULTI_LANG_ZAWGYI := REVERIE
endif
endif



#Android EGL implementation
PRODUCT_PACKAGES += libGLES_android

PRODUCT_PACKAGES += \
    libqcomvisualizer \
    libqcompostprocbundle \
    libqcomvoiceprocessing

# Audio configuration file
PRODUCT_COPY_FILES += \
    device/qcom/msm8953_32/audio_policy.conf:system/etc/audio_policy.conf \
    device/qcom/msm8953_32/audio_output_policy.conf:system/vendor/etc/audio_output_policy.conf \
    device/qcom/msm8953_32/audio_effects.conf:system/vendor/etc/audio_effects.conf \
    device/qcom/msm8953_32/mixer_paths_mtp.xml:system/etc/mixer_paths_mtp.xml \
    device/qcom/msm8953_32/mixer_paths_qrd_skuh.xml:system/etc/mixer_paths_qrd_skuh.xml \
    device/qcom/msm8953_32/mixer_paths_qrd_skui.xml:system/etc/mixer_paths_qrd_skui.xml \
    device/qcom/msm8953_32/mixer_paths_qrd_skuhf.xml:system/etc/mixer_paths_qrd_skuhf.xml \
    device/qcom/msm8953_32/mixer_paths_qrd_skum.xml:system/etc/mixer_paths_qrd_skum.xml \
    device/qcom/msm8953_32/mixer_paths_qrd_sku3.xml:system/etc/mixer_paths_qrd_sku3.xml \
    device/qcom/msm8953_32/mixer_paths_wcd9330.xml:system/etc/mixer_paths_wcd9330.xml \
    device/qcom/msm8953_32/mixer_paths_wcd9306.xml:system/etc/mixer_paths_wcd9306.xml \
    device/qcom/msm8953_32/mixer_paths_wcd9335.xml:system/etc/mixer_paths_wcd9335.xml \
    device/qcom/msm8953_32/mixer_paths_wcd9326.xml:system/etc/mixer_paths_wcd9326.xml \
    device/qcom/msm8953_32/mixer_paths_qrd_skun.xml:system/etc/mixer_paths_qrd_skun.xml \
    device/qcom/msm8953_32/mixer_paths_skuk.xml:system/etc/mixer_paths_skuk.xml \
    device/qcom/msm8953_32/mixer_paths.xml:system/etc/mixer_paths.xml \
    device/qcom/msm8953_32/sound_trigger_mixer_paths.xml:system/etc/sound_trigger_mixer_paths.xml \
    device/qcom/msm8953_32/sound_trigger_mixer_paths_wcd9306.xml:system/etc/sound_trigger_mixer_paths_wcd9306.xml \
    device/qcom/msm8953_32/sound_trigger_mixer_paths_wcd9330.xml:system/etc/sound_trigger_mixer_paths_wcd9330.xml \
    device/qcom/msm8953_32/sound_trigger_mixer_paths_wcd9335.xml:system/etc/sound_trigger_mixer_paths_wcd9335.xml \
    device/qcom/msm8953_32/sound_trigger_platform_info.xml:system/etc/sound_trigger_platform_info.xml \
    device/qcom/msm8953_32/audio_platform_info.xml:system/etc/audio_platform_info.xml \
    device/qcom/msm8953_32/audio_platform_info_extcodec.xml:system/etc/audio_platform_info_extcodec.xml \
    device/qcom/msm8953_64/aanc_tuning_mixer.txt:system/etc/aanc_tuning_mixer.txt

# MIDI feature
PRODUCT_COPY_FILES += \
    frameworks/native/data/etc/android.software.midi.xml:system/etc/permissions/android.software.midi.xml

#ANT+ stack
PRODUCT_PACKAGES += \
    AntHalService \
    libantradio \
    antradio_app

# NFC
ifeq ($(strip $(TARGET_USES_NQ_NFC)),true)
PRODUCT_PACKAGES += \
    NQNfcNci \
    libnqnfc-nci \
    libnqnfc_nci_jni \
    nfc_nci.nqx.default \
    libp61-jcop-kit \
    com.nxp.nfc.nq \
    com.nxp.nfc.nq.xml \
    libpn547_fw.so \
    libpn548ad_fw.so \
    libnfc-brcm.conf \
    libnfc-nxp.conf \
    nqnfcee_access.xml \
    nqnfcse_access.xml \
    Tag \
    com.android.nfc_extras \
    libQPayJNI \
    com.android.qti.qpay \
    com.android.qti.qpay.xml \
    SmartcardService \
    org.simalliance.openmobileapi \
    org.simalliance.openmobileapi.xml

PRODUCT_COPY_FILES += \
    packages/apps/Nfc/migrate_nfc.txt:system/etc/updatecmds/migrate_nfc.txt \
    frameworks/native/data/etc/com.nxp.mifare.xml:system/etc/permissions/com.nxp.mifare.xml \
    frameworks/native/data/etc/com.android.nfc_extras.xml:system/etc/permissions/com.android.nfc_extras.xml \
    frameworks/native/data/etc/android.hardware.nfc.xml:system/etc/permissions/android.hardware.nfc.xml \
    frameworks/native/data/etc/android.hardware.nfc.hce.xml:system/etc/permissions/android.hardware.nfc.hce.xml

# SmartcardService, SIM1,SIM2,eSE1 not including eSE2,SD1 as default
ADDITIONAL_BUILD_PROPERTIES += persist.nfc.smartcard.config=SIM1,SIM2,eSE1
endif # TARGET_USES_NQ_NFC

PRODUCT_PACKAGES += wcnss_service

# MSM IRQ Balancer configuration file
PRODUCT_COPY_FILES += \
    device/qcom/msm8953_64/msm_irqbalance.conf:system/vendor/etc/msm_irqbalance.conf

#wlan driver
PRODUCT_COPY_FILES += \
    device/qcom/msm8953_64/WCNSS_qcom_cfg.ini:system/etc/wifi/WCNSS_qcom_cfg.ini \
    device/qcom/msm8953_32/WCNSS_wlan_dictionary.dat:persist/WCNSS_wlan_dictionary.dat \
    device/qcom/msm8953_64/WCNSS_qcom_wlan_nv.bin:persist/WCNSS_qcom_wlan_nv.bin

PRODUCT_PACKAGES += \
    wpa_supplicant_overlay.conf \
    p2p_supplicant_overlay.conf

# Feature definition files for msm8953
PRODUCT_COPY_FILES += \
    frameworks/native/data/etc/android.hardware.sensor.accelerometer.xml:system/etc/permissions/android.hardware.sensor.accelerometer.xml \
    frameworks/native/data/etc/android.hardware.sensor.compass.xml:system/etc/permissions/android.hardware.sensor.compass.xml \
    frameworks/native/data/etc/android.hardware.sensor.gyroscope.xml:system/etc/permissions/android.hardware.sensor.gyroscope.xml \
    frameworks/native/data/etc/android.hardware.sensor.light.xml:system/etc/permissions/android.hardware.sensor.light.xml \
    frameworks/native/data/etc/android.hardware.sensor.proximity.xml:system/etc/permissions/android.hardware.sensor.proximity.xml \
    frameworks/native/data/etc/android.hardware.sensor.barometer.xml:system/etc/permissions/android.hardware.sensor.barometer.xml \
    frameworks/native/data/etc/android.hardware.sensor.stepcounter.xml:system/etc/permissions/android.hardware.sensor.stepcounter.xml \
    frameworks/native/data/etc/android.hardware.sensor.stepdetector.xml:system/etc/permissions/android.hardware.sensor.stepdetector.xml


# Defined the locales
PRODUCT_LOCALES += th_TH vi_VN tl_PH hi_IN ar_EG ru_RU tr_TR pt_BR bn_IN mr_IN ta_IN te_IN zh_HK \
        in_ID my_MM km_KH sw_KE uk_UA pl_PL sr_RS sl_SI fa_IR kn_IN ml_IN ur_IN gu_IN or_IN

# Add the overlay path
ifeq ($(strip $(TARGET_USES_QTIC)),true)
PRODUCT_PACKAGE_OVERLAYS := $(QCPATH)/qrdplus/Extension/res-overlay \
        $(QCPATH)/qrdplus/globalization/multi-language/res-overlay \
        $(PRODUCT_PACKAGE_OVERLAYS)
endif

# Sensor HAL conf file
 PRODUCT_COPY_FILES += \
     device/qcom/msm8953_64/sensors/hals.conf:system/etc/sensors/hals.conf

# Disable Verity boot feature
PRODUCT_SUPPORTS_VERITY := true

# Reduce client buffer size for fast audio output tracks
PRODUCT_PROPERTY_OVERRIDES += \
     af.fast_track_multiplier=1

# Low latency audio buffer size in frames
PRODUCT_PROPERTY_OVERRIDES += \
    audio_hal.period_size=192
