TARGET_KERNEL_VERSION := 4.4
$(call inherit-product, device/qcom/common/common64.mk)

PRODUCT_NAME := msmcobalt
PRODUCT_DEVICE := msmcobalt
PRODUCT_BRAND := Android
PRODUCT_MODEL := Cobalt for arm64

# default is nosdcard, S/W button enabled in resource
PRODUCT_CHARACTERISTICS := nosdcard

#Android EGL implementation
PRODUCT_PACKAGES += libGLES_android
PRODUCT_BOOT_JARS += tcmiface
# Audio configuration file
PRODUCT_COPY_FILES += \
    device/qcom/msmcobalt/audio_policy.conf:system/etc/audio_policy.conf \
    device/qcom/msmcobalt/audio_effects.conf:system/vendor/etc/audio_effects.conf \
    device/qcom/msmcobalt/mixer_paths.xml:system/etc/mixer_paths.xml \
    device/qcom/msmcobalt/mixer_paths_i2s.xml:system/etc/mixer_paths_i2s.xml \
    device/qcom/msmcobalt/audio_platform_info_i2s.xml:system/etc/audio_platform_info_i2s.xml

# Listen configuration file
PRODUCT_COPY_FILES += \
    device/qcom/msmcobalt/listen_platform_info.xml:system/etc/listen_platform_info.xml

# Sensor HAL conf file
PRODUCT_COPY_FILES += \
    device/qcom/msmcobalt/sensors/hals.conf:system/etc/sensors/hals.conf

# WLAN driver configuration file
PRODUCT_COPY_FILES += \
    device/qcom/msmcobalt/WCNSS_qcom_cfg.ini:system/etc/wifi/WCNSS_qcom_cfg.ini

# Sensor features
PRODUCT_COPY_FILES += \
    frameworks/native/data/etc/android.hardware.sensor.accelerometer.xml:system/etc/permissions/android.hardware.sensor.accelerometer.xml \
    frameworks/native/data/etc/android.hardware.sensor.compass.xml:system/etc/permissions/android.hardware.sensor.compass.xml \
    frameworks/native/data/etc/android.hardware.sensor.gyroscope.xml:system/etc/permissions/android.hardware.sensor.gyroscope.xml \
    frameworks/native/data/etc/android.hardware.sensor.light.xml:system/etc/permissions/android.hardware.sensor.light.xml \
    frameworks/native/data/etc/android.hardware.sensor.proximity.xml:system/etc/permissions/android.hardware.sensor.proximity.xml \
    frameworks/native/data/etc/android.hardware.sensor.barometer.xml:system/etc/permissions/android.hardware.sensor.barometer.xml \
    frameworks/native/data/etc/android.hardware.sensor.stepcounter.xml:system/etc/permissions/android.hardware.sensor.stepcounter.xml \
    frameworks/native/data/etc/android.hardware.sensor.stepdetector.xml:system/etc/permissions/android.hardware.sensor.stepdetector.xml \
    frameworks/native/data/etc/android.hardware.sensor.ambient_temperature.xml:system/etc/permissions/android.hardware.sensor.ambient_temperature.xml \
    frameworks/native/data/etc/android.hardware.sensor.relative_humidity.xml:system/etc/permissions/android.hardware.sensor.relative_humidity.xml \
    frameworks/native/data/etc/android.hardware.sensor.hifi_sensors.xml:system/etc/permissions/android.hardware.sensor.hifi_sensors.xml
