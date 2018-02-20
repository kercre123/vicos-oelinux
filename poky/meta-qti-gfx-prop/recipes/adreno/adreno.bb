inherit autotools qcommon qlicense qprebuilt

DESCRIPTION = "Adreno Graphics"
PR = "r0"
PROVIDES = "adreno200"

##### Display Frameworks #####
# Set this to 1, if Android display framework is supported
SUPPORTS_ANDROID_FRAMEWORK = "1"

# Set this to 1, if Wayland display framework is supported
SUPPORTS_WAYLAND_FRAMEWORK = "0"

# Set this to 1, if FBDEV is supported
SUPPORTS_FBDEV = "0"

##### Kernel Mode Drivers #####
#Set this to 1, if KGSL is supported
SUPPORTS_KGSL = "1"

#Set this to 1, if DRM is supported
SUPPORTS_DRM = "0"

##### Target specific setting #####
# APQ8098 uses DRM kernel with Wayland display framework
SUPPORTS_ANDROID_FRAMEWORK_apq8098 = "0"
SUPPORTS_WAYLAND_FRAMEWORK_apq8098 = "1"
SUPPORTS_FBDEV_apq8098 = "0"
SUPPORTS_KGSL_apq8098  = "0"
SUPPORTS_DRM_apq8098   = "1"

INCSUFFIX         = "adreno-cmake"
INCSUFFIX_apq8009 = "adreno-makefile-linux"
INCSUFFIX_apq8017 = "adreno-makefile-linux"

include ${INCSUFFIX}.inc
