# Packages installed to the root file system

# Following packages support only 32-bit compilation.
# When 32-bit multilib is enabled, compile them with lib32- prefix.

IMAGE_INSTALL += "${@base_contains('MULTILIB_VARIANTS', 'lib32', 'lib32-acdbloader', 'acdbloader', d)}"
IMAGE_INSTALL += "${@base_contains('MULTILIB_VARIANTS', 'lib32', 'lib32-acdbmapper', 'acdbmapper', d)}"
IMAGE_INSTALL += "${@base_contains('MULTILIB_VARIANTS', 'lib32', 'lib32-acdbrtac', 'acdbrtac', d)}"
IMAGE_INSTALL += "${@base_contains('MULTILIB_VARIANTS', 'lib32', 'lib32-adiertac', 'adiertac', d)}"
IMAGE_INSTALL += "${@base_contains('MULTILIB_VARIANTS', 'lib32', 'lib32-audcal', 'audcal', d)}"
IMAGE_INSTALL += "${@base_contains('MULTILIB_VARIANTS', 'lib32', 'lib32-audcaltests', 'audcaltests', d)}"
IMAGE_INSTALL += "${@base_contains('MULTILIB_VARIANTS', 'lib32', 'lib32-audio-ftm', 'audio-ftm', d)}"
IMAGE_INSTALL += "${@base_contains('MULTILIB_VARIANTS', 'lib32', 'lib32-soundtrigger', 'soundtrigger', d)}"
IMAGE_INSTALL += "${@base_contains('MULTILIB_VARIANTS', 'lib32', 'lib32-acdb-fts', 'acdb-fts', d)}"
IMAGE_INSTALL += "${@base_contains('MULTILIB_VARIANTS', 'lib32', 'lib32-acdb-mcs', 'acdb-mcs', d)}"
IMAGE_INSTALL += "${@base_contains('MULTILIB_VARIANTS', 'lib32', 'lib32-audio-codecs-noship', 'audio-codecs-noship', d)}"
IMAGE_INSTALL += "${@base_contains('MULTILIB_VARIANTS', 'lib32', 'lib32-audio-effects', 'audio-effects', d)}"
IMAGE_INSTALL += "${@base_contains('MULTILIB_VARIANTS', 'lib32', 'lib32-audio-effects-noship', 'audio-effects-noship', d)}"
IMAGE_INSTALL += "${@base_contains('MULTILIB_VARIANTS', 'lib32', 'lib32-audio-generic-effect-framework', 'audio-generic-effect-framework', d)}"
IMAGE_INSTALL += "${@base_contains('MULTILIB_VARIANTS', 'lib32', 'lib32-omx', 'omx', d)}"
IMAGE_INSTALL += "${@base_contains('MULTILIB_VARIANTS', 'lib32', 'lib32-omx-noship', 'omx-noship', d)}"
IMAGE_INSTALL += "${@base_contains('MULTILIB_VARIANTS', 'lib32', 'lib32-smwrapper', 'smwrapper', d)}"
IMAGE_INSTALL += "${@base_contains('MULTILIB_VARIANTS', 'lib32', 'lib32-qsthw-api', 'qsthw-api', d)}"
IMAGE_INSTALL += "${@base_contains('MULTILIB_VARIANTS', 'lib32', 'lib32-ffv', 'ffv', d)}"
