# Proprietary Packages

# Following packages support only 32-bit compilation.
# When 32-bit multilib is enabled, compile them with lib32- prefix.

# VIDEO Packages
IMAGE_INSTALL += "${@base_contains('MULTILIB_VARIANTS', 'lib32', 'lib32-mm-video-noship', 'mm-video-noship', d)}"
IMAGE_INSTALL += "${@base_contains('MULTILIB_VARIANTS', 'lib32', 'lib32-mm-video-utils', 'mm-video-utils', d)}"

# PARSER PACKAGES
IMAGE_INSTALL += "${@base_contains('MULTILIB_VARIANTS', 'lib32', 'lib32-mm-osal', 'mm-osal', d)}"
IMAGE_INSTALL += "${@base_contains('MULTILIB_VARIANTS', 'lib32', 'lib32-mm-parser-noship', 'mm-parser-noship', d)}"
