# Prop Security Recipes

# Following packages support only 32-bit compilation.
# When 32-bit multilib is enabled, compile them with lib32- prefix.

IMAGE_INSTALL += "${@base_contains('MULTILIB_VARIANTS', 'lib32', 'lib32-securemsm', 'securemsm', d)}"
IMAGE_INSTALL += "${@base_contains('MULTILIB_VARIANTS', 'lib32', 'lib32-securemsm-noship', 'securemsm-noship', d)}"

IMAGE_INSTALL += "start-scripts-qseecomd-daemon"
IMAGE_INSTALL += "start-scripts-sfs-config"
