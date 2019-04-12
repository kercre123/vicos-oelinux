# qmmf proprietary Packages

# Following packages support only 32-bit compilation.
# When multilib is enabled, compile them with lib32- prefix.

IMAGE_INSTALL += "${@base_contains('MULTILIB_VARIANTS', 'lib32', 'lib32-vam-lib', 'vam-lib', d)}"
IMAGE_INSTALL += "${@base_contains('MULTILIB_VARIANTS', 'lib32', 'lib32-vam-test', 'vam-test', d)}"
IMAGE_INSTALL += "${@base_contains('MULTILIB_VARIANTS', 'lib32', 'lib32-vam-engines', 'vam-engines', d)}"
