# gfx proprietary source Packages

# Adreno can compile for both 32bit and 64bit.
# When multilib is enabled, compile both variants.
IMAGE_INSTALL += "adreno200"
IMAGE_INSTALL += "${@base_contains('MULTILIB_VARIANTS', 'lib32', 'lib32-adreno200', '', d)}"
