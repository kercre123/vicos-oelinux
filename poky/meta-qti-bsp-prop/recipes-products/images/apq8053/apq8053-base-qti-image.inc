# Packages installed to the root file system

IMAGE_INSTALL += "common"
IMAGE_INSTALL += "modem-shutdown"
IMAGE_INSTALL += "msmirqbalance"
IMAGE_INSTALL += "start-scripts-misc-daemon"
IMAGE_INSTALL += "start-scripts-qseecomd-daemon"
IMAGE_INSTALL += "start-scripts-sfs-config"

# Following packages can compile for both 32bit and 64bit.
# When multilib is enabled, compile both variants.
IMAGE_INSTALL += "adsprpc"
IMAGE_INSTALL += "${@base_contains('MULTILIB_VARIANTS', 'lib32', 'lib32-adsprpc', '', d)}"
IMAGE_INSTALL += "sensors"
IMAGE_INSTALL += "${@base_contains('MULTILIB_VARIANTS', 'lib32', 'lib32-sensors', '', d)}"

IMAGE_INSTALL_ATTEMPTONLY += "fastmmi"
IMAGE_INSTALL_ATTEMPTONLY += "${@base_contains('MULTILIB_VARIANTS', 'lib32', 'lib32-fastmmi', '', d)}"
