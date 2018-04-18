#
# Include the robot settings
#

# Get base product settings
require apq8009-base-image.inc

# These two lines below is a workaround to remove init-mss,
# since packagegroup-ss-mgr just include reboot-daemon and init-mss
IMAGE_INSTALL_remove = "packagegroup-ss-mgr"
IMAGE_INSTALL += "reboot-daemon"

# Add additional components
IMAGE_INSTALL += "system-core-debuggerd"
IMAGE_INSTALL += "e2fsprogs"
IMAGE_INSTALL += "start-scripts-userfs-reset"
IMAGE_INSTALL += "chrony"
IMAGE_INSTALL += "chronyc"
IMAGE_INSTALL += "${@base_conditional('USER_BUILD', '1', '', 'openssh', d)}"
IMAGE_INSTALL += "connman connman-client connman-tools"
IMAGE_INSTALL += "iw"
IMAGE_INSTALL += "${@base_conditional('PRODUCT','robot-rome', 'sigma-dut', '', d)}"
IMAGE_INSTALL += "${@base_conditional('PRODUCT','robot-rome', 'libcap', '', d)}"
IMAGE_INSTALL += "${@base_conditional('PRODUCT','robot-rome', 'attr', '', d)}"
IMAGE_INSTALL += "mtd-utils-ubifs"
IMAGE_INSTALL += "ankibluetoothd"
IMAGE_INSTALL += "emr-cat"
IMAGE_INSTALL += "blkdiscard"
IMAGE_INSTALL += "ankiinit"
IMAGE_INSTALL += "ankiwifi"
IMAGE_INSTALL += "lookbusy"
IMAGE_INSTALL += "lalibs"
IMAGE_INSTALL += "python"
IMAGE_INSTALL += "python-subprocess"
IMAGE_INSTALL += "python-netclient"
IMAGE_INSTALL += "python-compression"
IMAGE_INSTALL += "python-shell"
IMAGE_INSTALL += "python-crypt"
IMAGE_INSTALL += "update-engine"
