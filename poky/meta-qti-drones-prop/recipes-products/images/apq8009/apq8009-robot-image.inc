#Packages that robot installed to the root file system

IMAGE_INSTALL += "time-sync"

## the default distro configuration
VSLAM_DIR_PRESENT = "${@os.path.exists('${WORKSPACE}/vendor/qcom/proprietary/robot/vslam')}"
IMAGE_INSTALL += "${@base_conditional('VSLAM_DIR_PRESENT', 'True', 'vslam', '', d)}"
