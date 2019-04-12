# Packages that drone installed to the root file system
require ${BASEMACHINE}-base-qti-image.inc

IMAGE_INSTALL += "lib32-mm-video-utils"
IMAGE_INSTALL += "start-scripts-qseecomd-daemon"
IMAGE_INSTALL += "securemsm-noship"
IMAGE_INSTALL += "securemsm"
IMAGE_INSTALL += "q6-admin-adsp"
IMAGE_INSTALL += "q6-admin-slpi"
