# WLAN open source Packages
require apq8009-wlan-image.inc
IMAGE_INSTALL += "${@base_conditional('PRODUCT','robot-rome', 'qcacld-hl', '', d)}"
IMAGE_INSTALL_remove = "${@base_conditional('PRODUCT','robot-rome', 'wlan-opensource', '', d)}"
IMAGE_INSTALL += "${@base_conditional('PRODUCT','robot-rome', 'wlan-services', '', d)}"
