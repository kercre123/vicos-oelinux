# WLAN open source Packages
IMAGE_INSTALL += "wcnss"
IMAGE_INSTALL += "wlan-opensource"
IMAGE_INSTALL += "qcacld-utils"
IMAGE_INSTALL += "hostap-daemon-qcacld"
IMAGE_INSTALL += "wpa-supplicant-qcacld"

IMAGE_INSTALL += "${@base_conditional('PRODUCT','drone', 'qcacld-hl', '', d)}"
IMAGE_INSTALL_remove = "${@base_conditional('PRODUCT','drone', 'wlan-opensource', '', d)}"
IMAGE_INSTALL += "${@base_conditional('PRODUCT','drone', 'wlan-services', '', d)}"
