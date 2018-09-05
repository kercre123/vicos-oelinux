#
# Include the robot settings
#
IMAGE_INSTALL += "iptables-persistent"
IMAGE_INSTALL += "${@base_contains('DISTRO_FEATURES', 'ipv6', 'ip6tables-persistent', '', d)}"
