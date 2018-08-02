#
# Include the robot settings
#
# Anki stuff
IMAGE_INSTALL += "anki-audio-init"
IMAGE_INSTALL += "anki-robot-target"
IMAGE_INSTALL += "anki-version"
#
# Victor stuff
IMAGE_INSTALL += "vic-anim"
IMAGE_INSTALL += "vic-christen"
IMAGE_INSTALL += "vic-engine"
IMAGE_INSTALL += "vic-init"
IMAGE_INSTALL += "vic-neuralnets"
IMAGE_INSTALL += "vic-on-failure"
IMAGE_INSTALL += "vic-on-failure-animfail"
IMAGE_INSTALL += "vic-robot"
IMAGE_INSTALL += "vic-switchboard"
IMAGE_INSTALL += "victor"
#
# Other stuff
IMAGE_INSTALL += "bspatch"
IMAGE_INSTALL += "fake-hwclock"
IMAGE_INSTALL += "rng-tools"
IMAGE_INSTALL += "anki-update-engine"

# BRC: Disabled for OOBA build
# IMAGE_INSTALL += "vic-webserver"
# IMAGE_INSTALL += "vic-gateway"
# IMAGE_INSTALL += "vic-gateway-cert"
# IMAGE_INSTALL += "vic-crashuploader"
# IMAGE_INSTALL += "vic-cloud"
# IMAGE_INSTALL += "vic-dasmgr"
# IMAGE_INSTALL += "rebooter"
