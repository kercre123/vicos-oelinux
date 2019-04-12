# Additional non-open source packages to be put to the image filesystem.
include ${BASEMACHINE}/${BASEMACHINE}-drone-qti-image.inc

inherit qimage

require internal-image.inc
# Set up for handling the generation of the /usr image
# partition...
require mdm-usr-image.inc

# Set up for handling the generation of the /cache image
# partition...
require mdm-cache-image.inc

# Set up for handling the generation of the /persist image
# partition only for APQ Targets
require apq-persist-image.inc

do_rootfs[nostamp] = "1"
do_build[nostamp]  = "1"

# Call function makesystem to generate sparse ext4 image
addtask makesystem after do_rootfs before do_build
