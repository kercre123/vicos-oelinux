# Override default init script run order. This has to run after other
# initscripts have finished remounting /dev.

INITSCRIPT_PARAMS = "start 04 S ."
SELINUX_SCRIPT_DST = "${BPN}"
