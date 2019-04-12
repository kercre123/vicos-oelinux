
# <anki>
# Don't start compiling openssl until libc headers are available in sysroot
# </anki>

DEPENDS += "linux-libc-headers"

