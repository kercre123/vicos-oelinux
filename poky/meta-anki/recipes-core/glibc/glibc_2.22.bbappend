#
# poky/meta-anki/recipes-core/glibc/glibc_2.22.bbappend
#
# Recipe extension to publish symbol files for backtrace
#

inherit anki-symbol-files

ANKI_LIB_SYMBOL_FILES += "libc-2.22.so"
ANKI_LIB_SYMBOL_FILES += "libnsl-2.22.so"
ANKI_LIB_SYMBOL_FILES += "libnss_nis-2.22.so"
ANKI_LIB_SYMBOL_FILES += "libnss_compat-2.22.so"
ANKI_LIB_SYMBOL_FILES += "libnss_files-2.22.so"

