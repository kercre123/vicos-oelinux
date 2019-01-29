#
# poky/meta-anki/recipes-core/libunwind/libunwind.bbappend
#
# Recipe extension to publish symbol files for backtrace
#

inherit anki-symbol-files

ANKI_LIB_SYMBOL_FILES += "libunwind.so.0.0.0"

