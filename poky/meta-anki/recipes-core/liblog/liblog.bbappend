#
# poky/meta-anki/recipes-core/liblog/liblog.bbappend
#
# Recipe extension to publish symbol files for backtrace
#

inherit anki-symbol-files

ANKI_LIB_SYMBOL_FILES += "liblog.so.0.0.0"

