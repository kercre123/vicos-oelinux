#
# poky/meta-anki/recipes-support/liburcu/liburcu_0.8.7.bbappend
#
# Recipe extension to publish symbol files for backtrace
#

inherit anki-symbol-files

ANKI_LIB_SYMBOL_FILES += "liburcu-bp.so.2.0.0"

