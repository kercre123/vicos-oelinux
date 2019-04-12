#
# poky/meta-anki/recipes-core/zlib/zlib_1.2.8.bbappend
#
# Recipe extension to publish symbol files for backtrace
#

inherit anki-symbol-files

ANKI_LIB_SYMBOL_FILES += "libz.so.1.2.8"

addtask anki_symbol_export after do_install before do_package

