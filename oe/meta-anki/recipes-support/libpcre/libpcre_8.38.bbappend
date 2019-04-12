#
# poky/meta-anki/recipes-support/libpcre/libpcre_8.38.bbappend
#
# Recipe extension to publish symbol files for backtrace
#

inherit anki-symbol-files

ANKI_LIB_SYMBOL_FILES += "libpcre.so.1.2.6"

addtask anki_symbol_export after do_install before do_package

