#
# poky/meta-anki/recipes-core/liblog/liblog.bbappend
#
# Recipe extension to publish symbol files for backtrace
#

inherit anki-symbol-files

ANKI_LIB_SYMBOL_FILES += "liblog.so.0.0.0"

addtask anki_symbol_export after do_install before do_package

