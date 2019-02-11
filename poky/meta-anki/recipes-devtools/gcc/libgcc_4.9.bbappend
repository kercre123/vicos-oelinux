#
# poky/meta-anki/recipes-devtools/gcc/libgcc_4.9.bbappend
#
# Recipe extension to publish symbol files for backtrace
#

inherit anki-symbol-files

ANKI_LIB_SYMBOL_FILES += "libgcc_s.so.1"

addtask anki_symbol_export after do_install before do_package

