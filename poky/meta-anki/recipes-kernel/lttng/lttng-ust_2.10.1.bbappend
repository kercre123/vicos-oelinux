#
# poky/meta-anki/recipes-kernel/lttng/lttng-ust_2.10.1.bbappend
#
# Recipe extension to publish symbol files for backtrace
#

inherit anki-symbol-files

ANKI_LIB_SYMBOL_FILES += "liblttng-ust.so.0.0.0"

addtask anki_symbol_export after do_install before do_package

