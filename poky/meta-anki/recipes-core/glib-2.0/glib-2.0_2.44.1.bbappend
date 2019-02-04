#
# poky/meta-anki/recipes-core/glib-2.0/glib-2.0_2.44.1.bbappend
#
# Recipe extension to publish symbol files for backtrace
#

inherit anki-symbol-files

ANKI_LIB_SYMBOL_FILES += "libgio-2.0.so.0.4400.1"
ANKI_LIB_SYMBOL_FILES += "libglib-2.0.so.0.4400.1"
ANKI_LIB_SYMBOL_FILES += "libgobject-2.0.so.0.4400.1"

addtask anki_symbol_export after do_install before do_package

