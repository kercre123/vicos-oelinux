#
# poky/meta-anki/recipes-support/sqlite/sqlite3_3.8.10.2.bbappend
#
# Recipe extension to publish symbol files for backtrace
#

inherit anki-symbol-files

ANKI_LIB_SYMBOL_FILES += "libsqlite3.so.0.8.6"

addtask anki_symbol_export after do_install before do_package

