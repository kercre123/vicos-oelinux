#
# poky/meta-anki/recipes-core/curl/curl_7.54.0.bbappend
#
# Recipe extension to publish symbol files for backtrace
#

inherit anki-symbol-files

ANKI_LIB_SYMBOL_FILES += "libcurl.so.5.4.0"

