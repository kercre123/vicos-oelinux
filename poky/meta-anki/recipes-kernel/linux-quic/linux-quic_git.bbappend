#
# poky/meta-anki/recipes-kernel/linux-quic/linux-quic_git.bbappend
#
# Recipe extension to publish symbol files for backtrace
#
# Note that kernel build does not follow the standard package pattern,
# so we have to use a custom task definition.
#
# Symbol file (vdso.so.dbg) is renamed to 'linux-gate.so' to match the 
# name that is reported to backtrace.
#

inherit anki-symbol-files

do_anki_symbol_export () {
  src="${B}/arch/${ARCH}/vdso/vdso.so.dbg"
  dst="${ANKI_LIB_SYMBOL_DIR}/linux-gate.so"
  mkdir -p ${ANKI_LIB_SYMBOL_DIR}
  install ${src} ${dst}
}

addtask anki_symbol_export after do_install before do_package

