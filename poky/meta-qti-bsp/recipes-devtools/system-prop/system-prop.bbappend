SUMMARY = "Anki added persistent properties"

do_compile_append() {
  echo "service.adb.tcp.port=5555" >> ${S}/build.prop
  bbwarn "TODO Disable ADB for shipping"
}

