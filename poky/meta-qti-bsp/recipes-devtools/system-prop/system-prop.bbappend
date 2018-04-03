SUMMARY = "Anki added persistent properties"

do_compile_append() {
  # turn off verbose qcom debug logs
  echo "persist.camera.hal.debug.mask=7" >> ${S}/build.prop

  echo "service.adb.tcp.port=5555" >> ${S}/build.prop
  bbwarn "TODO Disable ADB for shipping"

  # VIC-2127: Increase max tombstones
  echo "service.debuggerd.tombstones=32" >> ${S}/build.prop

}

