do_install_append() {
  cd ${D}${bindir}
  ln -s ./mg emacs
}

