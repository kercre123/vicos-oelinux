SUMMARY = "Wayland Protocol upgrade recipe for wayland-1.9.0"
FILESEXTRAPATHS_prepend := "${WORKSPACE}/poky/meta-qti-display/recipes/wayland:"
SRC_URI_append = " \
          file://wayland-protocol-Add-wl_output-v3.patch \
       "
