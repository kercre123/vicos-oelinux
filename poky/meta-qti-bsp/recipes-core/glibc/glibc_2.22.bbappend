
# glibc can't be built with -fstack-protector optimization, override it.

def fix_optimization(d):
    selected_opt = d.getVar("SELECTED_OPTIMIZATION", True)
    if bb.utils.contains("SELECTED_OPTIMIZATION", "-fstack-protector", "y", "", d) == "y":
        # bb.note("glibc can't be built with stack-protector.")
        return selected_opt.replace("-fstack-protector", "")
    return selected_opt

SELECTED_OPTIMIZATION := "${@fix_optimization(d)}"

FILESEXTRAPATHS_prepend := "${THISDIR}/files:"
SRC_URI += " file://0001-Adding-support-to-dlopen-library-from-memory.patch "
