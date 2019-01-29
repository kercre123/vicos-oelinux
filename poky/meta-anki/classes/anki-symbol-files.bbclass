#
# poky/meta-anki/classes/anki-symbol-files.bbclass
#
# This class adds a bitbake task "do_anki_symbol_files" to publish 
# full (unstripped) symbol files for export to backtrace.
#
# Enable by adding "inherit anki-symbol-files" to your bitbake recipe.
#
# Identify library files to be published by appending object file names to 
# ANKI_LIB_SYMBOL_FILES.
#
# This task runs after "do_install", so object files have been staged
# but not yet stripped.
#

# Which files do we publish?
ANKI_LIB_SYMBOL_FILES ?= ""

# Where do we put them?
ANKI_LIB_SYMBOL_DIR ?= "${WORKSPACE}/anki/victor/_build/vicos/Release/lib"

#
# For each named symbol file, find it in the build tree
# and install a copy to target directory.
#
do_anki_symbol_files () {

    for f in ${ANKI_LIB_SYMBOL_FILES}
    do
        # Locate symbol file
        IMAGE=${WORKDIR}/image
        if [ -f ${IMAGE}/lib/${f} ]; then
            src=${IMAGE}/lib/${f}
        elif [ -f ${IMAGE}/usr/lib/${f} ]; then
            src=${IMAGE}/usr/lib/${f}
        else
            echo "Can't find ${f} under ${IMAGE}"
            exit 1
        fi

        # Publish to target directory
        mkdir -p ${ANKI_LIB_SYMBOL_DIR}
        install ${src} ${ANKI_LIB_SYMBOL_DIR}/${f}.full
        
    done
}

#
# Add new task to invoke new build step
#
addtask anki_symbol_files after do_install before do_package

