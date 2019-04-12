#
# poky/meta-anki/classes/anki-symbol-files.bbclass
#
# This class defines macros to manage full (unstripped) symbol files 
# produced by other recipes.
#
# Enable macros by adding "inherit anki-symbol-files" to your bitbake recipe.
#
# To export symbol files, add names to global ANKI_LIB_SYMBOL_FILES
# and add "do_anki_symbol_export" to your recipe install step.
#

# Which files do we publish?
ANKI_LIB_SYMBOL_FILES ?= ""

# Where do we put them?
ANKI_LIB_SYMBOL_DIR ?= "${WORKSPACE}/poky/build/tmp-glibc/anki-symbol-files/lib"

#
# For each named symbol file, find it in the build tree
# and install a copy to target directory.
#
do_anki_symbol_export () {

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
        install ${src} ${ANKI_LIB_SYMBOL_DIR}
        
    done
}

