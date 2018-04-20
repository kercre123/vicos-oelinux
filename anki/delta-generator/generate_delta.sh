#!/usr/bin/env bash

set -e
set -u

SCRIPT_PATH=$(dirname $([ -L $0 ] && echo "$(dirname $0)/$(readlink -n $0)" || echo $0))

function usage() {
    echo "$0 old_build_artifacts_dir new_build_artifacts_dir delta_file"
    echo ""
    echo "old_build_artifacts_dir  - The directory with the artifacts from"
    echo "                           the build you want to upgrade."
    echo "new_build_artifacts_dir  - The directory with the artifacts for"
    echo "                           the build you want to upgrade to."
    echo ""
    echo "Each build artifacts directory must contain two files:"
    echo "apq8009-robot-boot.img   - The boot/kernel image"
    echo "apq8009-robot-sysfs.ext4 - The rootfs"
    echo ""
    echo "delta_file               - The file that will hold the delta"
    echo "                           for the upgrade from the old boot"
    echo "                           and rootfs to the new ones."
}

if [ $# -ne 3 ]; then
    usage
    exit 1
fi


# truncate_file <file_path> <file_size>
#
# Truncate the given |file_path| to |file_size| using perl.
# The truncate binary might not be available.
truncate_file() {
  local file_path="$1"
  local file_size="$2"
  perl -e "open(FILE, \"+<\", \$ARGV[0]); \
           truncate(FILE, ${file_size}); \
           close(FILE);" "${file_path}"
}

BOOT_IMG_NAME=apq8009-robot-boot.img
SYS_EXT_NAME=apq8009-robot-sysfs.ext4
SYS_IMG_NAME=apq8009-robot-sysfs.img

process_build_artifact_dir() {
    pushd $1
    filesize=$(stat -c%s $BOOT_IMG_NAME)
    if [[ $(( filesize % 4096 )) -ne 0 ]]; then
	echo "Rounding UP partition $BOOT_IMG_NAME to a multiple of 4 KiB."
	: $(( filesize = (filesize + 4095) & -4096 ))
	truncate_file "${BOOT_IMG_NAME}" "${filesize}"
    fi
    if [ ! -f ${SYS_IMG_NAME} ]; then
	simg2img ${SYS_EXT_NAME} ${SYS_IMG_NAME}
    fi
    popd
}

OLDDIR=$1
NEWDIR=$2
DELTABIN=$3

if [ ! -d $OLDDIR -o ! -d $NEWDIR ]; then
    usage
    exit 1
fi

process_build_artifact_dir $OLDDIR
process_build_artifact_dir $NEWDIR

LD_LIBRARY_PATH=${SCRIPT_PATH}/lib64 PATH=${SCRIPT_PATH}/bin \
  delta_generator \
  --old_partitions=${OLDDIR}/${SYS_IMG_NAME}:${OLDDIR}/${BOOT_IMG_NAME} \
  --new_partitions=${NEWDIR}/${SYS_IMG_NAME}:${NEWDIR}/${BOOT_IMG_NAME} \
  --out_file=${DELTABIN}

