# This class provides utilities to generate and append verity metadata
# into images as required by device-mapper-verity feature.

VERITY_ENABLED = "${@bb.utils.contains('DISTRO_FEATURES', 'dm-verity', '1', '0', d)}"
DEPENDS += " ${@bb.utils.contains('VERITY_ENABLED', '1', 'verity-utils-native', '', d)}"

FIXED_SALT = "aee087a5be3b982978c923f566a94613496b417f2af592639bc80d141e34dfe7"
BLOCK_SIZE = "4096"
BLOCK_DEVICE_SYSTEM ?= "/dev/block/bootdevice/by-name/system"
ORG_SYSTEM_SIZE_EXT4 = "0"
VERITY_SIZE = "0"
FEC_SUPPORT = "0"

VERITY_IMAGE_DIR     = "${DEPLOY_DIR_IMAGE}/verity"
SPARSE_SYSTEM_IMG    = "${DEPLOY_DIR_IMAGE}/${MACHINE}-sysfs.ext4"
VERITY_IMG           = "${VERITY_IMAGE_DIR}/${MACHINE}-verity.img"
VERITY_METADATA_IMG  = "${VERITY_IMAGE_DIR}/${MACHINE}-verity-metadata.img"
VERITY_FEC_IMG       = "${VERITY_IMAGE_DIR}/${MACHINE}-verity-fec.img"

python adjust_system_size_for_verity () {
    partition_size = int(d.getVar("SYSTEM_SIZE_EXT4",True))
    block_size = int(d.getVar("BLOCK_SIZE",True))
    fec_support = d.getVar("FEC_SUPPORT",True)
    hi = partition_size
    if hi % block_size != 0:
        hi = (hi // block_size) * block_size
    verity_size = get_verity_size(d, hi, fec_support)
    lo = partition_size - verity_size
    result = lo
    while lo < hi:
        i = ((lo + hi) // (2 * block_size)) * block_size
        v = get_verity_size(d, i, fec_support)
        if i + v <= partition_size:
            if result < i:
                result = i
                verity_size = v
            lo = i + block_size
        else:
            hi = i

    d.setVar('SYSTEM_SIZE_EXT4', str(result))
    d.setVar('VERITY_SIZE', str(verity_size))
    d.setVar('ORG_SYSTEM_SIZE_EXT4', str(partition_size))
    bb.warn("Old system partition size: %s" % d.getVar("ORG_SYSTEM_SIZE_EXT4",True))
    bb.warn("New system partition size: %s" % d.getVar("SYSTEM_SIZE_EXT4",True))
    bb.warn("verity size is: %s" % d.getVar("VERITY_SIZE",True))

}

def get_verity_size(d, partition_size, fec_support):
    import subprocess

    # Get verity tree size
    bvt_bin_path = d.getVar('STAGING_BINDIR_NATIVE', True) + '/build_verity_tree'
    cmd = bvt_bin_path + " -s %s " % partition_size
    try:
        verity_tree_size =  int(subprocess.check_output(cmd, shell=True).strip())
    except subprocess.CalledProcessError as e:
        bb.fatal("Error in calculating verity tree size:\n%s" % e.output)

    # Get verity metadata size
    bvmd_script_path = d.getVar('STAGING_BINDIR_NATIVE', True) + '/build_verity_metadata.py'
    cmd = bvmd_script_path + " size %s " % partition_size
    try:
        verity_metadata_size = int(subprocess.check_output(cmd, shell=True).strip())
    except subprocess.CalledProcessError as e:
        bb.fatal("Error in calculating verity metadata size:\n%s" % e.output)

    verity_size = verity_tree_size + verity_metadata_size

    # Get fec size
    if fec_support is "1":
        fec_bin_path = d.getVar('STAGING_BINDIR_NATIVE', True) + '/fec'
        cmd = fec_bin_path + " -s %s " % (partition_size + verity_size)
        try:
            fec_size =  int(subprocess.check_output(cmd, shell=True).strip())
        except subprocess.CalledProcessError as e:
            bb.fatal("Error in calculating fec size:\n%s" % e.output)
        return verity_size + fec_size
    return verity_size

make_verity_enabled_system_image[cleandirs] += " ${VERITY_IMAGE_DIR}"

python make_verity_enabled_system_image () {
    import subprocess

    sparse_img = d.getVar('SPARSE_SYSTEM_IMG', True)
    verity_img = d.getVar('VERITY_IMG', True)
    verity_md_img = d.getVar('VERITY_METADATA_IMG', True)
    signer_path = "${STAGING_BINDIR_NATIVE}/verity_signer"
    signer_key = "${TMPDIR}/work-shared/security_tools/verity.pk8"

    # Build verity tree
    bvt_bin_path = d.getVar('STAGING_BINDIR_NATIVE', True) + '/build_verity_tree'
    cmd = bvt_bin_path + " -A %s %s %s " % (d.getVar("FIXED_SALT",True), sparse_img, verity_img)
    try:
        [root_hash, salt] = (subprocess.check_output(cmd, shell=True)).split()
    except subprocess.CalledProcessError as e:
        bb.fatal("Error in building verity tree :\n%s" % e.output)
    bb.warn("Value of root hash is %s" % root_hash)
    bb.warn("Value of salt is %s" % salt)

    # Build verity metadata
    blk_dev = d.getVar("BLOCK_DEVICE_SYSTEM",True)
    image_size = d.getVar("SYSTEM_SIZE_EXT4",True)
    bvmd_script_path = d.getVar('STAGING_BINDIR_NATIVE', True) + '/build_verity_metadata.py'
    cmd = bvmd_script_path + " build %s %s %s %s %s %s %s " % (image_size, verity_md_img, root_hash, salt, blk_dev, signer_path, signer_key)
    subprocess.call(cmd, shell=True)

    # Append verity metadata to verity image.
    bb.warn("appending verity_md_img to verity_img .... ")
    with open(verity_img, "a") as out_file:
        with open(verity_md_img, "r") as input_file:
            for line in input_file:
                out_file.write(line)

    # Calculate padding.
    partition_size = int(d.getVar("ORG_SYSTEM_SIZE_EXT4",True))
    img_size = int(d.getVar("SYSTEM_SIZE_EXT4",True))
    verity_size = int(d.getVar("VERITY_SIZE",True))
    padding_size = partition_size - img_size - verity_size
    assert padding_size >= 0

    fec_supported=d.getVar("FEC_SUPPORT",True)
    if fec_supported is "1":
        fec_bin_path = d.getVar('STAGING_BINDIR_NATIVE', True) + '/fec'
        fec_img_path = d.getVar('VERITY_FEC_IMG', True)
        cmd = fec_bin_path + " -e -p %s %s %s %s" % (padding_size, sparse_img, verity_img, fec_img_path)
        subprocess.call(cmd, shell=True)

        bb.warn("appending fec_img_path to verity_img.... ")
        with open(verity_img, "a") as out_file:
            with open(fec_img_path, "r") as input_file:
                for line in input_file:
                    out_file.write(line)

    # All done. Append verity img to sparse system img.
    append2simg_path = d.getVar('STAGING_BINDIR_NATIVE', True) + '/append2simg'
    cmd = append2simg_path + " %s %s " % (sparse_img, verity_img)
    subprocess.call(cmd, shell=True)
}

# Tasks to alter system image if varity is enabled.
VERITYPREFUNCS = " ${@bb.utils.contains('VERITY_ENABLED', '1', 'adjust_system_size_for_verity', '', d)}"
do_makesystem[prefuncs] += " ${VERITYPREFUNCS}"

VERITYPOSTFUNCS = " ${@bb.utils.contains('VERITY_ENABLED', '1', 'make_verity_enabled_system_image', '', d)}"
do_makesystem[postfuncs] += " ${VERITYPOSTFUNCS}"
