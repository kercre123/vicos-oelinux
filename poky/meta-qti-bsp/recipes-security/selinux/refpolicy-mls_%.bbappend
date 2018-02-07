FILESEXTRAPATHS_append := "${THISDIR}:${THISDIR}/files:"

SRCREV_refpolicy := "RELEASE_2_20161023"
SRCREV_refpolicy-contrib = "RELEASE_2_20161023"

SRC_URI += "file://Remove-MLS-constraints.patch \ 
            file://0001-add-monolithic-policy.patch \
            file://0001-Remove-mls-relabelto.patch \
            file://Assign-proc_t-to-proc-dir.patch \
            file://0004-selinux-update-the-getty.te-for-addressing.patch \
            file://mls-enabling-sysadmin-role-for-the-ssh-login.patch \
            file://policy/ \
            file://append/ \
"

do_patch_append() {
    # Move all policies declared in the meta layer to the expected locations
    import shutil
    for dir in os.listdir("${WORKDIR}/policy"):
        shutil.move("${WORKDIR}/policy/" + dir, "${S}/policy/modules/" + dir)
    shutil.rmtree("${WORKDIR}/policy")

    # Append policy fragments to upstream policy definitions
    append_dir = "${WORKDIR}/append/"
    for dir in os.listdir(append_dir):
        for policy_file in os.listdir(append_dir + dir):
            try:
                with open("${S}/policy/modules/" + dir + "/" + policy_file, 'a') as orig_policy:
                    with open(append_dir + dir + "/" + policy_file, 'r') as policy_append:
                        orig_policy.write('\n')
                        for line in policy_append.readlines():
                            orig_policy.write(line)
            except Exception:
                sys.stderr.write("Could not append to \"" + policy_file + "\"")
    shutil.rmtree(append_dir)
}
