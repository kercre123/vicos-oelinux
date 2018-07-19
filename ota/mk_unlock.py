#!/usr/bin/env python3
"""Utility script for generating aboot images locked to specific QSN ids."""
__author__ = "Daniel Casner <daniel@anki.com>"

import os
import argparse
import subprocess
from getpass import getpass
try:
    import pexpect
except ModuleNotFoundError:
    print("pexpect not installed, suggest `pip3 install pexpect`")

SECTOOLS_DEFAULT = ('..', '..', 'common', 'tools', 'sectools')
BITBAKE_DEFAULT = ('..', 'poky', 'build')
ABOOT_BUILD_PATH = ('tmp-glibc', 'deploy', 'images', 'apq8009-robot-robot-perf', 'emmc_appsboot.mbn')
SECTOOLS_OUTPUT = ('secimage_output', '8909', 'appsbl')
OUTPUT_DEFAULT = ('..', '_build', 'unlock')
OTA_BUILD_DIR = ('..', '_build')

def make_aboot(qsn, build_dir, out_dir):
    """Use bitbake to make an aboot image for a given QSN"""
    environ = dict(os.environ)
    environ['QSN'] = qsn
    proc = subprocess.Popen(['bitbake', '-c', 'cleanall', 'lk'], cwd=build_dir, env=environ)
    assert proc.wait() == 0
    proc = subprocess.Popen(['bitbake', 'lk'], cwd=build_dir, env=environ)
    assert proc.wait() == 0
    output = os.path.join(out_dir, "{}.img".format(qsn))
    os.rename(os.path.join(build_dir, *ABOOT_BUILD_PATH), output)
    return output


def parse_list(qsn_list_fh):
    "Yields QSNs from a file assuming first word of each line is the QSN"
    for line in qsn_list_fh:
        yield line.split()[0]


def sign(aboot_path, sectools_path, password=None):
    "Sign the aboot image"
    cmd = ['python', 'sectools.py', 'secimage', '-s', '-i', aboot_path,
           '-g', 'appsbl', '-p', '8909', '--cfg_selected_cert_config=OEM-KEYS']
    if not password:
        proc = subprocess.Popen(cmd, cwd=sectools_path)
        assert proc.wait() == 0
    else:
        proc = pexpect.spawn(cmd[0], cmd[1:], cwd=sectools_path)
        for _ in range(3): # Why do we have to enter it 3 times? Who knows but it asks three times
            proc.expect('Enter pass phrase for.+')
            proc.sendline(password)
        assert proc.wait() == 0
    os.rename(os.path.join(sectools_path, *SECTOOLS_OUTPUT, os.path.split(aboot_path)[1]), aboot_path)
    return aboot_path


def make_ota(qsn, aboot_path, ota_dir, ota_key_pass, out_dir):
    "Make OTA file packaged with new OTA file"
    ota_build_dir = os.path.join(ota_dir, *OTA_BUILD_DIR)
    os.rename(aboot_path, os.path.join(ota_build_dir, 'emmc_appsboot.img'))
    proc = subprocess.Popen(['make', '-f', 'unlock.mk', 'ANKIDEV=0',
                             'QSN={}'.format(qsn),
                             'OTA_KEY_PASS={}'.format(ota_key_pass)],
                            cwd=ota_dir)
    assert proc.wait() == 0
    ota_file_name = "vicos-unlock-{}.ota".format(qsn)
    ota_path_name = os.path.join(out_dir, ota_file_name)
    os.rename(os.path.join(ota_build_dir, ota_file_name), ota_path_name)
    return ota_path_name


def main():
    "Script entry point"
    parser = argparse.ArgumentParser()
    parser.add_argument("-q", "--qsn",
                        help="Build for a specific QSN")
    parser.add_argument("-l", "--list", type=argparse.FileType("rt"),
                        help="Build aboots for a list of QSNs.")
    parser.add_argument('-C', '--bitbake_build', default=os.path.join(*BITBAKE_DEFAULT),
                        help="Bitbake build folder")
    parser.add_argument('--sectools', default=os.path.join(*SECTOOLS_DEFAULT))
    parser.add_argument('--ota_dir', default=os.path.curdir)
    parser.add_argument('-s', '--sign', action="store_true",
                        help="Sign the generated images")
    parser.add_argument('-m', '--make_ota', action="store_true",
                        help="Also make OTA file")
    parser.add_argument("-o", "--out", default=os.path.join(*OUTPUT_DEFAULT),
                        help="Directory to put the generated aboot images in.")
    args = parser.parse_args()

    if not os.path.isdir(args.out):
        os.makedirs(args.out)
    out_dir = os.path.abspath(args.out)

    if args.make_ota:
        ota_key_pass = getpass("OTA key password: ")

    if args.sign:
        try:
            assert pexpect
            sign_password = getpass("Aboot signing password: ")
        except (NameError, KeyboardInterrupt): # Either no pexpect module or user ctrl+C'd out of prompt
            sign_password = None

    if args.qsn:
        aboot = make_aboot(args.qsn, args.bitbake_build, out_dir)
        if args.sign:
            sign(aboot, args.sectools, sign_password)
        if args.make_ota:
            make_ota(args.qsn, aboot, args.ota_dir, ota_key_pass, out_dir)

    if args.list:
        for qsn in parse_list(args.list):
            print(qsn, "...")
            aboot = make_aboot(qsn, args.bitbake_build, out_dir)
            if args.sign:
                sign(aboot, args.sectools, sign_password)
            if args.make_ota:
                ota = make_ota(qsn, aboot, args.ota_dir, ota_key_pass, out_dir)
                print("\t", ota)


if __name__ == '__main__':
    main()
