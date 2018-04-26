#!/usr/bin/env python3

from __future__ import print_function
"""
Generate a delta .ota file from 2 full .ota files.
Several methods taken or modified from update-engine
(Daniel Casner <daniel@anki.com>)
"""
__author__ = "Stuart Eichert <seichert@anki.com>"

import argparse
import sys
import os
import shutil
import subprocess
import tarfile
import gzip
import configparser
from hashlib import sha256

SCRIPT_PATH = os.path.dirname(os.path.realpath(__file__))
TOPLEVEL = os.path.dirname(os.path.dirname(SCRIPT_PATH))
DD_BLOCK_SIZE = 1024*256


def safe_rmtree(name):
    if os.path.isfile(name):
        os.remove(name)
    elif os.path.isdir(name):
        shutil.rmtree(name)


def cleanup_working_dirs():
    safe_rmtree("old")
    safe_rmtree("new")
    os.remove("manifest.ini")
    os.remove("manifest.sha256")
    os.remove("delta.bin")
    os.remove("delta.bin.gz")


def die(code, text):
    "Write out an error string and exit with given status code"
    sys.stderr.write(str(text))
    sys.stderr.write(os.linesep)
    exit(code)


def get_manifest(fileobj):
    "Returns config parsed from INI file in filelike object"
    config = configparser.ConfigParser()
    config.readfp(fileobj)
    return config


def make_tar_stream(name):
    "Open a file as a tar stream"
    try:
        return tarfile.open(name=name)
    except Exception as e:
        die(204, "Couldn't open {0} as tar file ".format(name) + str(e))


def sha256_file(name):
    "Calculate the sha256 hash of a file"
    digest = sha256()
    with open(name, 'rb') as f:
        for block in iter(lambda: f.read(65536), b''):
            digest.update(block)
    return digest.hexdigest()


def extract_full_ota(name, tmpdir):
    safe_rmtree(tmpdir)
    os.makedirs(tmpdir)
    with make_tar_stream(name) as tar_stream:
        tar_stream.extractall(tmpdir)
    boot_img_gz = os.path.join(tmpdir, 'apq8009-robot-boot.img.gz')
    boot_img = os.path.join(tmpdir, 'boot.img')
    with gzip.open(boot_img_gz, 'rb') as f_in, open(boot_img, 'wb') as f_out:
        shutil.copyfileobj(f_in, f_out)
    sys_img_gz = os.path.join(tmpdir, 'apq8009-robot-sysfs.img.gz')
    sys_img = os.path.join(tmpdir, 'system.img')
    with gzip.open(sys_img_gz, 'rb') as f_in, open(sys_img, 'wb') as f_out:
        shutil.copyfileobj(f_in, f_out)


def create_signature(file_path_name, sig_path_name, private_key):
    "Create the signature of a file based on a key"
    openssl = subprocess.Popen(["/usr/bin/openssl",
                                "dgst",
                                "-sha256",
                                "-sign",
                                private_key,
                                "-out",
                                sig_path_name,
                                file_path_name],
                               shell=False,
                               stdout=subprocess.PIPE,
                               stderr=subprocess.PIPE)
    ret_code = openssl.wait()
    openssl_out, openssl_err = openssl.communicate()
    return ret_code == 0, ret_code, openssl_out, openssl_err


parser = argparse.ArgumentParser(description="Create a delta .ota file "
                                 "from 2 full .ota files.",
                                 formatter_class=argparse
                                 .ArgumentDefaultsHelpFormatter)
parser.add_argument('--old', action='store', required=True,
                    help="File with the old version to update from")
parser.add_argument('--new', action='store', required=True,
                    help="File with the new version to update to")
args = parser.parse_args()

extract_full_ota(args.old, "old")
extract_full_ota(args.new, "new")

old_manifest = get_manifest(open(os.path.join("old", "manifest.ini"), "r"))
new_manifest = get_manifest(open(os.path.join("new", "manifest.ini"), "r"))

old_manifest_ver = old_manifest.get("META", "update_version")
new_manifest_ver = new_manifest.get("META", "update_version")

delta_ota_name = "vicos-{0}_to_{1}.ota".format(old_manifest_ver,
                                               new_manifest_ver)

delta_env = os.environ.copy()
delta_env["LD_LIBRARY_PATH"] = os.path.join(SCRIPT_PATH, "lib64")
delta_env["PATH"] = os.path.join(SCRIPT_PATH, "bin")

delta_gen = subprocess.Popen(["delta_generator",
                              "--old_partitions={0}:{1}"
                              .format(os.path.join("old", "system.img"),
                                      os.path.join("old", "boot.img")),
                              "--new_partitions={0}:{1}"
                              .format(os.path.join("new", "system.img"),
                                      os.path.join("new", "boot.img")),
                              "--out_file=delta.bin"],
                             shell=False,
                             stdout=subprocess.PIPE,
                             stderr=subprocess.PIPE,
                             env=delta_env)
delta_gen_out, delta_gen_err = delta_gen.communicate()
if delta_gen.returncode != 0:
    die(219,
        "Delta generation failed \n{}\n{}"
        .format(delta_gen_out.decode("utf-8"),
                delta_gen_err.decode("utf-8")))

with open('delta.bin', 'rb') as f_in, gzip.open('delta.bin.gz', 'wb') as f_out:
    shutil.copyfileobj(f_in, f_out)

delta_manifest = configparser.ConfigParser()
delta_manifest['META'] = {'manifest_version': '0.9.4',
                          'update_version': new_manifest_ver,
                          'num_images': '1'}
delta_manifest['DELTA'] = {'base_version': old_manifest_ver,
                           'compression': 'gz',
                           'wbits': '31',
                           'bytes': os.path.getsize('delta.bin.gz'),
                           'sha256': sha256_file('delta.bin.gz'),
                           'system_size':
                           os.path.getsize(os.path.join("new", "system.img")),
                           'boot_size':
                           os.path.getsize(os.path.join("new", "boot.img"))}

with open("manifest.ini", "w") as manifest:
    delta_manifest.write(manifest)

private_key = os.getenv('OTAKEY',
                        os.path.join(TOPLEVEL, "ota", "ota_test.key"))
create_signature_status = create_signature("manifest.ini",
                                           "manifest.sha256",
                                           private_key)

if not create_signature_status[0]:
    die(219,
        "Failed to sign manifest.ini, openssl returned {}\n{}\n{}"
        .format(create_signature_status[1],
                create_signature_status[2].decode("utf-8"),
                create_signature_status[3].decode("utf-8")))

ota = tarfile.open(delta_ota_name, "w")
namelist = ["manifest.ini",
            "manifest.sha256",
            "delta.bin.gz"]
for name in namelist:
    tarinfo = ota.gettarinfo(name)
    tarinfo.uid = 0
    tarinfo.gid = 0
    tarinfo.uname = "root"
    tarinfo.gname = "root"
    tarinfo.mode = 0o0400
    ota.addfile(tarinfo, fileobj=open(name, 'rb'))
ota.close()
cleanup_working_dirs()
