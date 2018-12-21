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
    safe_rmtree("manifest.ini")
    safe_rmtree("manifest.sha256")
    safe_rmtree("delta.bin")
    safe_rmtree("delta.bin.gz.plaintext")
    safe_rmtree("delta.bin.gz")


def die(code, text):
    "Write out an error string and exit with given status code"
    sys.stderr.write(str(text))
    sys.stderr.write(os.linesep)
    exit(code)


def get_manifest(fileobj):
    "Returns config parsed from INI file in filelike object"
    config = configparser.ConfigParser()
    config['META'] = {'ankidev': 0,
                      'reboot_after_install': 0}
    config['BOOT'] = {'encryption': 0}
    config['SYSTEM'] = {'encryption': 0}
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

def decrypt_file(ciphertext_path, plaintext_path, key_path):
    "Use openssl to decrypt a file"
    openssl = subprocess.Popen(["/usr/bin/openssl",
                                "enc",
                                "-d",
                                "-aes-256-ctr",
                                "-pass",
                                "file:{0}".format(key_path),
                                "-in",
                                "{0}".format(ciphertext_path),
                                "-out",
                                "{0}".format(plaintext_path)],
                               shell=False,
                               stdout=subprocess.PIPE,
                               stderr=subprocess.PIPE)
    ret_code = openssl.wait()
    openssl_out, openssl_err = openssl.communicate()
    return ret_code == 0, ret_code, openssl_out, openssl_err

def extract_full_ota(name, tmpdir, private_pass):
    safe_rmtree(tmpdir)
    os.makedirs(tmpdir)
    with make_tar_stream(name) as tar_stream:
        tar_stream.extractall(tmpdir)
    manifest = get_manifest(open(os.path.join(tmpdir, "manifest.ini"), "r"))
    encryption = int(manifest.get("BOOT", "encryption"))
    boot_img_gz = os.path.join(tmpdir, 'apq8009-robot-boot.img.gz')
    if encryption == 1:
        ciphertext = boot_img_gz
        boot_img_gz = os.path.join(tmpdir, "boot.img.gz")
        decrypt_status = decrypt_file(ciphertext, boot_img_gz, private_pass)
        if not decrypt_status[0]:
            die(219,
                "Failed to decrypt boot image, openssl returned {}\n{}\n{}"
                .format(decrypt_status[1],
                        decrypt_status[2].decode("utf-8"),
                        decrypt_status[3].decode("utf-8")))
    boot_img = os.path.join(tmpdir, 'boot.img')
    with gzip.open(boot_img_gz, 'rb') as f_in, open(boot_img, 'wb') as f_out:
        shutil.copyfileobj(f_in, f_out)
    sys_img_gz = os.path.join(tmpdir, 'apq8009-robot-sysfs.img.gz')
    encryption = int(manifest.get("SYSTEM", "encryption"))
    if encryption == 1:
        ciphertext = sys_img_gz
        sys_img_gz = os.path.join(tmpdir, "system.img.gz")
        decrypt_status = decrypt_file(ciphertext, sys_img_gz, private_pass)
        if not decrypt_status[0]:
            die(219,
                "Failed to decrypt system image, openssl returned {}\n{}\n{}"
                .format(decrypt_status[1],
                        decrypt_status[2].decode("utf-8"),
                        decrypt_status[3].decode("utf-8")))
    sys_img = os.path.join(tmpdir, 'system.img')
    with gzip.open(sys_img_gz, 'rb') as f_in, open(sys_img, 'wb') as f_out:
        shutil.copyfileobj(f_in, f_out)
    return manifest.get("META", "update_version")


def create_signature(file_path_name, sig_path_name, private_key, private_key_pass):
    "Create the signature of a file based on a key"
    openssl = subprocess.Popen(["/usr/bin/openssl",
                                "dgst",
                                "-sha256",
                                "-sign",
                                private_key,
                                "-passin",
                                "pass:{}".format(private_key_pass),
                                "-out",
                                sig_path_name,
                                file_path_name],
                               shell=False,
                               stdout=subprocess.PIPE,
                               stderr=subprocess.PIPE)
    ret_code = openssl.wait()
    openssl_out, openssl_err = openssl.communicate()
    return ret_code == 0, ret_code, openssl_out, openssl_err


def encrypt_file(plaintext_path, ciphertext_path, key_path):
    "Encrypt the file using the key"
    openssl = subprocess.Popen(["/usr/bin/openssl",
                                "aes-256-ctr",
                                "-pass",
                                "file:{0}".format(key_path),
                                "-in",
                                "{0}".format(plaintext_path),
                                "-out",
                                "{0}".format(ciphertext_path)],
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
parser.add_argument('--out', action='store', default=None, required=False,
                    help="Path to hold the delta OTA")
parser.add_argument('--reboot-after-install', action='store', default=None, required=False,
                    help="Override value of reboot_after_install")
parser.add_argument('--ota-pass', action='store', default=None, required=False,
                    help="Path to file with private ota password")
parser.add_argument('--ota-key', action='store', default=None, required=False,
                    help="Path to file with private key")
parser.add_argument('--ota-key-pass', action='store', default=None, required=False,
                    help="Passphrase for private key")
parser.add_argument('--max-delta-size', type=int, action='store', default=None, required=False,
                    help="If the delta OTA file is larger than this, just use a full OTA")
parser.add_argument('--verbose', '-v', action='store_true', default=False)
args = parser.parse_args()

cleanup_working_dirs()

private_pass = os.getenv('OTAPASS',
                         os.path.join(TOPLEVEL, "ota", "ota_test.pass"))

if args.ota_pass:
    private_pass = args.ota_pass


old_manifest_ver = extract_full_ota(args.old, "old", private_pass)
new_manifest_ver = extract_full_ota(args.new, "new", private_pass)
new_manifest = get_manifest(open(os.path.join("new", "manifest.ini"), "r"))
ankidev = new_manifest.get("META", "ankidev")

reboot_after_install = args.reboot_after_install
if reboot_after_install == None:
    reboot_after_install = new_manifest.get("META", "reboot_after_install")

default_delta_ota_name = "vicos-{0}_to_{1}.ota".format(old_manifest_ver,
                                                       new_manifest_ver)

delta_ota_name = args.out
if delta_ota_name and os.path.isdir(delta_ota_name):
    delta_ota_name = os.path.join(delta_ota_name, default_delta_ota_name)
if not delta_ota_name:
    delta_ota_name = default_delta_ota_name

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

with open('delta.bin', 'rb') as f_in, \
     gzip.open('delta.bin.gz.plaintext', 'wb') as f_out:
    shutil.copyfileobj(f_in, f_out)

encrypt_status = encrypt_file('delta.bin.gz.plaintext',
                              'delta.bin.gz',
                              private_pass)
if not encrypt_status[0]:
    die(219,
        "Failed to encrypt delta payload, openssl returned {}\n{}\n{}"
        .format(encrypt_status[1],
                encrypt_status[2].decode("utf-8"),
                encrypt_status[3].decode("utf-8")))

delta_manifest = configparser.ConfigParser()
delta_manifest['META'] = {'manifest_version': '1.0.0',
                          'update_version': new_manifest_ver,
                          'ankidev': ankidev,
                          'reboot_after_install': reboot_after_install,
                          'num_images': '1'}
delta_manifest['DELTA'] = {'base_version': old_manifest_ver,
                           'compression': 'gz',
                           'encryption': '1',
                           'wbits': '31',
                           'bytes': os.path.getsize('delta.bin'),
                           'sha256': sha256_file('delta.bin'),
                           'system_size':
                           os.path.getsize(os.path.join("new", "system.img")),
                           'boot_size':
                           os.path.getsize(os.path.join("new", "boot.img"))}

with open("manifest.ini", "w") as manifest:
    delta_manifest.write(manifest)

private_key = os.getenv('OTAKEY',
                        os.path.join(TOPLEVEL, "ota", "ota_prod.key"))
if args.ota_key:
    private_key = args.ota_key

private_key_pass = os.getenv('OTA_KEY_PASS', "")
if args.ota_key_pass:
    private_key_pass = args.ota_key_pass

create_signature_status = create_signature("manifest.ini",
                                           "manifest.sha256",
                                           private_key,
                                           private_key_pass)

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

if args.max_delta_size and os.path.getsize(delta_ota_name) > args.max_delta_size:
    # This delta OTA file is too big, just use the new OTA file for a full update
    if args.verbose:
        print("Delta OTA file is too big ({0} > {1}). Will substitute a full OTA instead.".format(os.path.getsize(delta_ota_name), args.max_delta_size))
    os.remove(delta_ota_name)
    shutil.copyfile(args.new, delta_ota_name)

cleanup_working_dirs()
