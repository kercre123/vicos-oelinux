#!/usr/bin/env python
from __future__ import print_function
"""
Example implementation of Anki Victor Update engine.
"""
__author__ = "Daniel Casner <daniel@anki.com>"

import sys
import os
import urllib2
import subprocess
import tarfile
import zlib
import ConfigParser

EMR_FILE = "/factory/birthcertificate"
BOOT_STEM = "/dev/block/bootdevice/by-name/boot"
SYSTEM_STEM = "/dev/block/bootdevice/by-name/system"
STATUS_DIR = "/data/update-engine"
EXPECTED_DOWNLOAD_SIZE_FILE = os.path.join(STATUS_DIR, "expected-download-size")
EXPECTED_WRITE_SIZE_FILE = os.path.join(STATUS_DIR, "expected-size")
PROGRESS_FILE = os.path.join(STATUS_DIR, "progress")
ERROR_FILE = os.path.join(STATUS_DIR, "error")
DONE_FILE = os.path.join(STATUS_DIR, "done")
DD_BLOCK_SIZE = 1024*256
SUPPORTED_MANIFEST_VERSIONS = ["0.9.1"]

DEBUG = False


def clear_status():
    "Clear everything out of the status directory"
    if os.path.isdir(STATUS_DIR):
        for filename in os.listdir(STATUS_DIR):
            os.remove(os.path.join(STATUS_DIR, filename))


def write_status(file_name, status):
    "Simple function to (over)write a file with a status"
    with open(file_name, "w") as output:
        output.write(str(status))
        output.truncate()


def die(code, text):
    write_status(ERROR_FILE, text)
    if DEBUG:
        sys.stderr.write(str(text))
        sys.stderr.write(os.linesep)
    exit(code)


def call(*args):
    "Simple wrapper arround subprocess.call to make ret=0 -> True"
    return subprocess.call(*args) == 0


def get_prop(property_name):
    "Gets a value from the property server via subprocess"
    getprop = subprocess.Popen(["/usr/bin/getprop", property_name], shell=False, stdout=subprocess.PIPE)
    if getprop.wait() == 0:
        return getprop.communicate()[0].strip()
    return None


def get_cmdline():
    "Returns /proc/cmdline arguments as a dict"
    cmdline = open("/proc/cmdline", "r").read()
    ret = {}
    for arg in cmdline.split(' '):
        try:
            key, val = arg.split('=')
        except ValueError:
            ret[arg] = None
        else:
            if val.isdigit():
                val = int(val)
            ret[key] = val
    return ret


def get_slot(kernel_command_line):
    "Get the current and target slots from the kernel commanlines"
    suffix = kernel_command_line.get("androidboot.slot_suffix", '_f')
    if suffix == '_a':
        return 'a', 'b'
    elif suffix == '_b':
        return 'b', 'a'
    else:
        return 'f', 'a'


def get_manifest(fileobj):
    "Returns config parsed from INI file in filelike object"
    config = ConfigParser.ConfigParser()
    config.readfp(fileobj)
    return config


def open_url_stream(url):
    "Open a URL as a filelike stream"
    try:
        if '?' in url:  # Already has a querry string
            if not url.endswith('?'):
                url += '&'
        else:
            url += '?'
        url += "emresn={0:s}&ankiversion={1:s}".format(get_prop("ro.serialno"), get_prop("ro.anki.version"))
        request = urllib2.Request(url)
        opener = urllib2.build_opener()
        return opener.open(request)
    except Exception as e:
        die(203, "Failed to open URL: " + str(e))


def make_tar_stream(fileobj):
    "Converts a file like object into a streaming tar object"
    try:
        return tarfile.open(mode='r|', fileobj=fileobj)
    except Exception as e:
        die(204, "Couldn't open contents as tar file " + str(e))


def GZStreamGenerator(fileobj, block_size, wbits):
    "A generator for decompressing gzip data from a filelike object"
    decompressor = zlib.decompressobj(wbits)
    block = fileobj.read(block_size)
    while len(block) == block_size:
        yield decompressor.decompress(block, block_size)
        block = decompressor.unconsumed_tail + fileobj.read(block_size - len(decompressor.unconsumed_tail))
    output_block = decompressor.decompress(block, block_size)
    while len(output_block) == block_size:
        yield output_block
        output_block = decompressor.decompress(decompressor.unconsumed_tail, block_size)
    yield output_block


def update_from_url(url):
    "Updates the inactive slot from the given URL"
    # Figure out slots
    cmdline = get_cmdline()
    current_slot, target_slot = get_slot(cmdline)
    if DEBUG:
        print("Target slot is", target_slot)
    # Make status directory
    if not os.path.isdir(STATUS_DIR):
        os.mkdir(STATUS_DIR)
    # Open URL as a tar stream
    stream = open_url_stream(url)
    content_length = stream.info().getheaders("Content-Length")[0]
    write_status(EXPECTED_DOWNLOAD_SIZE_FILE, content_length)
    with make_tar_stream(stream) as tar_stream:
        # Get the manifest
        if DEBUG:
            print("Manifest")
        manifest_ti = tar_stream.next()
        if not manifest_ti.name.endswith('manifest.ini'):
            die(200, "Expected manifest.ini at beginning of download, found \"{0.name}\"".format(manifest_ti))
        manifest = get_manifest(tar_stream.extractfile(manifest_ti))
        # Inspect the manifest
        if manifest.get("META", "manifest_version") not in SUPPORTED_MANIFEST_VERSIONS:
            die(201, "Unexpected manifest version")
        if manifest.getint("META", "num_images") != 2:
            die(201, "Unexpected number of images in OTA")
        try:
            total_size = manifest.getint("BOOT", "bytes") + manifest.getint("SYSTEM", "bytes")
        except Exception as e:
            print("Unable to get total bytes of download: ", e)
        else:
            write_status(EXPECTED_WRITE_SIZE_FILE, total_size)
        written_size = 0
        write_status(PROGRESS_FILE, written_size)
        if DEBUG:
            print("Updating to version {}".format(manifest.get("META", "update_version")))
        # Mark target unbootable
        if not call(['/bin/bootctl', current_slot, 'set_unbootable', target_slot]):
            die(202, "Could not mark target slot unbootable")
        # Extract boot image
        if DEBUG:
            print("Boot")
        boot_ti = tar_stream.next()
        if not boot_ti.name.endswith("boot.img.gz"):
            die(200, "Expected boot.img.gz to be next in tar but found \"{}\"".format(boot_ti.name))
        wbits = manifest.getint("BOOT", "wbits")
        with open(BOOT_STEM + "_" + target_slot, "wb") as boot_slot:
            for boot_block in GZStreamGenerator(tar_stream.extractfile(boot_ti), DD_BLOCK_SIZE, wbits):
                boot_slot.write(boot_block)
                written_size += len(boot_block)
                write_status(PROGRESS_FILE, written_size)
                if DEBUG:
                    sys.stdout.write("{0:0.3f}\r".format(float(written_size)/float(total_size)))
                    sys.stdout.flush()
        # Extract system images
        if DEBUG:
            print("System")
        system_ti = tar_stream.next()
        if not system_ti.name.endswith("sysfs.img.gz"):
            die(200, "Expected sysfs.img.gz to be next in tar but found \"{}\"".format(system_ti.name))
        wbits = manifest.getint("SYSTEM", "wbits")
        with open(SYSTEM_STEM + "_" + target_slot, "wb") as system_slot:
            for system_block in GZStreamGenerator(tar_stream.extractfile(system_ti), DD_BLOCK_SIZE, wbits):
                system_slot.write(system_block)
                written_size += len(system_block)
                write_status(PROGRESS_FILE, written_size)
                if DEBUG:
                    sys.stdout.write("{0:0.3f}\r".format(float(written_size)/float(total_size)))
                    sys.stdout.flush()
    stream.close()
    if not call(["/bin/bootctl", current_slot, "set_active", target_slot]):
        die(202, "Could not set target slot as active")
    write_status(DONE_FILE, 1)


if __name__ == '__main__':
    if len(sys.argv) == 1:  # Clear the output directory
        clear_status()
        exit(0)
    elif len(sys.argv) == 3 and sys.argv[2] == '-v':
        DEBUG = True

    if DEBUG:
        update_from_url(sys.argv[1])
    else:
        try:
            update_from_url(sys.argv[1])
        except zlib.error as decompressor_error:
            die(205, "Decompression error: " + str(decompressor_error))
        except IOError as io_error:
            die(208, "IO Error: " + str(io_error))
        except Exception as e:
            die(219, e)
        exit(0)
