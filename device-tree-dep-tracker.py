#!/usr/bin/env python3
"""
A dumb python script to follow device tree includes"
"""
__author__ = "Daniel Casner <daniel@anki.com>"

import sys
import os
import re

INCLUDE_RE = re.compile(r'#include\s"(.+)"')


def walk_includes(todo_list, indent_level=0):
    "Recursive function to walk includes"
    for file_name in todo_list:
        if os.path.isfile(file_name):
            sys.stdout.write("{}{}{}".format(" "*indent_level, file_name, os.linesep))
            walk_includes(INCLUDE_RE.findall(open(file_name, 'rt').read()), indent_level+2)
        else:
            sys.stderr.write("Missing file: {}{}".format(file_name, os.linesep))


if __name__ == "__main__":
    walk_includes([sys.argv[1]])
