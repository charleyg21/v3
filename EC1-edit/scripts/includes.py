#!/usr/env python

# Copyright © 2021 Jonathan Starr
#
# Permission is hereby granted, free of charge, to any person obtaining a copy of
# this software and associated documentation files (the “Software”), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
# IN THE SOFTWARE.

"""Get include paths from GCC."""

import os
import pathlib
import subprocess


def main():
    """Get include paths from GCC."""
    try:
        cmd = ["arm-none-eabi-gcc", "--help"]
        with open(os.devnull, "w", encoding="utf-8") as devnull:
            subprocess.check_call(cmd, stdout=devnull, stderr=devnull)
    except FileNotFoundError as exc:
        raise FileNotFoundError("arm-none-eabi-gcc needs to be installed") from exc

    options = [
        "-specs=nano.specs",
        "-mcpu=cortex-m0plus",
        "-mthumb",
        "-mfloat-abi=soft",
    ]

    cmd = ["arm-none-eabi-gcc", *options, "-E", "-Wp,-v", "-xc++", "-"]
    pipe = subprocess.PIPE
    with subprocess.Popen(cmd, stdin=pipe, stdout=pipe, stderr=pipe) as process:
        *_, stderr = process.communicate("")

    lines = stderr.decode().split("\n")
    lines = map(lambda line: line.strip(), lines)
    lines = filter(None, lines)

    paths = map(pathlib.Path, lines)
    paths = filter(lambda path: path.is_dir(), paths)

    print(";".join(map(str, paths)))


if __name__ == "__main__":
    main()
