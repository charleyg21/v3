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

"""Wrapper for generating projects using STM32CubeMX."""

import argparse
import contextlib
import itertools
import os
import pathlib
import shutil
import subprocess
import tempfile
import textwrap
import typing


@contextlib.contextmanager
def temporary_filename(**kwargs):
    """Get a named temporary file."""
    try:
        tmp = tempfile.NamedTemporaryFile(**kwargs, delete=False)
        tmp.close()
        yield pathlib.Path(tmp.name)
    finally:
        os.unlink(tmp.name)


@contextlib.contextmanager
def temporary_directory(**kwargs):
    """Get a named temporary directory."""
    with tempfile.TemporaryDirectory(**kwargs) as tmp:
        yield pathlib.Path(tmp)


def write_cubemx_script(filename, output_directory):
    """Write the cubemx script file."""
    script = textwrap.dedent(
        f"""\
        project path {output_directory}
        project generate
        exit\
    """
    )
    with open(filename, "w", encoding="utf-8") as out_file:
        out_file.write(script)


def copy_cubemx_project(src, dst):
    """Copy a cubemx project from one directory to another."""
    src = pathlib.Path(src)
    dst = pathlib.Path(dst)

    walk = os.walk(src)

    try:
        dirpath, dirnames, filenames = next(walk)
    except StopIteration:
        return

    def is_wanted_dirname(dirname):
        return dirname not in {"MXTmpFiles"}

    dirnames = list(filter(is_wanted_dirname, dirnames))

    def is_wanted_filename(filename):
        unwanted = {"Makefile", ".mxproject"}
        return not filename.endswith(".ioc") and filename not in unwanted

    filenames = list(filter(is_wanted_filename, filenames))

    walk_item = (dirpath, dirnames, filenames)
    walk = itertools.chain((walk_item,), walk)

    for dirpath, dirnames, filenames in walk:
        dirpath = pathlib.Path(dirpath)
        relpath = dirpath.relative_to(src)

        for dirname in dirnames:
            dstpath = dst.joinpath(relpath, dirname)
            try:
                os.mkdir(dstpath)
            except FileExistsError:
                pass

        for filename in filenames:
            srcpath = src.joinpath(relpath, filename)
            dstpath = dst.joinpath(relpath, filename)
            shutil.copy2(srcpath, dstpath)
            dstpath.touch()


def is_source_file(filepath: pathlib.Path) -> bool:
    """Return whether or not the given filepath is a source file."""
    _, ext = os.path.splitext(filepath)
    return ext in {".c", ".h"}


def iterate_files(
    directory: pathlib.Path,
) -> typing.Generator[pathlib.Path, None, None]:
    """Recursively iterate through the files of a directory."""
    for dirpath, _, filenames in os.walk(directory):
        path = pathlib.Path(dirpath)
        for filename in filenames:
            yield path.joinpath(filename)


def run_dos2unix(directory: pathlib.Path) -> None:
    """Run dos2unix on the files in a directory."""
    with open(os.devnull, "w", encoding="utf-8") as devnull:
        try:
            cmd = ["dos2unix", "--help"]
            subprocess.check_call(cmd, stdout=devnull, stderr=devnull)
        except FileNotFoundError as exc:
            raise FileNotFoundError("dos2unix needs to be installed") from exc

        for filepath in iterate_files(directory):
            cmd = ["dos2unix", str(filepath)]
            subprocess.check_call(cmd, stdout=devnull, stderr=devnull)


def run_clang_format(directory: pathlib.Path) -> None:
    """Run clang format on the C source files in a directory."""
    with open(os.devnull, "w", encoding="utf-8") as devnull:
        try:
            cmd = ["clang-format", "--help"]
            subprocess.check_call(cmd, stdout=devnull, stderr=devnull)
        except FileNotFoundError as exc:
            raise FileNotFoundError("clang-format needs to be installed") from exc

        style = "{BasedOnStyle: WebKit, ColumnLimit: 88, SortIncludes: false}"
        filepaths = iterate_files(directory)
        source_filepaths = filter(is_source_file, filepaths)
        for filepath in source_filepaths:
            cmd = ["clang-format", "-i", f"-style={style}", str(filepath)]
            subprocess.check_call(cmd)


def main():
    """Generate a cubemx project given an .ioc file."""
    parser = argparse.ArgumentParser(description=main.__doc__)
    parser.add_argument("config_file")
    parser.add_argument("output_directory")
    args = parser.parse_args()

    config_file = pathlib.Path(args.config_file)
    if not config_file.is_file():
        raise ValueError("specified config file is not a file")

    output_directory = pathlib.Path(args.output_directory)
    if not output_directory.is_dir():
        raise ValueError("specified output directory is not a directory")

    try:
        cmd = ["xvfb-run", "--help"]
        with open(os.devnull, "w", encoding="utf-8") as devnull:
            subprocess.check_call(cmd, stdout=devnull, stderr=devnull)
    except FileNotFoundError as exc:
        raise FileNotFoundError("Xvfb needs to be installed") from exc

    with (
        temporary_filename(suffix=".ioc") as tmp_config_file,
        temporary_filename(suffix=".script.txt") as script_file,
        temporary_directory() as tmp_output_directory,
    ):
        shutil.copy2(config_file, tmp_config_file)
        config_file = tmp_config_file

        write_cubemx_script(script_file, tmp_output_directory)

        try:
            cmd = ["xvfb-run", "stm32cubemx", "-s", script_file, config_file]
            with open(os.devnull, "w", encoding="utf-8") as devnull:
                subprocess.check_call(cmd, stdout=devnull)
        except FileNotFoundError as exc:
            raise FileNotFoundError("stm32cubemx needs to be installed") from exc

        base_config_name, *_ = os.path.splitext(config_file.name)
        tmp_output_directory = tmp_output_directory.joinpath(base_config_name)
        run_dos2unix(tmp_output_directory)
        run_clang_format(tmp_output_directory)
        copy_cubemx_project(tmp_output_directory, output_directory)


if __name__ == "__main__":
    main()
