#!/usr/bin/env python

"""Generate source code from a menu hierarchy."""

import argparse
import contextlib
import itertools
import json
import pathlib

from argparse import FileType
from contextlib import contextmanager
from dataclasses import dataclass, field
from typing import ForwardRef, Optional, Tuple

import jsonschema  # type: ignore

# pylint: disable=invalid-name


@dataclass
class MenuNode:  # pylint: disable=too-many-instance-attributes
    """Object for modeling node of hierarchical menu."""

    name: str
    lines: Tuple[str, str]

    parent: Optional[ForwardRef("MenuNode")] = field(default=None)  # type: ignore
    child: Optional[ForwardRef("MenuNode")] = field(default=None)  # type: ignore

    up: ForwardRef("MenuNode") = field(default=None)  # type: ignore
    down: ForwardRef("MenuNode") = field(default=None)  # type: ignore

    callback: Optional[str] = field(default=None)


def get_local_file_path(a_file_name):
    """Get the absolute path for a file contained in same directory as script."""
    return pathlib.Path(__file__).absolute().parent.joinpath(a_file_name)


@contextlib.contextmanager
def open_input_file_or_file_name(a_file_or_file_name):
    """Safely open an input file regardless of whether it has already been opened."""
    try:
        # Test if this is an already-opened file.
        # pylint: disable=pointless-statement
        a_file_or_file_name.read
        a_file_or_file_name.write
    except AttributeError:
        # If it isn't, open it and yield it to the context handler.
        try:
            a_file = open(a_file_or_file_name, "r")
            yield a_file
        finally:
            a_file.close()
    else:
        # Otherwise, just yield it back to the context handler.
        yield a_file_or_file_name


def read_json_data(a_file_or_file_name):
    """Load a JSON file while ignoring comment lines."""

    def not_a_comment(line):
        return not line.lstrip().startswith("//")  # Flag lines starting with "//".

    with open_input_file_or_file_name(a_file_or_file_name) as a_file:
        a_json_str = "".join(line for line in a_file if not_a_comment(line))
        return json.loads(a_json_str)


def build_menu_nodes(menu):
    """Build the nodes of a given menu hierarchy."""

    def map_node_name(node_name: str) -> str:
        return node_name.lower().replace(" ", "_")

    def threewise(menu):
        up, current, down = itertools.tee(menu, 3)
        up = itertools.chain((None,), up)
        down = itertools.chain(down, (None,))
        next(down, None)
        return zip(up, current, down)

    def helper(menu, parent=None):
        nodes = [MenuNode(name=node["name"], lines=node["lines"]) for node in menu]

        for (up, current, down), node in zip(threewise(nodes), menu):
            name = current.name if parent is None else parent.name + " " + current.name
            current.name = map_node_name(name)

            children = helper(node["children"], parent=current)
            current.child = next(children, None)

            if current.child is not None:
                children = itertools.chain((current.child,), children)

            current.name += "_node"
            current.up = up
            current.down = down
            current.parent = parent
            current.callback = node["callback"]

            yield current
            yield from children

    return list(helper(menu, parent=None))


def write_menu_nodes(nodes, outfile):
    """Write menu nodes to a given output file."""
    outfile.writelines(
        (
            "/**********************************************************\n",
            " * Programatically generated file -- do not edit by hand! *\n",
            " **********************************************************/\n\n",
        )
    )

    def try_get_reference(node):
        return "&" + node.name if node else "nullptr"

    def try_get_callback(callback):
        return callback if callback is not None else "nullptr"

    for node in nodes:
        outfile.writelines(
            (
                f"Menu {node.name} = {{\n",
                f'.line1 = "{node.lines[0]}",\n',
                f'.line2 = "{node.lines[1]}",\n',
                ".parent = nullptr,\n",
                ".child = nullptr,\n",
                ".up = nullptr,\n",
                ".down = nullptr,\n",
                f".callback = {try_get_callback(node.callback)},\n",
                "};\n",
            )
        )
        outfile.write("\n")

    for node in nodes:
        outfile.writelines(
            (
                f"{node.name}.parent = {try_get_reference(node.parent)};\n",
                f"{node.name}.child = {try_get_reference(node.child)};\n",
                f"{node.name}.up = {try_get_reference(node.up)};\n",
                f"{node.name}.down = {try_get_reference(node.down)};\n",
            )
        )
        outfile.write("\n")

    main_node = nodes[0]
    outfile.write(f"const Menu* main_menu = &{main_node.name};\n\n")


TERMINAL_COLORS = {
    "HEADER": "\033[95m",
    "OKBLUE": "\033[94m",
    "OKCYAN": "\033[96m",
    "OKGREEN": "\033[92m",
    "WARNING": "\033[93m",
    "FAIL": "\033[91m",
    "ENDC": "\033[0m",
    "BOLD": "\033[1m",
    "UNDERLINE": "\033[4m",
}


@contextmanager
def task_message(msg, on_success=None, on_error=None):
    """Manage the messaging surrounding the execution of a task."""
    on_error = "Failed with error:" if on_error is None else str(on_error)
    on_success = "Success." if on_success is None else str(on_success)
    try:
        start = TERMINAL_COLORS["HEADER"] + ">"
        end = TERMINAL_COLORS["ENDC"]
        print(start, msg, "...", "", end, end="", flush=True)
        yield
    except Exception:
        start = TERMINAL_COLORS["FAIL"]
        end = TERMINAL_COLORS["ENDC"]
        print(start, on_error, end, sep="", flush=True)
        raise
    else:
        start = TERMINAL_COLORS["OKGREEN"]
        end = TERMINAL_COLORS["ENDC"]
        print(start, on_success, end, sep="", flush=True)


def main():
    """Generate source code from a menu hierarchy."""
    parser = argparse.ArgumentParser(description=main.__doc__)
    parser.add_argument("infile", type=FileType("r"), help="Input JSON file.")
    parser.add_argument("outfile", type=FileType("w"), help="Output generated file.")
    args = parser.parse_args()

    # make sure the schema exists and can be read
    schema_filename = "hmi_schema.json"
    schema_filepath = get_local_file_path(schema_filename)
    on_error = f"Failed to read expected JSON schema: [ {schema_filename} ]."
    with task_message("Reading JSON schema", on_error=on_error):
        json_schema = read_json_data(schema_filepath)

    # make sure the newly generated JSON file can be read
    on_error = f"Failed to load JSON input file: [ {args.infile.name} ]."
    with task_message("Reading JSON input file", on_error=on_error):
        json_instance = read_json_data(args.infile)

    with task_message("Validating JSON input fileagainst schema"):
        jsonschema.validate(json_instance, json_schema)

    on_success = f"Menu file written and saved: [ {args.outfile.name} ]."
    with task_message("Building the menu file", on_success=on_success):
        nodes = build_menu_nodes(json_instance)
        write_menu_nodes(nodes, args.outfile)


if __name__ == "__main__":
    main()
