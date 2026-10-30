#!/usr/bin/env python3
"""Process a xacro file into a single-line, comment-free URDF.

Why this exists instead of plain `xacro`:

`gazebo_ros2_control` 0.4.x and `gz_ros2_control` 0.7.x hand the robot
description to the controller manager as a ROS 2 command-line parameter
override, `--param robot_description:=<the whole URDF>`. rcl parses the part
after `:=` as a standalone YAML document, so the value has to be a valid YAML
plain scalar. A pretty-printed URDF is not one:

  * every line after the first starts at column 0, which ends the scalar, and
  * xacro's comments contain `: ` and ` #`, both YAML indicators.

Either of those makes the plugin fail with

    parser error Couldn't parse parameter override rule: '--param robot_description:=...

and the controller manager never starts. Emitting the URDF on one line with the
comments removed sidesteps both. The XML is unchanged in every other respect.

Usage mirrors xacro's own CLI:

    xacro_to_urdf.py <file.xacro> [name:=value ...]
"""

import sys
from xml.dom import Node

import xacro


def strip_comments(node):
    for child in list(node.childNodes):
        if child.nodeType == Node.COMMENT_NODE:
            node.removeChild(child)
            child.unlink()
        elif child.hasChildNodes():
            strip_comments(child)


def main(argv):
    if len(argv) < 2:
        print(__doc__, file=sys.stderr)
        return 2

    input_file = argv[1]
    mappings = {}
    for token in argv[2:]:
        if ':=' not in token:
            print("ignoring argument without ':=': %s" % token, file=sys.stderr)
            continue
        key, value = token.split(':=', 1)
        mappings[key] = value

    doc = xacro.process_file(input_file, mappings=mappings)
    strip_comments(doc)

    # minidom's toxml() does not pretty-print, so the result is already a single
    # line; collapsing any stray newline keeps that guarantee.
    xml = doc.toxml().replace('\n', ' ').replace('\r', ' ')
    sys.stdout.write(xml)
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
