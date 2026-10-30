#!/usr/bin/env python3
"""Export a flat URDF for the Isaac Sim URDF importer.

The USD assets under new_meshes/ were built from the pre-port URDF and have
drifted: they still carry the 0.00238 kg.m^2 reaction plate, the 40 N.m steering
limit and the 200 rad/s plate speed limit, so a scooter imported from them
cannot right itself and its steering is back-driven off its stop. Regenerating
from this script keeps the Isaac asset tied to urdf/e-scooter.urdf.xacro, which
is the single source of truth for every other target.

What it does, beyond running xacro with sim:=none:

  * drops the `dummy` root link and `dummy_joint`. `dummy` carries no inertial,
    and Isaac's importer would make it the articulation root - a zero-mass root
    body. The link only exists to keep KDL quiet in robot_state_publisher, and
    the previous Isaac export dropped it too.
  * leaves `imu_link` in place. The importer merges it into base_link like any
    other fixed-joint child, but keeping it documents where the IMU belongs:
    (-0.0254, 0.02032, 0.071234) from base_link.

Usage:
    ros2 run escooter_simulation export_isaac_urdf.py [-o OUTPUT] [xacro args...]
"""

import argparse
import os
import subprocess
import sys
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory

PACKAGE = 'escooter_simulation'
DROP_LINKS = {'dummy'}
DROP_JOINTS = {'dummy_joint'}

STATE = """\
State of the assets in new_meshes/ as of this writing:

  e-scooter/                      corrected in place, matches this URDF:
                                  plate inertia 0.06/0.06/0.12, steering limit
                                  200 N.m, plate speed limit 400 rad/s, no world
                                  fixation, articulation root on base_link,
                                  drives set per joint, imu_link frame present.
                                  Only the IMU sensor itself still has to be
                                  attached - see point 3.
  e-scooter_1st/                  the original import, still on the old numbers.
  _backup_usd/                    both assets exactly as they were before the
                                  correction, in case anything needs comparing.

Re-importing from scratch is still the cleaner option if the model changes
much; this is the list of what the importer gets wrong for this vehicle.
"""

CHECKLIST = """\
After importing this file with Isaac Sim's URDF importer, four things still need
fixing by hand. The importer gets them wrong for this vehicle every time.

1. ROOT JOINT
   Do not fix the base to the world. The previous export
   (new_meshes/e-scooter/) carries a `root_joint` of type PhysicsFixedJoint with
   an empty body0, and puts the articulation root on that joint - the scooter is
   bolted down and can neither fall nor drive. Clear "Fix Base Link" on import,
   and check the articulation root ends up on /e_scooter/base_link.
   new_meshes/e-scooter_1st/ is correct in this respect and is worth comparing
   against.

2. JOINT DRIVES
   The importer gives every joint a position servo. Only one of them wants it:

     back_wheel_joint         passive  -> stiffness 0, damping ~0
                                         (it had stiffness 357: the rear wheel
                                          is held at angle 0, like a dragging
                                          brake)
     front_wheel_joint        velocity -> stiffness 0, damping > 0
     rotational_plate_joint   torque   -> stiffness 0, damping 0
                                         (it had stiffness 39.8, which pulls the
                                          flywheel back to angle 0 and fights
                                          the balancing controller)
     handle_bar_chasis_joint  position -> stiffness > 0, and maxForce 200 N.m

3. IMU
   The previous export has an empty sensor layer, so there is no IMU at all and
   the balancing loop has no roll feedback. Add an IMU sensor on base_link at
   (-0.0254, 0.02032, 0.071234), publishing on /imu, and give it the same
   100 Hz update rate the Gazebo builds use.

4. CHECK THE NUMBERS LANDED
   rotational_plate   diagonal inertia   0.06, 0.06, 0.12   (not 0.00141/0.00238)
   handle_bar_chasis_joint  effort limit 200 N.m            (not 40)
   rotational_plate_joint   speed limit  400 rad/s          (not 200)

   Those three are exactly what had to change for the scooter to right itself
   after a fall; an asset still carrying the old figures will not.

See docs/PORTING.md for how each of those numbers was arrived at.
"""


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument(
        '-o', '--output',
        help='Where to write the URDF. Defaults to new_meshes/e-scooter_ros2humble.urdf '
             'inside the package source tree if it can be found, else the cwd.')
    parser.add_argument(
        '--keep-dummy', action='store_true',
        help='Keep the dummy root link instead of dropping it.')
    parser.add_argument(
        'xacro_args', nargs='*',
        help='Extra name:=value arguments passed through to xacro.')
    args = parser.parse_args()

    share = get_package_share_directory(PACKAGE)
    xacro_file = os.path.join(share, 'urdf', 'e-scooter.urdf.xacro')

    cmd = ['xacro', xacro_file, 'sim:=none'] + list(args.xacro_args)
    print('$ ' + ' '.join(cmd), file=sys.stderr)
    urdf = subprocess.run(cmd, capture_output=True, text=True, check=True).stdout

    root = ET.fromstring(urdf)
    if not args.keep_dummy:
        for link in list(root.findall('link')):
            if link.get('name') in DROP_LINKS:
                root.remove(link)
        for joint in list(root.findall('joint')):
            if joint.get('name') in DROP_JOINTS:
                root.remove(joint)

    out = args.output
    if out is None:
        # Follow the xacro file itself: with --symlink-install the installed
        # share/ directory is real but its contents are symlinks into the source
        # tree, so realpath() on the file lands where new_meshes/ actually is.
        source_urdf_dir = os.path.dirname(os.path.realpath(xacro_file))
        candidate = os.path.join(os.path.dirname(source_urdf_dir), 'new_meshes')
        if not os.path.isdir(candidate):
            candidate = os.getcwd()
        out = os.path.join(candidate, 'e-scooter_ros2humble.urdf')

    ET.indent(root, space='  ')
    body = ET.tostring(root, encoding='unicode')
    header = ('<?xml version="1.0"?>\n'
              '<!--\n'
              '  Generated by escooter_simulation/scripts/export_isaac_urdf.py from\n'
              '  urdf/e-scooter.urdf.xacro. Do not edit: regenerate instead, so the\n'
              '  Isaac asset cannot drift from the description the other targets use.\n'
              '\n'
              + CHECKLIST.replace('--', '- -') +
              '-->\n')
    with open(out, 'w') as handle:
        handle.write(header + body + '\n')

    notes = os.path.join(os.path.dirname(out), 'ISAAC_IMPORT.md')
    with open(notes, 'w') as handle:
        handle.write('# Importing the e-scooter into Isaac Sim\n\n'
                     'Source: `e-scooter_ros2humble.urdf`, generated by\n'
                     '`escooter_simulation/scripts/export_isaac_urdf.py`.\n\n'
                     + STATE + '\n' + CHECKLIST)

    print('wrote %s' % out)
    print('wrote %s' % notes)
    return 0


if __name__ == '__main__':
    sys.exit(main())
