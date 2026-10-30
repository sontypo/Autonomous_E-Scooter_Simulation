#!/usr/bin/env python3
"""Activate the controllers and release the paused simulation in one step.

A self-balancing vehicle cannot afford the usual bring-up order. Spawning the
model into a running world and then activating controllers one by one leaves
several seconds in which nothing holds the scooter up, and the reaction plate
only has enough angular momentum to recover about 1.9 degrees of lean from
rest - less than the scooter accumulates in the first half second of free fall.

So the world starts paused and this script closes the gap:

  1. the controllers are already loaded and configured (the launch file spawns
     them with `--inactive`, which works while the world is paused),
  2. the activation request is sent first and blocks server side, because the
     controller manager performs switches from its update loop,
  3. the world is unpaused, and the very first simulation step carries out the
     pending switch.

The controllers are therefore live within one control cycle of physics starting.
"""

import argparse
import subprocess
import sys

import rclpy
from controller_manager_msgs.srv import SwitchController
from rclpy.node import Node
from std_srvs.srv import Empty


class SimStarter(Node):

    def __init__(self, controller_manager):
        super().__init__('start_simulation')
        self.switch = self.create_client(
            SwitchController, '%s/switch_controller' % controller_manager)
        self.unpause = self.create_client(Empty, '/unpause_physics')


def unpause_ignition(world, timeout_ms=10000):
    cmd = [
        'ign', 'service', '-s', '/world/%s/control' % world,
        '--reqtype', 'ignition.msgs.WorldControl',
        '--reptype', 'ignition.msgs.Boolean',
        '--timeout', str(timeout_ms),
        '--req', 'pause: false',
    ]
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        raise RuntimeError('ign service failed: %s%s' % (result.stdout, result.stderr))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--backend', choices=['classic', 'ignition'], default='classic')
    parser.add_argument('--controller-manager', default='/e_scooter/controller_manager')
    parser.add_argument('--world', default='default',
                        help='World name, used by the Ignition unpause service.')
    parser.add_argument('--timeout', type=float, default=60.0)
    parser.add_argument('controllers', nargs='+')
    args = parser.parse_args()

    rclpy.init()
    node = SimStarter(args.controller_manager)

    if not node.switch.wait_for_service(timeout_sec=args.timeout):
        node.get_logger().error(
            'controller manager %s never offered switch_controller' % args.controller_manager)
        return 1

    request = SwitchController.Request()
    request.activate_controllers = list(args.controllers)
    request.strictness = SwitchController.Request.STRICT
    # The switch is carried out by the controller manager's update loop, so this
    # call stays pending until the world starts stepping. That is exactly what
    # we want: send it first, then let physics go.
    future = node.switch.call_async(request)

    if args.backend == 'classic':
        if not node.unpause.wait_for_service(timeout_sec=args.timeout):
            node.get_logger().error('/unpause_physics never appeared')
            return 1
        node.unpause.call_async(Empty.Request())
    else:
        try:
            unpause_ignition(args.world)
        except RuntimeError as exc:
            node.get_logger().error(str(exc))
            return 1

    rclpy.spin_until_future_complete(node, future, timeout_sec=args.timeout)
    if not future.done():
        node.get_logger().error('controller activation timed out')
        return 1
    if not future.result().ok:
        node.get_logger().error('controller activation was rejected')
        return 1

    node.get_logger().info(
        'simulation running, controllers active: %s' % ', '.join(args.controllers))
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
