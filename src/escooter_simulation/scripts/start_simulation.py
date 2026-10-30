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
  2. the world is unpaused,
  3. the controllers are activated, which the controller manager carries out
     from its update loop - now running.

The gap is one service round trip, tens of milliseconds, during which the
scooter free-falls from its 1.4 degree spawn lean and picks up well under
0.1 rad/s - far inside what the reaction plate can arrest.

Sending the activation first and unpausing second would close the gap to a
single control cycle, but it deadlocks: the controller manager blocks inside the
switch callback waiting for an update that cannot happen until the world runs,
and /unpause_physics is served from the same process, so neither call returns.
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

    # Release the world first. The switch below is carried out by the controller
    # manager's update loop, which does not run while the world is paused, so
    # issuing it first would block the controller manager inside its own service
    # callback - and /unpause_physics is served from the same process, so that
    # call would never be answered either.
    if args.backend == 'classic':
        if not node.unpause.wait_for_service(timeout_sec=args.timeout):
            node.get_logger().error('/unpause_physics never appeared')
            return 1
        unpaused = node.unpause.call_async(Empty.Request())
        rclpy.spin_until_future_complete(node, unpaused, timeout_sec=args.timeout)
    else:
        try:
            unpause_ignition(args.world)
        except RuntimeError as exc:
            node.get_logger().error(str(exc))
            return 1

    request = SwitchController.Request()
    request.activate_controllers = list(args.controllers)
    request.strictness = SwitchController.Request.STRICT
    future = node.switch.call_async(request)
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
