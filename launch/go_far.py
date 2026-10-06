#!/usr/bin/env python3
"""
Send the robot to a fixed far-away target and let it find its own way there.

The target is given in Gazebo WORLD coordinates, so every trial goes to the same place.
At start-up the script asks Gazebo where the robot really is and converts the target into
the map frame. It then keeps handing Nav2 an intermediate goal ~20 m ahead in the target's
direction ("carrot"), moving it forward as the robot gets close. If an intermediate goal
fails, it tries other directions and shorter steps.

Each run is appended to ~/SIH/trials.csv (time, distance, failures, result).

Usage (sim + pipeline running):
  python3 ~/SIH/launch/go_far.py --x 120 --y 80        # target in world coordinates
  python3 ~/SIH/launch/go_far.py                       # uses the default target below
"""
import argparse
import csv
import math
import os
import re
import subprocess
import time

DEFAULT_TARGET = (1.9,35.1)        # world x, y - change once you pick your target
STEP = 20.0                           # metres per intermediate goal
SWITCH_DIST = 5.0                     # move the carrot on when this close to it
FINAL_TOL = 1.5                       # success when this close to the target
MAX_FAILS = 12                        # give up after this many failed intermediate goals
TRIAL_LOG = os.path.expanduser('~/SIH/trials.csv')


# ----------------------------------------------------------------------------- pure helpers

def world_to_map(wx, wy, robot_world, robot_map):
    """Convert a world point to the map frame, using the robot's pose in both frames.
    robot_world / robot_map = (x, y, yaw)."""
    rwx, rwy, rwyaw = robot_world
    rmx, rmy, rmyaw = robot_map
    dx, dy = wx - rwx, wy - rwy
    # express the offset in the robot's frame, then in the map frame
    c, s = math.cos(-rwyaw), math.sin(-rwyaw)
    bx, by = c * dx - s * dy, s * dx + c * dy
    c, s = math.cos(rmyaw), math.sin(rmyaw)
    return rmx + c * bx - s * by, rmy + s * bx + c * by


def carrot(rx, ry, tx, ty, step, angle_offset=0.0):
    """Intermediate goal `step` m from the robot towards the target (optionally rotated)."""
    d = math.hypot(tx - rx, ty - ry)
    if d <= step and angle_offset == 0.0:
        return tx, ty, math.atan2(ty - ry, tx - rx)
    a = math.atan2(ty - ry, tx - rx) + angle_offset
    L = min(step, d)
    return rx + L * math.cos(a), ry + L * math.sin(a), a


def parse_gz_pose(text):
    nums = re.findall(r'\[([-\d.eE+ ]+)\]', text)
    if len(nums) < 2:
        raise RuntimeError('could not read the robot pose from Gazebo')
    xyz = [float(v) for v in nums[-2].split()]
    rpy = [float(v) for v in nums[-1].split()]
    return xyz[0], xyz[1], rpy[2]


def yaw_from_quat(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


# ----------------------------------------------------------------------------- ROS part

def main():
    ap = argparse.ArgumentParser(description='Drive to a fixed far-away world target.')
    ap.add_argument('--x', type=float, default=DEFAULT_TARGET[0], help='target x (Gazebo world)')
    ap.add_argument('--y', type=float, default=DEFAULT_TARGET[1], help='target y (Gazebo world)')
    ap.add_argument('--step', type=float, default=STEP)
    ap.add_argument('--model', default='rosbot', help='robot model name in Gazebo')
    args = ap.parse_args()

    import rclpy
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from geometry_msgs.msg import PoseStamped
    from nav2_msgs.action import NavigateToPose
    from action_msgs.msg import GoalStatus
    import tf2_ros

    rclpy.init()
    node = Node('go_far', parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
    log = node.get_logger()
    tf_buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(tf_buf, node)
    client = ActionClient(node, NavigateToPose, 'navigate_to_pose')

    def spin_for(sec):
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.05)

    def robot_map_pose():
        t = tf_buf.lookup_transform('map', 'base_link', rclpy.time.Time())
        tr = t.transform
        return tr.translation.x, tr.translation.y, yaw_from_quat(tr.rotation)

    def sim_now():
        return node.get_clock().now().nanoseconds * 1e-9

    log.info('Waiting for Nav2 and the map frame...')
    if not client.wait_for_server(timeout_sec=60.0):
        log.error('Nav2 action server not available - is the pipeline running with nav?')
        return
    for _ in range(200):
        spin_for(0.1)
        try:
            robot_map_pose()
            break
        except Exception:
            pass
    else:
        log.error('No map -> base_link transform - is SLAM running?')
        return

    # target: world -> map, using the robot's true pose from Gazebo
    out = subprocess.run(['gz', 'model', '-m', args.model, '-p'], capture_output=True, text=True, timeout=20)
    robot_world = parse_gz_pose(out.stdout + out.stderr)
    robot_map = robot_map_pose()
    tx, ty = world_to_map(args.x, args.y, robot_world, robot_map)
    dist0 = math.hypot(tx - robot_map[0], ty - robot_map[1])
    log.info(f'Robot world ({robot_world[0]:.1f}, {robot_world[1]:.1f}) -> target world ({args.x:.1f}, {args.y:.1f})')
    log.info(f'Target in map frame ({tx:.1f}, {ty:.1f}), straight-line distance {dist0:.1f} m')

    state = {'handle': None, 'status': None}

    def send(gx, gy, gyaw):
        g = NavigateToPose.Goal()
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.pose.position.x, p.pose.position.y = gx, gy
        p.pose.orientation.z, p.pose.orientation.w = math.sin(gyaw / 2), math.cos(gyaw / 2)
        g.pose = p
        state['status'] = None
        fut = client.send_goal_async(g)
        while not fut.done():
            rclpy.spin_once(node, timeout_sec=0.05)
        h = fut.result()
        if not h.accepted:
            state['status'] = GoalStatus.STATUS_ABORTED
            return
        state['handle'] = h
        h.get_result_async().add_done_callback(lambda f: state.update(status=f.result().status))

    t_start_sim, t_start_wall = sim_now(), time.time()
    path_len, last = 0.0, robot_map[:2]
    fails, offsets = 0, [0.0, 0.5, -0.5, 1.0, -1.0]
    offset_i, step = 0, args.step
    result = 'failed'

    gx, gy, gyaw = carrot(*robot_map[:2], tx, ty, step)
    send(gx, gy, gyaw)
    log.info(f'Intermediate goal ({gx:.1f}, {gy:.1f})')
    try:
        while True:
            spin_for(0.5)
            rx, ry, _ = robot_map_pose()
            path_len += math.hypot(rx - last[0], ry - last[1])
            last = (rx, ry)
            to_target = math.hypot(tx - rx, ty - ry)
            final = math.hypot(gx - tx, gy - ty) < 0.01

            if to_target < FINAL_TOL:
                result = 'success'
                break
            if not final and math.hypot(gx - rx, gy - ry) < SWITCH_DIST:
                offset_i, step = 0, args.step                       # progress - reset fallbacks
                gx, gy, gyaw = carrot(rx, ry, tx, ty, step)
                send(gx, gy, gyaw)
                log.info(f'{to_target:.0f} m to go - next goal ({gx:.1f}, {gy:.1f})')
                continue
            if state['status'] == GoalStatus.STATUS_SUCCEEDED and final:
                result = 'success'
                break
            if state['status'] in (GoalStatus.STATUS_ABORTED, GoalStatus.STATUS_CANCELED) or \
               (state['status'] == GoalStatus.STATUS_SUCCEEDED and not final):
                if state['status'] != GoalStatus.STATUS_SUCCEEDED:
                    fails += 1
                    log.warn(f'Intermediate goal failed ({fails}/{MAX_FAILS}) - trying another way')
                    if fails >= MAX_FAILS:
                        break
                    offset_i += 1
                    if offset_i >= len(offsets):
                        offset_i, step = 0, max(5.0, step / 2)
                gx, gy, gyaw = carrot(rx, ry, tx, ty, step, offsets[offset_i])
                send(gx, gy, gyaw)
                log.info(f'{to_target:.0f} m to go - goal ({gx:.1f}, {gy:.1f})')
    except KeyboardInterrupt:
        result = 'stopped'
        if state['handle'] is not None:
            state['handle'].cancel_goal_async()
            spin_for(0.5)

    sim_t, wall_t = sim_now() - t_start_sim, time.time() - t_start_wall
    rx, ry, _ = robot_map_pose()
    left = math.hypot(tx - rx, ty - ry)
    log.info(f'RESULT: {result} | straight-line {dist0:.1f} m, driven {path_len:.1f} m, '
             f'{left:.1f} m left | sim time {sim_t:.0f} s (wall {wall_t:.0f} s) | failed goals {fails}')

    new = not os.path.exists(TRIAL_LOG)
    with open(TRIAL_LOG, 'a', newline='') as f:
        w = csv.writer(f)
        if new:
            w.writerow(['date', 'target_world_x', 'target_world_y', 'result', 'straight_line_m',
                        'driven_m', 'remaining_m', 'sim_time_s', 'wall_time_s', 'failed_goals'])
        w.writerow([time.strftime('%Y-%m-%d %H:%M'), args.x, args.y, result, f'{dist0:.1f}',
                    f'{path_len:.1f}', f'{left:.1f}', f'{sim_t:.0f}', f'{wall_t:.0f}', fails])
    log.info(f'Trial saved to {TRIAL_LOG}')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()