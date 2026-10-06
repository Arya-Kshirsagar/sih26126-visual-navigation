#!/usr/bin/env python3
"""
Generate ~/SIH/launch/nav2_params.yaml from the Nav2 default that ships with your install,
changing only what our stereo-camera ROSbot needs.

Usage:  python3 ~/SIH/launch/make_nav2_params.py
"""
import os
import sys
import yaml

SRC = sys.argv[1] if len(sys.argv) > 1 else '/opt/ros/jazzy/share/nav2_bringup/params/nav2_params.yaml'
OUT = os.path.expanduser(sys.argv[2] if len(sys.argv) > 2 else '~/SIH/launch/nav2_params.yaml')

ODOM_TOPIC = '/vo'                    # visual odometry is the official odometry
BASE = 'base_link'
CLOUD = '/obstacles_cloud'            # stereo points already classified as obstacles (relative to robot)
GROUND = '/ground_cloud'              # stereo points classified as ground - used to clear the map
ROBOT_RADIUS = 0.17                   # ROSbot is ~0.20 x 0.23 m
INFLATION = 0.35                      # safety margin around obstacles (m)
COST_SCALING = 8.0                    # higher = danger fades faster, open ground looks cheap
MAX_V, MIN_V, MAX_W = 0.6, -0.2, 1.5     # m/s forward, m/s reverse, rad/s turning

p = yaml.safe_load(open(SRC))


def rp(node, sub=None):
    """ros__parameters dict of a node (costmaps are nested one level deeper)."""
    d = p[node] if sub is None else p[node][sub]
    return d['ros__parameters']


def obstacle_source():
    # Marks obstacles - only within 3.5 m, where stereo depth is accurate.
    return {
        'topic': CLOUD,
        'data_type': 'PointCloud2',
        'marking': True,
        'clearing': True,
        # Points are already classified as obstacles relative to the robot, so accept any height.
        # (On hills the robot's z in odom/map can be metres away from zero.)
        'min_obstacle_height': -100.0,
        'max_obstacle_height': 100.0,
        'obstacle_min_range': 0.0,
        'obstacle_max_range': 3.5,
        'raytrace_min_range': 0.0,
        'raytrace_max_range': 5.5,
    }


def ground_source():
    # Clears only: every ray to a ground point erases stale obstacles (ghosts) along it.
    return {
        'topic': GROUND,
        'data_type': 'PointCloud2',
        'marking': False,
        'clearing': True,
        'min_obstacle_height': -100.0,
        'max_obstacle_height': 100.0,
        'raytrace_min_range': 0.0,
        'raytrace_max_range': 5.0,
    }


def obstacle_layer():
    return {
        'plugin': 'nav2_costmap_2d::ObstacleLayer',
        'enabled': True,
        'min_obstacle_height': -100.0,
        'max_obstacle_height': 100.0,
        'observation_sources': 'stereo ground',
        'stereo': obstacle_source(),
        'ground': ground_source(),
    }


# --- BT navigator
bt = rp('bt_navigator')
bt['robot_base_frame'] = BASE
bt['odom_topic'] = ODOM_TOPIC

# --- Controller (MPPI): slower, suited to a small robot outdoors
cs = rp('controller_server')
cs['enable_stamped_cmd_vel'] = True
cs['odom_topic'] = ODOM_TOPIC          # controller must know the robot's real speed (default /odom doesn't exist)
fp = cs['FollowPath']
fp['vx_max'], fp['vx_min'], fp['wz_max'] = MAX_V, MIN_V, MAX_W
fp['vx_std'] = 0.3                    # try a wider range of speeds
fp['batch_size'] = 1200               # fewer sampled paths per cycle -> less CPU
fp['time_steps'] = 48                 # 48 x 0.05 s = 2.4 s look-ahead
fp['PathAlignCritic']['cost_weight'] = 8.0   # follow the path's general line, not every kink (was 14)

# --- Local costmap: rolling 6 m window in odom, stereo obstacles instead of laser/voxels
lc = rp('local_costmap', 'local_costmap')
lc['robot_base_frame'] = BASE
lc['width'] = lc['height'] = 6
lc['robot_radius'] = ROBOT_RADIUS
lc['plugins'] = ['obstacle_layer', 'inflation_layer']
lc.pop('voxel_layer', None)
lc.pop('static_layer', None)
lc['obstacle_layer'] = obstacle_layer()
lc['inflation_layer']['inflation_radius'] = INFLATION
lc['inflation_layer']['cost_scaling_factor'] = COST_SCALING

# --- Global costmap: RTAB-Map's /map + live stereo obstacles
gc = rp('global_costmap', 'global_costmap')
gc['robot_base_frame'] = BASE
gc['robot_radius'] = ROBOT_RADIUS
gc['resolution'] = 0.1                # match RTAB-Map's 10 cm grid
# A 60 m window that moves with the robot, so far-away goals are never 'outside the map'.
# Obstacles come from the stereo camera; unexplored cells are allowed for planning.
gc['rolling_window'] = True
gc['width'] = gc['height'] = 60
gc['track_unknown_space'] = True
gc['plugins'] = ['obstacle_layer', 'inflation_layer']
gc.pop('static_layer', None)
gc['obstacle_layer'] = obstacle_layer()
gc['inflation_layer']['inflation_radius'] = INFLATION
gc['inflation_layer']['cost_scaling_factor'] = COST_SCALING

# --- Planner: Theta* draws straight segments between free points instead of grid staircases
rp('planner_server')['GridBased'] = {
    'plugin': 'nav2_theta_star_planner::ThetaStarPlanner',
    'how_many_corners': 8,
    'w_euc_cost': 1.0,
    'w_traversal_cost': 2.0,                # keeps a sensible distance from obstacles
    'allow_unknown': True,                   # may plan through unexplored ground
    'use_final_approach_orientation': False,
}

# --- Behaviors (spin, backup...) and docking also publish velocity commands
rp('behavior_server')['robot_base_frame'] = BASE
rp('behavior_server')['enable_stamped_cmd_vel'] = True
if 'docking_server' in p:
    rp('docking_server')['base_frame'] = BASE
    rp('docking_server')['enable_stamped_cmd_vel'] = True

# --- Velocity smoother
vs = rp('velocity_smoother')
vs['enable_stamped_cmd_vel'] = True
vs['max_velocity'] = [MAX_V, 0.0, MAX_W]
vs['min_velocity'] = [MIN_V, 0.0, -MAX_W]
vs['odom_topic'] = ODOM_TOPIC

# --- Collision monitor: last safety check, then out to twist_mux's autonomous input
cm = rp('collision_monitor')
cm['enable_stamped_cmd_vel'] = True
cm['base_frame_id'] = BASE
cm['cmd_vel_out_topic'] = '/autonomous/cmd_vel'
cm['FootprintApproach']['time_before_collision'] = 0.6   # brake later (was 1.2 s)
cm['FootprintApproach']['min_points'] = 10             # ignore small noise blobs (was 6)
cm['observation_sources'] = ['stereo']
cm.pop('scan', None)
cm['stereo'] = {
    'type': 'pointcloud',
    'topic': CLOUD,
    'min_height': -1.0,               # already classified; heights here are relative to base_link
    'max_height': 1.0,
    'enabled': True,
}

os.makedirs(os.path.dirname(OUT), exist_ok=True)
with open(OUT, 'w') as f:
    f.write('# Generated by make_nav2_params.py from ' + SRC + '\n')
    yaml.safe_dump(p, f, sort_keys=False, default_flow_style=None)
print('Wrote ' + OUT)