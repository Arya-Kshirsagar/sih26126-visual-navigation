#!/usr/bin/env python3
"""Show the forest world's extent and the objects farthest from the robot's start,
to help choose a fixed target for go_far.py.

Usage:  python3 ~/SIH/launch/world_info.py
"""
import math
import os
import re
import sys

WORLD = os.path.expanduser(sys.argv[1] if len(sys.argv) > 1 else '~/SIH/forest_world/Forest3D/forest.world')
SPAWN = (9.0, -45.0)

text = open(WORLD).read()
items = []
for block in re.findall(r'<(include|model)\b.*?</\1>', text, re.S):
    pass
for m in re.finditer(r'<(include|model)\b([^>]*)>(.*?)</\1>', text, re.S):
    kind, attrs, body = m.groups()
    name = re.search(r'<name>(.*?)</name>', body)
    name = name.group(1) if name else (re.search(r'name="([^"]*)"', attrs) or [None, kind])[1]
    uri = re.search(r'<uri>(.*?)</uri>', body)
    pose = re.search(r'<pose[^>]*>\s*([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)', body)
    if pose:
        x, y, z = map(float, pose.groups())
        items.append((name, uri.group(1) if uri else '', x, y, z))

if not items:
    sys.exit('No objects with a <pose> found in ' + WORLD)
xs, ys = [i[2] for i in items], [i[3] for i in items]
print(f'World: {WORLD}')
print(f'{len(items)} objects | x from {min(xs):.0f} to {max(xs):.0f} | y from {min(ys):.0f} to {max(ys):.0f}')
print(f'Robot start (world): {SPAWN}\n')
far = sorted(items, key=lambda i: -math.hypot(i[2] - SPAWN[0], i[3] - SPAWN[1]))[:10]
print('Farthest objects from the start:')
for name, uri, x, y, z in far:
    d = math.hypot(x - SPAWN[0], y - SPAWN[1])
    print(f'  {name[:28]:28s} x={x:8.1f}  y={y:8.1f}  z={z:6.1f}   {d:6.1f} m away   {os.path.basename(uri)}')
print('\nPick a target in OPEN ground near the far side (a few metres away from any object),')
print('then put it in go_far.py (DEFAULT_TARGET) or pass it:  python3 ~/SIH/launch/go_far.py --x X --y Y')