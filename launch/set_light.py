#!/usr/bin/env python3
"""
Change the forest lighting while the simulation runs (for "changing light" demos).

  python3 ~/SIH/launch/set_light.py evening            # switch now
  python3 ~/SIH/launch/set_light.py evening --fade 20  # change gradually over 20 s
  python3 ~/SIH/launch/set_light.py default            # back to the world's original sun

Presets: morning, noon, evening, overcast, dim, default
"""
import argparse
import json
import math
import os
import re
import subprocess
import sys
import time

WORLD = os.path.expanduser('~/SIH/forest_world/Forest3D/forest.world')
STATE = os.path.expanduser('~/SIH/.light_state.json')

# direction = where the light shines towards; colours 0..1
PRESETS = {
    'morning':  dict(direction=(-0.80, 0.20, -0.35), diffuse=(1.00, 0.85, 0.65), specular=0.25, intensity=0.80, shadows=True),
    'noon':     dict(direction=(-0.20, 0.10, -0.97), diffuse=(1.00, 0.98, 0.92), specular=0.30, intensity=1.00, shadows=True),
    'evening':  dict(direction=(0.80, -0.30, -0.30), diffuse=(0.95, 0.60, 0.35), specular=0.15, intensity=0.60, shadows=True),
    'overcast': dict(direction=(0.00, 0.00, -1.00),  diffuse=(0.55, 0.55, 0.60), specular=0.05, intensity=0.60, shadows=False),
    'dim':      dict(direction=(0.30, 0.20, -0.90),  diffuse=(0.35, 0.30, 0.35), specular=0.02, intensity=0.35, shadows=False),
}


def read_world(path):
    text = open(path).read()
    world = re.search(r'<world\s+name="([^"]+)"', text)
    if not world:
        sys.exit('No <world name="..."> in ' + path)
    light = re.search(r'<light[^>]*type="directional"[^>]*>(.*?)</light>', text, re.S) or \
        re.search(r'<light[^>]*>(.*?)</light>', text, re.S)
    if not light:
        sys.exit('No <light> in ' + path)
    head = light.group(0)[:light.group(0).index('>')]
    name = re.search(r'name="([^"]+)"', head).group(1)
    body = light.group(1)

    def vec(tag, n, default):
        m = re.search(rf'<{tag}>\s*([^<]+)</{tag}>', body)
        if not m:
            return default
        v = [float(x) for x in m.group(1).split()[:n]]
        return tuple(v) if len(v) == n else default

    diffuse = vec('diffuse', 3, (0.8, 0.8, 0.8))
    spec = vec('specular', 3, (0.2, 0.2, 0.2))
    shadows = re.search(r'<cast_shadows>\s*(\w+)', body)
    intensity = re.search(r'<intensity>\s*([-\d.eE+]+)', body)
    default = dict(direction=vec('direction', 3, (-0.5, 0.1, -0.9)), diffuse=diffuse,
                   specular=spec[0], intensity=float(intensity.group(1)) if intensity else 1.0,
                   shadows=(shadows.group(1).lower() in ('true', '1')) if shadows else True)
    return world.group(1), name, default


def blend(a, b, t):
    lerp = lambda x, y: x + (y - x) * t
    d = [lerp(x, y) for x, y in zip(a['direction'], b['direction'])]
    n = math.sqrt(sum(v * v for v in d)) or 1.0
    return dict(direction=tuple(v / n for v in d),
                diffuse=tuple(lerp(x, y) for x, y in zip(a['diffuse'], b['diffuse'])),
                specular=lerp(a['specular'], b['specular']),
                intensity=lerp(a['intensity'], b['intensity']),
                shadows=b['shadows'] if t >= 0.5 else a['shadows'])


def light_msg(name, L):
    r, g, b = L['diffuse']
    s = L['specular']
    x, y, z = L['direction']
    return (f'name: "{name}" type: DIRECTIONAL '
            f'diffuse {{r: {r:.3f} g: {g:.3f} b: {b:.3f} a: 1}} '
            f'specular {{r: {s:.3f} g: {s:.3f} b: {s:.3f} a: 1}} '
            f'direction {{x: {x:.3f} y: {y:.3f} z: {z:.3f}}} '
            f'cast_shadows: {"true" if L["shadows"] else "false"} '
            f'intensity: {L["intensity"]:.3f}')


def apply(world, name, L, dry=False):
    cmd = ['gz', 'service', '-s', f'/world/{world}/light_config', '--reqtype', 'gz.msgs.Light',
           '--reptype', 'gz.msgs.Boolean', '--timeout', '2000', '--req', light_msg(name, L)]
    if dry:
        print(' '.join(cmd[:-1]) + " '" + cmd[-1] + "'")
        return True
    out = subprocess.run(cmd, capture_output=True, text=True)
    return 'true' in (out.stdout + out.stderr).lower()


def main():
    ap = argparse.ArgumentParser(description='Change the forest lighting at runtime.')
    ap.add_argument('preset', choices=list(PRESETS) + ['default'])
    ap.add_argument('--fade', type=float, default=0.0, help='seconds for a gradual change (0 = instant)')
    ap.add_argument('--world', default=WORLD)
    ap.add_argument('--dry-run', action='store_true', help='print the command instead of running it')
    args = ap.parse_args()

    world, name, default = read_world(args.world)
    target = default if args.preset == 'default' else PRESETS[args.preset]
    try:
        current = json.load(open(STATE))
    except Exception:
        current = default

    steps = max(1, int(args.fade / 0.5)) if args.fade > 0 else 1
    print(f'World "{world}", light "{name}": -> {args.preset}' + (f' over {args.fade:.0f} s' if steps > 1 else ''))
    for i in range(1, steps + 1):
        L = blend(current, target, i / steps)
        if not apply(world, name, L, args.dry_run):
            sys.exit('Gazebo did not accept the change - is the simulation running?')
        if i < steps:
            time.sleep(0.5)
    if not args.dry_run:
        json.dump({k: list(v) if isinstance(v, tuple) else v for k, v in target.items()}, open(STATE, 'w'))
    print('Done.')


if __name__ == '__main__':
    main()