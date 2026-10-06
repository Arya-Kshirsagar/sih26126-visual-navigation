# GPS-Denied Visual Navigation for UGVs

**Smart India Hackathon 2026 · Problem Statement 26126 (Bharat Electronics Limited) · Team Beatles**

A camera-only autonomous navigation system for an outdoor Unmanned Ground Vehicle: it sees, localizes and drives using a single stereo camera, with **no GPS and no LiDAR**. Demonstrated end to end in a Gazebo forest world with hills, trees and rocks.

| Problem statement challenge | Our solution |
|---|---|
| **Path detection** | Stereo depth (SGBM) + terrain-aware obstacle classifier (slopes drivable, rocks/roots/trunks avoided, canopy ignored); HRNet+OCR terrain segmentation (RELLIS-3D) running live on the GPU |
| **Visual localization** | RTAB-Map stereo visual odometry (official position) + RTAB-Map SLAM (2D and 3D maps) |
| **Collision avoidance** | Nav2: Theta\* planner, MPPI controller, collision monitor, live stereo costmaps; long-range navigation to distant targets |

## Measured results (Gazebo, vs ground truth)

| Metric | Result |
|---|---|
| Visual odometry distance error (2 runs, ~5 m, slopes) | **0.2 – 2%** |
| Wheel odometry, same run | 32% short |
| 1.69 m slide with no commands | camera **1.70 m** · wheels 0 m |
| Visual odometry rate | ~8 Hz · 0.04–0.06 s/frame |
| SLAM update time | 0.05–0.07 s |
| Autonomous cruise speed | ~0.5 m/s |
| Terrain AI (HRNet+OCR, fp16, 640×480, RTX 3050 Laptop) | 73 ms/frame (~14 fps) |

## Repository layout

```
launch/     start scripts, pipeline launch file, Nav2 settings generator, RViz layout, tools
overlay/    our changes to third-party packages (applied by setup.sh)
ml/         HRNet+OCR config, MiDaS script (weights downloaded separately)
docs/       motivation
setup.sh    downloads third-party code at the exact versions used and applies our changes
```

## Requirements

- Ubuntu 24.04, **ROS 2 Jazzy**, **Gazebo Harmonic**
- An NVIDIA GPU is recommended (rendering and the segmentation model)
- ROS packages:
  ```bash
  sudo apt install ros-jazzy-rtabmap-ros ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
                   ros-jazzy-stereo-image-proc ros-jazzy-image-view ros-jazzy-nav2-theta-star-planner
  ```
- Python packages for the terrain AI (keep your existing numpy/torch versions):
  ```bash
  python3 -m pip install --no-deps --break-system-packages yacs
  python3 -m pip install --break-system-packages "timm==0.6.13"    # MiDaS only
  ```
  PyTorch with CUDA is required for the segmentation node.

## Setup

```bash
git clone https://github.com/Arya-Kshirsagar/sih26126-visual-navigation.git
cd sih26126-visual-navigation
./setup.sh                      # creates ~/SIH with everything in place
cd ~/SIH/bot_basics2 && rosdep install --from-paths src -y --ignore-src && colcon build
python3 ~/SIH/launch/make_nav2_params.py
```

**Model weights** (too large for GitHub):
- HRNet+OCR trained on RELLIS-3D, `best.pth` → `~/SIH/ML_Models/HRNet+OCR/best.pth` — download: *[add link]*
- MiDaS v2.1 small is downloaded automatically by `torch.hub` on first use.

## Run

```bash
~/SIH/launch/start_sim.sh                 # Terminal 1: simulation (add 'headless' for speed)
~/SIH/launch/start_pipeline.sh            # Terminal 2: stereo, VO, SLAM, Nav2, RViz
~/SIH/launch/start_pipeline.sh seg:=true  #   ... with live terrain segmentation
```

Then send a goal with **2D Goal Pose** in RViz, or use the tools:

```bash
python3 ~/SIH/launch/go_far.py --x X --y Y       # drive to a far world target, logs each trial
python3 ~/SIH/launch/world_info.py               # list world extent / far objects to pick a target
python3 ~/SIH/launch/set_light.py evening --fade 20   # change lighting mid-run
python3 ~/SIH/launch/seg_test.py                 # standalone segmentation test on one image
```

## Credits

Built on open-source work, used under their respective licenses:
[ROS 2](https://docs.ros.org), [Nav2](https://nav2.org), [RTAB-Map](https://github.com/introlab/rtabmap_ros),
[Husarion ROSbot packages](https://github.com/husarion), [Forest3D](https://github.com/unitsSpaceLab/Forest3D),
[RELLIS-3D / HRNet-OCR](https://github.com/unmannedlab/RELLIS-3D), [MiDaS](https://github.com/isl-org/MiDaS).
Third-party code is not redistributed here; `setup.sh` fetches it from the original sources.

**Team Beatles** — SIH 2026
