# Franka Panda Peg-and-Ring Pick-and-Place

Python controller for a simulated Franka Panda manipulator that picks three colour-coded rings and places them on their matching pegs. The controller uses Cartesian waypoints, CoppeliaSim inverse kinematics, grasp-offset compensation, and greedy sequencing of the remaining rings.

For a fuller description of the project and its design choices, see the [project overview](https://ilalunghini.github.io/projects/franka-panda-pick-and-place.html).

## What is included

- `src/panda_movement_controller.py` — the controller;
- `requirements.txt` — the Python dependency.

The original CoppeliaSim scene, demonstration media, and other project assets are intentionally not included. To run the controller, provide a compatible CoppeliaSim scene with the required Panda, ring, peg, target, and gripper object hierarchy, and enable the ZeroMQ Remote API. The expected object paths are collected in `SceneObjectPaths` at the top of the controller for straightforward adaptation.

## Setup

```zsh
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
python src/panda_movement_controller.py
```

This is an academic simulation project; it does not include a physical-robot validation or a shared scene file.
