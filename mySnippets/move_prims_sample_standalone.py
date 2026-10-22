"""Sample: create a stage, add primitive shapes, then translate / rotate them step by step.

Run:
    python.bat extUser\\move_prims_sample_standalone.py
"""

from isaacsim import SimulationApp

# SimulationApp must be created before importing any other Isaac Sim / Omniverse module
simulation_app = SimulationApp({"headless": False})

import time

import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
from isaacsim.core.experimental.objects import Cube, Cylinder, DistantLight, GroundPlane, Sphere


def wait(seconds: float) -> None:
    """Sleep while keeping the app rendering.

    A plain ``time.sleep()`` freezes the window, so we keep calling ``update()`` until the time has passed.
    """
    end = time.time() + seconds
    while time.time() < end and simulation_app.is_running():
        simulation_app.update()


def quat_from_axis_angle(axis, angle_deg: float) -> np.ndarray:
    """Return a quaternion (w, x, y, z) for a rotation of ``angle_deg`` around ``axis``."""
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    half = np.deg2rad(angle_deg) / 2.0
    return np.array([np.cos(half), *(axis * np.sin(half))])


def animate(duration: float, fn, fps: float = 60.0) -> None:
    """Call ``fn(t)`` with ``t`` going from 0.0 to 1.0 over ``duration`` seconds, rendering each frame."""
    frames = max(1, int(duration * fps))
    for i in range(frames + 1):
        if not simulation_app.is_running():
            return
        fn(i / frames)
        simulation_app.update()
        time.sleep(1.0 / fps)


# ------------------------------------------------------------------
# 1. Stage setup
# ------------------------------------------------------------------
stage_utils.create_new_stage()
GroundPlane("/World/GroundPlane", positions=[0, 0, 0])
DistantLight("/World/DistantLight").set_intensities(300)

# ------------------------------------------------------------------
# 2. Add primitive shapes (visual only, no physics)
# ------------------------------------------------------------------
cube = Cube("/World/Cube", sizes=0.4, positions=[0.0, 0.0, 0.2], colors="red")
sphere = Sphere("/World/Sphere", radii=0.2, positions=[0.0, 1.0, 0.2], colors="dodgerblue")
cylinder = Cylinder("/World/Cylinder", radii=0.15, heights=0.5, positions=[0.0, -1.0, 0.25], colors="limegreen")

print("[sample] shapes created")
wait(2.0)

# ------------------------------------------------------------------
# 3. Translation: move the cube along +X and back
# ------------------------------------------------------------------
print("[sample] translate cube along +X")
start = np.array([0.0, 0.0, 0.2])
goal = np.array([1.5, 0.0, 0.2])
animate(2.0, lambda t: cube.set_world_poses(positions=start + (goal - start) * t))
wait(1.0)

print("[sample] translate cube back")
animate(2.0, lambda t: cube.set_world_poses(positions=goal + (start - goal) * t))
wait(1.0)

# ------------------------------------------------------------------
# 4. Rotation: spin the cube 360 deg around Z
# ------------------------------------------------------------------
print("[sample] rotate cube around Z")
animate(3.0, lambda t: cube.set_world_poses(orientations=quat_from_axis_angle([0, 0, 1], 360.0 * t)))
wait(1.0)

# Tilt the cylinder 90 deg around X (lay it down), then stand it back up
print("[sample] tilt cylinder around X")
animate(1.5, lambda t: cylinder.set_world_poses(orientations=quat_from_axis_angle([1, 0, 0], 90.0 * t)))
wait(1.0)
animate(1.5, lambda t: cylinder.set_world_poses(orientations=quat_from_axis_angle([1, 0, 0], 90.0 * (1.0 - t))))
wait(1.0)

# ------------------------------------------------------------------
# 5. Translation + rotation together: sphere orbits the cube while the cube spins
# ------------------------------------------------------------------
print("[sample] sphere orbit + cube spin")


def orbit(t: float) -> None:
    theta = 2.0 * np.pi * t
    sphere.set_world_poses(positions=[np.sin(theta), np.cos(theta), 0.2 + 0.5 * np.sin(2.0 * theta) ** 2])
    cube.set_world_poses(orientations=quat_from_axis_angle([0, 0, 1], -720.0 * t))


animate(5.0, orbit)
wait(1.0)

# ------------------------------------------------------------------
# 6. Print final poses and keep the window open until it is closed
# ------------------------------------------------------------------
for name, prim in [("cube", cube), ("sphere", sphere), ("cylinder", cylinder)]:
    positions, orientations = prim.get_world_poses()
    print(f"[sample] {name}: position={positions.numpy()[0]}, orientation(wxyz)={orientations.numpy()[0]}")

print("[sample] done. Close the window to exit.")
while simulation_app.is_running():
    simulation_app.update()

simulation_app.close()
