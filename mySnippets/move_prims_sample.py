"""Sample: add primitive shapes to the open stage, then translate / rotate them step by step.

Runs inside an already running Isaac Sim (VS Code "Run" / Script Editor / Python server).
The current stage is kept; everything is created under ``/World/Sample`` (re-created on each run).
For the standalone version (``python.bat``), see ``move_prims_sample_standalone.py``.
"""

import time

import numpy as np
import omni.kit.app
import omni.usd
from isaacsim.core.experimental.objects import Cube, Cylinder, DistantLight, Sphere
from pxr import Usd

ROOT = "/World/Sample"


async def next_frame() -> None:
    """Let Isaac Sim render one frame."""
    await omni.kit.app.get_app().next_update_async()


async def wait(seconds: float) -> None:
    """Sleep while keeping the app rendering (``time.sleep()`` would freeze the UI)."""
    end = time.time() + seconds
    while time.time() < end:
        await next_frame()


def quat_from_axis_angle(axis, angle_deg: float) -> np.ndarray:
    """Return a quaternion (w, x, y, z) for a rotation of ``angle_deg`` around ``axis``."""
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    half = np.deg2rad(angle_deg) / 2.0
    return np.array([np.cos(half), *(axis * np.sin(half))])


async def animate(duration: float, fn) -> None:
    """Call ``fn(t)`` with ``t`` going from 0.0 to 1.0 over ``duration`` seconds, one call per rendered frame."""
    start = time.time()
    while True:
        t = min((time.time() - start) / duration, 1.0)
        fn(t)
        await next_frame()
        if t >= 1.0:
            return


async def main() -> None:
    # ------------------------------------------------------------------
    # 1. Stage setup (keep the current stage, reset only our own subtree)
    # ------------------------------------------------------------------
    context = omni.usd.get_context()
    if context.get_stage() is None:
        await context.new_stage_async()
    stage = context.get_stage()
    # remove the previous run's prims from every layer (Kit also writes overs to the session layer)
    for layer in stage.GetLayerStack(includeSessionLayers=True):
        if layer.GetPrimAtPath(ROOT):
            with Usd.EditContext(stage, layer):
                stage.RemovePrim(ROOT)
    await next_frame()

    DistantLight(f"{ROOT}/DistantLight").set_intensities(300)

    # ------------------------------------------------------------------
    # 2. Add primitive shapes (visual only, no physics)
    # ------------------------------------------------------------------
    cube = Cube(f"{ROOT}/Cube", sizes=0.4, positions=[0.0, 0.0, 0.2], colors="red")
    sphere = Sphere(f"{ROOT}/Sphere", radii=0.2, positions=[0.0, 1.0, 0.2], colors="dodgerblue")
    cylinder = Cylinder(f"{ROOT}/Cylinder", radii=0.15, heights=0.5, positions=[0.0, -1.0, 0.25], colors="limegreen")

    print("[sample] shapes created")
    await wait(2.0)

    # ------------------------------------------------------------------
    # 3. Translation: move the cube along +X and back
    # ------------------------------------------------------------------
    print("[sample] translate cube along +X")
    start = np.array([0.0, 0.0, 0.2])
    goal = np.array([1.5, 0.0, 0.2])
    await animate(2.0, lambda t: cube.set_world_poses(positions=start + (goal - start) * t))
    await wait(1.0)

    print("[sample] translate cube back")
    await animate(2.0, lambda t: cube.set_world_poses(positions=goal + (start - goal) * t))
    await wait(1.0)

    # ------------------------------------------------------------------
    # 4. Rotation: spin the cube 360 deg around Z, tilt the cylinder around X
    # ------------------------------------------------------------------
    print("[sample] rotate cube around Z")
    await animate(3.0, lambda t: cube.set_world_poses(orientations=quat_from_axis_angle([0, 0, 1], 360.0 * t)))
    await wait(1.0)

    print("[sample] tilt cylinder around X")
    await animate(1.5, lambda t: cylinder.set_world_poses(orientations=quat_from_axis_angle([1, 0, 0], 90.0 * t)))
    await wait(1.0)
    await animate(
        1.5, lambda t: cylinder.set_world_poses(orientations=quat_from_axis_angle([1, 0, 0], 90.0 * (1.0 - t)))
    )
    await wait(1.0)

    # ------------------------------------------------------------------
    # 5. Translation + rotation together: sphere orbits the cube while the cube spins
    # ------------------------------------------------------------------
    print("[sample] sphere orbit + cube spin")

    def orbit(t: float) -> None:
        theta = 2.0 * np.pi * t
        sphere.set_world_poses(positions=[np.sin(theta), np.cos(theta), 0.2 + 0.5 * np.sin(2.0 * theta) ** 2])
        cube.set_world_poses(orientations=quat_from_axis_angle([0, 0, 1], -720.0 * t))

    await animate(5.0, orbit)

    # ------------------------------------------------------------------
    # 6. Print final poses
    # ------------------------------------------------------------------
    for name, prim in [("cube", cube), ("sphere", sphere), ("cylinder", cylinder)]:
        positions, orientations = prim.get_world_poses()
        print(f"[sample] {name}: position={positions.numpy()[0]}, orientation(wxyz)={orientations.numpy()[0]}")
    print("[sample] done")


await main()
