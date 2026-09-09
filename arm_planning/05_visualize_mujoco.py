"""
05_visualize_mujoco.py
========================
Simulate/visualize the planned trajectory in MuJoCo.

Why not just load g1_23dof.urdf directly into MuJoCo?
-------------------------------------------------------
MuJoCo's URDF importer insists on resolving every <mesh filename="..."/>
reference to an actual STL file on disk. The URDF you uploaded references
meshes/*.STL, but only the .urdf itself was uploaded -- not the meshes/
folder -- so MuJoCo's loader fails with a missing-file error (yourdfpy is
more lenient here, which is why stages 1-3 could use it for the
verification check without the mesh files).

Instead, this script builds a small MJCF (MuJoCo's native XML format)
directly from the SAME numbers in arm_config.py that already drive the
pipeline (joint origins, axes, limits) -- the ones already checked
against yourdfpy to 1e-16 precision. It draws a plain capsule "stick
figure" for each link, which is all you need to visually validate the
motion. If you obtain the real meshes/ folder (it normally ships
alongside a URDF from Unitree's robot-description repo), you can load
g1_23dof.urdf directly in MuJoCo instead and skip this script entirely --
the joint names and angles output by 03_plan_path.py will map onto it
directly.
"""

import argparse
import time

import numpy as np
import mujoco
import mujoco.viewer

import arm_config as cfg


def mat_to_quat(R: np.ndarray) -> np.ndarray:
    """3x3 rotation matrix -> quaternion (w, x, y, z), MuJoCo's convention."""
    tr = np.trace(R)
    if tr > 0:
        S = np.sqrt(tr + 1.0) * 2
        w = 0.25 * S
        x = (R[2, 1] - R[1, 2]) / S
        y = (R[0, 2] - R[2, 0]) / S
        z = (R[1, 0] - R[0, 1]) / S
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / S
        x = 0.25 * S
        y = (R[0, 1] + R[1, 0]) / S
        z = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / S
        x = (R[0, 1] + R[1, 0]) / S
        y = 0.25 * S
        z = (R[1, 2] + R[2, 1]) / S
    else:
        S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / S
        x = (R[0, 2] + R[2, 0]) / S
        y = (R[1, 2] + R[2, 1]) / S
        z = 0.25 * S
    return np.array([w, x, y, z])


def build_mjcf() -> str:
    """
    Build a minimal MJCF string: a fixed torso + 5 hinge joints in series,
    each drawn as a capsule reaching to the next joint's origin, using
    arm_config's yourdfpy-verified geometry directly.
    """
    joint_tags = []
    for i in range(cfg.NUM_JOINTS):
        quat = mat_to_quat(cfg._rpy_to_R(cfg.ORIGIN_RPY[i]))
        pos = cfg.ORIGIN_XYZ[i]
        axis = cfg.JOINT_AXES[i]
        lo, hi = cfg.JOINT_LIMITS[i]

        # Link geometry: capsule from this joint's origin to the next
        # joint's origin (expressed in this body's own local frame); the
        # last body just gets a short marker capsule + a site for the
        # end-effector, plus a small red sphere marking the "hand".
        if i < cfg.NUM_JOINTS - 1:
            to = cfg.ORIGIN_XYZ[i + 1]
            geom = f'<geom type="capsule" fromto="0 0 0 {to[0]} {to[1]} {to[2]}" size="0.012" rgba="0.7 0.7 0.8 1"/>'
        else:
            geom = ('<geom type="capsule" fromto="0 0 0 0.05 0 0" size="0.014" rgba="0.9 0.3 0.2 1"/>'
                    '<site name="end_effector" pos="0 0 0" size="0.008" rgba="1 0 0 1"/>')

        joint_tags.append(f'''
{'  ' * (i + 1)}<body name="link{i}" pos="{pos[0]} {pos[1]} {pos[2]}" quat="{quat[0]} {quat[1]} {quat[2]} {quat[3]}">
{'  ' * (i + 2)}<joint name="{cfg.JOINT_NAMES[i]}" type="hinge" axis="{axis[0]} {axis[1]} {axis[2]}" range="{lo} {hi}" damping="0.2"/>
{'  ' * (i + 2)}{geom}''')

    # Close tags in reverse, and nest via string building.
    body_open = "".join(joint_tags)
    body_close = "".join("</body>" for _ in range(cfg.NUM_JOINTS))

    mjcf = f"""
<mujoco model="g1_right_arm_5dof">
  <compiler angle="radian"/>
  <option gravity="0 0 0"/>
  <visual>
    <headlight ambient="0.4 0.4 0.4"/>
  </visual>
  <worldbody>
    <light diffuse="0.8 0.8 0.8" pos="0.5 0 1" dir="-0.5 0 -1"/>
    <geom type="plane" size="1 1 0.01" pos="0 0 -0.9" rgba="0.9 0.9 0.9 1"/>
    <body name="torso_link" pos="0 0 0">
      <geom type="box" size="0.08 0.1 0.15" rgba="0.5 0.5 0.6 0.6"/>
      {body_open}
      {body_close}
    </body>
  </worldbody>
</mujoco>
"""
    return mjcf


def load_trajectory(csv_path: str):
    data = np.genfromtxt(csv_path, delimiter=",", names=True)
    joints = np.stack([data[name] for name in cfg.JOINT_NAMES], axis=1)
    return joints


def main():
    parser = argparse.ArgumentParser(description="Animate the planned trajectory in MuJoCo.")
    parser.add_argument("--trajectory", type=str, default="joint_trajectory.csv")
    parser.add_argument("--mode", choices=["viewer", "video"], default="viewer",
                         help="'viewer' opens an interactive window (needs a local display). "
                              "'video' renders offscreen to an .mp4 (works headless/SSH).")
    parser.add_argument("--out", type=str, default="trajectory.mp4")
    parser.add_argument("--seconds-per-step", type=float, default=0.4)
    args = parser.parse_args()

    mjcf = build_mjcf()
    model = mujoco.MjModel.from_xml_string(mjcf)
    data = mujoco.MjData(model)

    joints = load_trajectory(args.trajectory)

    if args.mode == "viewer":
        with mujoco.viewer.launch_passive(model, data) as viewer:
            print("Looping the planned trajectory in the MuJoCo viewer window "
                  "(Ctrl+C in this terminal to stop)...")
            while viewer.is_running():
                for q in joints:
                    data.qpos[:] = q
                    mujoco.mj_forward(model, data)
                    viewer.sync()
                    time.sleep(args.seconds_per_step)
    else:
        renderer = mujoco.Renderer(model, height=480, width=640)
        frames = []
        for q in joints:
            data.qpos[:] = q
            mujoco.mj_forward(model, data)
            renderer.update_scene(data)
            frames.append(renderer.render().copy())

        try:
            import imageio
            fps = max(1.0 / args.seconds_per_step, 1.0)
            imageio.mimsave(args.out, frames, fps=fps)
            print(f"Saved video -> {args.out}  ({len(frames)} frames)")
        except ImportError:
            import numpy as _np
            npy_out = args.out.rsplit(".", 1)[0] + "_frames.npy"
            _np.save(npy_out, _np.stack(frames))
            print(f"'imageio' not installed (pip install imageio), so raw frames "
                  f"were saved as a numpy array instead -> {npy_out}")


if __name__ == "__main__":
    main()
