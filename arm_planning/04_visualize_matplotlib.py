"""
04_visualize_matplotlib.py
===========================
Quick, zero-hassle visual validation: no meshes, no MuJoCo, no URDF file
needed at all -- just matplotlib and the joint_trajectory.csv that
03_plan_path.py already produced. Draws:

  1. A 3D stick-figure of the arm at each waypoint (torso -> shoulder ->
     elbow -> wrist -> hand), colored from start (light) to end (dark).
  2. The front workspace bounding box, so you can see the path stays
     inside the zone the network was trained on.
  3. The requested target points (x) vs. what the arm actually achieves
     (o), so you can visually confirm the small errors reported by
     stage 3.

Run this first, before bothering with MuJoCo -- it answers "does the
motion look sane" in a few seconds with nothing extra to install.
"""

import argparse

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Line3DCollection

import arm_config as cfg


def load_trajectory(csv_path: str):
    data = np.genfromtxt(csv_path, delimiter=",", names=True)
    n = data.shape[0]
    target_xyz = np.stack([data["x"], data["y"], data["z"]], axis=1)
    joint_cols = [f"{name}" for name in cfg.JOINT_NAMES]
    joints = np.stack([data[name] for name in joint_cols], axis=1)
    err_cm = data["cartesian_err_cm"]
    return target_xyz, joints, err_cm


def plot_workspace_box(ax):
    (xlo, xhi) = cfg.WORKSPACE_BOUNDS["x"]
    (ylo, yhi) = cfg.WORKSPACE_BOUNDS["y"]
    (zlo, zhi) = cfg.WORKSPACE_BOUNDS["z"]
    corners = np.array([[x, y, z] for x in (xlo, xhi) for y in (ylo, yhi) for z in (zlo, zhi)])
    edges = [
        (0, 1), (0, 2), (0, 4), (3, 1), (3, 2), (3, 7),
        (5, 1), (5, 4), (5, 7), (6, 2), (6, 4), (6, 7),
    ]
    segs = [(corners[i], corners[j]) for i, j in edges]
    ax.add_collection3d(Line3DCollection(segs, colors="gray", linewidths=0.6, linestyles="dashed"))


def main():
    parser = argparse.ArgumentParser(description="Visualize the planned trajectory with matplotlib.")
    parser.add_argument("--trajectory", type=str, default="joint_trajectory.csv")
    parser.add_argument("--out", type=str, default="trajectory_plot.png")
    args = parser.parse_args()

    target_xyz, joints, err_cm = load_trajectory(args.trajectory)
    n_steps = joints.shape[0]

    chain = cfg.fk_chain(joints)  # (n_steps, 6, 3): torso -> shoulder..wrist -> hand
    achieved_xyz = chain[:, -1, :]

    fig = plt.figure(figsize=(9, 8))
    ax = fig.add_subplot(111, projection="3d")

    plot_workspace_box(ax)

    cmap = plt.get_cmap("viridis")
    for i in range(n_steps):
        color = cmap(i / max(n_steps - 1, 1))
        xs, ys, zs = chain[i, :, 0], chain[i, :, 1], chain[i, :, 2]
        ax.plot(xs, ys, zs, "-o", color=color, markersize=3, linewidth=1.5, alpha=0.8)

    ax.scatter(*target_xyz.T, marker="x", color="red", s=40, label="requested target")
    ax.scatter(*achieved_xyz.T, marker="o", facecolors="none", edgecolors="black",
               s=50, label="achieved (network+FK)")

    ax.set_xlabel("x (m, forward)")
    ax.set_ylabel("y (m, lateral)")
    ax.set_zlabel("z (m, vertical)")
    ax.set_title(f"Planned arm trajectory ({n_steps} steps)\n"
                 f"mean error: {err_cm.mean():.2f} cm, max: {err_cm.max():.2f} cm")
    ax.legend(loc="upper left")
    ax.view_init(elev=20, azim=-60)

    plt.tight_layout()
    plt.savefig(args.out, dpi=150)
    print(f"Saved plot -> {args.out}")
    try:
        plt.show()
    except Exception:
        pass  # no display available (e.g. headless server) -- the PNG is already saved


if __name__ == "__main__":
    main()
