#!/usr/bin/env python3
"""Generate the synthetic furnished room used by the simulator and the offline acceptance tests.

    python3 tools/make_sim_map.py            # writes navigation/sim/sim_room/{map.pgm,map.yaml,semantic.yaml}

Same file layout and conventions as a real slam_manager save (map frame z-up, PGM row 0 = north,
254 free / 0 occupied / 205 unknown, origin = lower-left corner), so the simulator exercises the SAME
grid_composer / navigation code paths as the robot.

Room (metres, map frame): interior x 0..10, y 0..8, 0.1 m walls, partition wall at x = 5 with a 1.0 m door
(y 3.5..4.5), table 1.0 x 1.0 at (2.5, 2.0), chair 0.5 x 0.5 at (1.25, 3.25), sofa 1.8 x 0.8 at (7.4, 5.4).
"""
import os

import numpy as np

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
OUT = os.path.join(REPO, "navigation", "sim", "sim_room")
RES = 0.05
OX, OY = -1.0, -1.0          # origin: 1 m of unknown padding around the room
W, H = 220, 180              # 11 m x 9 m


def box(grid, x0, x1, y0, y1):
    i0, i1 = int(round((x0 - OX) / RES)), int(round((x1 - OX) / RES))
    j0, j1 = int(round((y0 - OY) / RES)), int(round((y1 - OY) / RES))
    grid[j0:j1, i0:i1] = 0          # 0 = occupied here (grid of pixel values)


def main():
    img = np.full((H, W), 205, np.uint8)               # unknown
    box_free = lambda x0, x1, y0, y1: img.__setitem__(  # noqa: E731
        (slice(int(round((y0 - OY) / RES)), int(round((y1 - OY) / RES))),
         slice(int(round((x0 - OX) / RES)), int(round((x1 - OX) / RES)))), 254)
    box_free(0.0, 10.0, 0.0, 8.0)                      # interior
    for (x0, x1, y0, y1) in [(-0.1, 0.0, -0.1, 8.1), (10.0, 10.1, -0.1, 8.1),     # outer walls
                             (-0.1, 10.1, -0.1, 0.0), (-0.1, 10.1, 8.0, 8.1)]:
        box(img, x0, x1, y0, y1)
    box(img, 4.95, 5.05, 0.0, 3.5)                     # partition with a door 3.5 .. 4.5
    box(img, 4.95, 5.05, 4.5, 8.0)
    box(img, 2.0, 3.0, 1.5, 2.5)                       # table
    box(img, 1.0, 1.5, 3.0, 3.5)                       # chair
    box(img, 6.5, 8.3, 5.0, 5.8)                       # sofa
    os.makedirs(OUT, exist_ok=True)
    pgm = img[::-1, :]                                 # grid row 0 = lowest y  ->  image row 0 = top
    with open(os.path.join(OUT, "map.pgm"), "wb") as f:
        f.write(("P5\n# synthetic sim room, %.2f m/pix\n%d %d\n255\n" % (RES, W, H)).encode() + pgm.tobytes())
    with open(os.path.join(OUT, "map.yaml"), "w") as f:
        f.write("image: map.pgm\nresolution: %f\norigin: [%f, %f, 0.000000]\nnegate: 0\n"
                "occupied_thresh: 0.65\nfree_thresh: 0.196\n" % (RES, OX, OY))
    with open(os.path.join(OUT, "semantic.yaml"), "w") as f:
        f.write("# Synthetic sim room: coordinates are the object centres in the z-up map frame.\n"
                "waypoints:\n"
                "  table: {x: 2.5, y: 2.0, yaw: 0.0}\n"
                "  sofa:  {x: 7.4, y: 5.4, yaw: 0.0}\n")
    print("wrote", OUT)


if __name__ == "__main__":
    main()
