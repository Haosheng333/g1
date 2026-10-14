#!/usr/bin/env python3
"""sim_world - a tiny kinematic world so navigation can be tested without the robot (replaces
fake_g1_sensors.py, whose constant 3 m wall could never test "reach a goal around furniture").

It stands in for: the walking controller, the pelvis TF, the localization flag and the live lidar layer.

    /locomotion/cmd_vel  --(holonomic robot, first-order velocity lag)-->  pose
    TF  map -> pelvis                          (50 Hz, ground truth)
    /slam/manager/state   Bool latched True after ~localized_delay seconds (so the RELOCALIZING gate runs)
    /obstacle_cloud       PointCloud2 in `map`: 360-ray lidar ray-cast on the map + the optional dynamic box
    /sim/collision_count  Int32: how many times the ground-truth footprint touched an obstacle (must stay 0)
    srv /sim/toggle_obstacle (std_srvs/Trigger): add / remove a box in the way (live-layer test)

Collision truth uses the SAME 5-circle footprint as the planner (costmap.Costmap) but with zero margin, so a
planner that cuts corners is caught.  When the robot would collide it is stopped instead of moving.

Params: ~map_yaml (required)  ~start [x, y, yaw]  ~localized_delay 5.0  ~tau 0.25  ~dynamic_box [x, y, half_size]
"""
import math
import os
import sys
import threading

import numpy as np
import rospy
import tf2_ros
from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, Header, Int32
from std_srvs.srv import Trigger, TriggerResponse

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from costmap import Costmap, OBSTACLE  # noqa: E402
from g1_slam import cloud_utils as cu  # noqa: E402
from g1_slam import grid_utils as gu  # noqa: E402

LIMITS = (0.5, 0.3, 1.0)       # same ceiling as cmd_vel_gate


class SimWorld:
    def __init__(self):
        rospy.init_node("sim_world")
        p = rospy.get_param
        grid, self.res, self.ox, self.oy = gu.load_map_yaml(p("~map_yaml"))
        self.base = grid.copy()
        self.dyn = grid.copy()
        self.box_on = False
        bx, by, bh = p("~dynamic_box", [3.0, 1.0, 0.2])
        self._box_cells = self._cells(bx, by, bh)
        self.pose = list(p("~start", [1.0, 1.0, 0.0]))
        self.tau = float(p("~tau", 0.25))
        self.localized_delay = float(p("~localized_delay", 5.0))
        self.cmd_timeout = float(p("~cmd_timeout", 0.5))
        self.vel = [0.0, 0.0, 0.0]
        self.cmd, self.cmd_t = (0.0, 0.0, 0.0), 0.0
        self.collisions, self.in_collision = 0, False
        self.lock = threading.Lock()
        self._rebuild_truth()

        self.br = tf2_ros.TransformBroadcaster()
        self.cloud_pub = rospy.Publisher("/obstacle_cloud", PointCloud2, queue_size=1)
        self.state_pub = rospy.Publisher("/slam/manager/state", Bool, queue_size=1, latch=True)
        self.col_pub = rospy.Publisher("/sim/collision_count", Int32, queue_size=1, latch=True)
        self.state_pub.publish(Bool(data=False))
        self.col_pub.publish(Int32(data=0))
        rospy.Subscriber(p("~cmd_topic", "/locomotion/cmd_vel"), Twist, self.cmd_cb, queue_size=5)
        rospy.Service("/sim/toggle_obstacle", Trigger, self.toggle_cb)
        self.t0 = rospy.Time.now().to_sec()
        self.localized = False
        rospy.Timer(rospy.Duration(0.1), self.lidar_cb)
        rospy.loginfo("[sim_world] start=%s map=%s localized after %.1f s", self.pose, p("~map_yaml"), self.localized_delay)

    # ---------------------------------------------------------------- world
    def _cells(self, x, y, half):
        i0, i1 = int((x - half - self.ox) / self.res), int((x + half - self.ox) / self.res)
        j0, j1 = int((y - half - self.oy) / self.res), int((y + half - self.oy) / self.res)
        return (slice(j0, j1), slice(i0, i1))

    def _rebuild_truth(self):
        g = self.base.copy()
        if self.box_on:
            g[self._box_cells] = OBSTACLE
        self.dyn = g
        # zero collision margin, unknown counts as lethal: this is the ground truth, not the planner's view
        self.truth = Costmap.from_grid(g, self.res, self.ox, self.oy, collision_margin=0.0)

    def toggle_cb(self, _req):
        with self.lock:
            self.box_on = not self.box_on
            self._rebuild_truth()
        return TriggerResponse(success=True, message="dynamic obstacle %s" % ("ON" if self.box_on else "OFF"))

    def cmd_cb(self, msg):
        with self.lock:
            self.cmd = (max(-LIMITS[0], min(LIMITS[0], msg.linear.x)),
                        max(-LIMITS[1], min(LIMITS[1], msg.linear.y)),
                        max(-LIMITS[2], min(LIMITS[2], msg.angular.z)))
            self.cmd_t = rospy.Time.now().to_sec()

    # ------------------------------------------------------------ simulation
    def step(self, dt):
        now = rospy.Time.now().to_sec()
        with self.lock:
            cmd = self.cmd if now - self.cmd_t <= self.cmd_timeout else (0.0, 0.0, 0.0)
            truth = self.truth
        a = dt / (self.tau + dt)
        self.vel = [v + a * (c - v) for v, c in zip(self.vel, cmd)]
        x, y, th = self.pose
        nx = x + (self.vel[0] * math.cos(th) - self.vel[1] * math.sin(th)) * dt
        ny = y + (self.vel[0] * math.sin(th) + self.vel[1] * math.cos(th)) * dt
        nth = th + self.vel[2] * dt
        if truth.footprint_clearance(nx, ny, nth) < 0.0 and \
                truth.footprint_clearance(nx, ny, nth) <= truth.footprint_clearance(x, y, th):
            self.vel = [0.0, 0.0, 0.0]                      # blocked by the world
            if not self.in_collision:
                self.collisions += 1
                self.in_collision = True
                self.col_pub.publish(Int32(data=self.collisions))
                rospy.logerr("[sim_world] COLLISION #%d at (%.2f, %.2f, %.2f)", self.collisions, x, y, th)
        else:
            self.pose = [nx, ny, nth]
            self.in_collision = False
        if not self.localized and now - self.t0 >= self.localized_delay:
            self.localized = True
            self.state_pub.publish(Bool(data=True))
            rospy.loginfo("[sim_world] localized flag -> True")

    def broadcast(self):
        x, y, th = self.pose
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id, t.child_frame_id = "map", "pelvis"
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = x, y, 0.8
        t.transform.rotation.z, t.transform.rotation.w = math.sin(th / 2), math.cos(th / 2)
        self.br.sendTransform(t)

    def lidar_cb(self, _evt):
        x, y, _ = self.pose
        g = self.dyn
        h, w = g.shape
        n_rays, rng = 360, 6.0
        ang = np.linspace(-math.pi, math.pi, n_rays, endpoint=False)
        steps = np.arange(1, int(rng / self.res) + 1) * self.res
        px = x + np.outer(steps, np.cos(ang))
        py = y + np.outer(steps, np.sin(ang))
        mx = np.floor((px - self.ox) / self.res).astype(int)
        my = np.floor((py - self.oy) / self.res).astype(int)
        inside = (mx >= 0) & (mx < w) & (my >= 0) & (my < h)
        occ = np.zeros(px.shape, bool)
        occ[inside] = g[my[inside], mx[inside]] == OBSTACLE
        occ |= ~inside                                         # leaving the map ends the ray (no hit)
        hit_any = occ.any(axis=0)
        first = occ.argmax(axis=0)
        ok = hit_any & inside[first, np.arange(n_rays)]
        pts = np.stack([px[first, np.arange(n_rays)][ok], py[first, np.arange(n_rays)][ok],
                        np.full(ok.sum(), 0.5)], axis=1)
        header = Header(stamp=rospy.Time.now(), frame_id="map")
        self.cloud_pub.publish(point_cloud2.create_cloud_xyz32(header, pts.tolist()))

    def run(self):
        rate, dt = rospy.Rate(50), 0.02
        while not rospy.is_shutdown():
            self.step(dt)
            self.broadcast()
            rate.sleep()


if __name__ == "__main__":
    try:
        SimWorld().run()
    except rospy.ROSInterruptException:
        pass
