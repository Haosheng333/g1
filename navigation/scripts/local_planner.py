import math

from geometry_msgs.msg import Twist

# Arrival tolerance, doc section 7: 0.10 m / 0.1 rad.
XY_TOL = 0.10
YAW_TOL = 0.1


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class PurePursuitController:
    """Pure pursuit + final in-place yaw alignment.

    (The full holonomic three-phase DWA of doc 4.3 is section C of the fix list;
    this class only gets the ARRIVAL rule of section A right: reached means
    distance < XY_TOL AND yaw error < YAW_TOL.)
    """

    def __init__(self, lookahead_dist=0.3, max_v=0.2, max_w=0.5,
                 xy_tol=XY_TOL, yaw_tol=YAW_TOL):
        self.lookahead_dist = lookahead_dist
        self.max_v = max_v
        self.max_w = max_w
        self.xy_tol = xy_tol
        self.yaw_tol = yaw_tol

    def compute_cmd_vel(self, current_pose, path, goal_yaw=None):
        """-> (Twist, reached).  goal_yaw=None means 'any heading is fine'."""
        cmd = Twist()
        if not path:
            return cmd, True

        rx, ry, ryaw = current_pose
        goal = path[-1]
        dist_to_goal = math.hypot(goal[0] - rx, goal[1] - ry)

        # TERMINAL: inside the position tolerance -> align heading, then done.
        if dist_to_goal < self.xy_tol:
            if goal_yaw is None:
                return cmd, True
            yaw_err = wrap(goal_yaw - ryaw)
            if abs(yaw_err) < self.yaw_tol:
                return cmd, True
            w = max(0.1, min(self.max_w, 1.5 * abs(yaw_err)))
            cmd.angular.z = math.copysign(w, yaw_err)
            return cmd, False

        # Look-ahead point: search FORWARD from the path point nearest to the robot.
        # (Scanning from the start of the path picks points BEHIND the robot once it
        # has moved more than lookahead_dist, and the robot circles forever.)
        nearest = min(range(len(path)),
                      key=lambda i: math.hypot(path[i][0] - rx, path[i][1] - ry))
        target_pt = goal
        for pt in path[nearest:]:
            if math.hypot(pt[0] - rx, pt[1] - ry) >= self.lookahead_dist:
                target_pt = pt
                break

        target_angle = math.atan2(target_pt[1] - ry, target_pt[0] - rx)
        heading_error = wrap(target_angle - ryaw)

        cmd.linear.x = min(self.max_v, 0.5 * dist_to_goal)
        cmd.angular.z = max(-self.max_w, min(self.max_w, 1.5 * heading_error))
        return cmd, False
