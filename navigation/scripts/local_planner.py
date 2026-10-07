import math
from geometry_msgs.msg import Twist

class PurePursuitController:
    def __init__(self, lookahead_dist=0.3, max_v=0.2, max_w=0.5):
        self.lookahead_dist = lookahead_dist
        self.max_v = max_v
        self.max_w = max_w

    def compute_cmd_vel(self, current_pose, path):
        cmd = Twist()
        if not path or len(path) == 0:
            return cmd, True

        rx, ry, ryaw = current_pose

        target_pt = path[-1]
        for pt in path:
            dist = math.hypot(pt[0] - rx, pt[1] - ry)
            if dist >= self.lookahead_dist:
                target_pt = pt
                break

        dist_to_goal = math.hypot(path[-1][0] - rx, path[-1][1] - ry)
        if dist_to_goal < 0.1:
            return cmd, True

        target_angle = math.atan2(target_pt[1] - ry, target_pt[0] - rx)
        heading_error = math.atan2(math.sin(target_angle - ryaw), math.cos(target_angle - ryaw))

        cmd.linear.x = min(self.max_v, 0.5 * dist_to_goal)
        cmd.angular.z = max(-self.max_w, min(self.max_w, 1.5 * heading_error))

        return cmd, False
