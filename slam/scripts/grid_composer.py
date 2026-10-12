#!/usr/bin/env python3
"""grid_composer - composes the TIERED occupancy grid /map (doc section 4.1/4.3).

Reads the map_server-style  map.yaml + map.pgm  that slam_manager's save step
produced (pcd_to_map output, already in the z-UP `map` frame) and publishes

    /map   nav_msgs/OccupancyGrid  (latched, republished at ~1 Hz)

with these int8 tier values (distance to the nearest obstacle):

     -1  UNKNOWN
      0  WALKABLE
     33  RISK       free, within  risk_radius   (default 0.50 m) of an obstacle
     66  DANGER     free, within  danger_radius (default 0.25 m) of an obstacle
    100  OBSTACLE

Params: ~map_yaml (required) ~frame_id (map) ~danger_radius ~risk_radius ~publish_rate
        ~hole_fill_iterations (3: fills isolated unknown cells inside free space)
"""
import rospy
from nav_msgs.msg import MapMetaData, OccupancyGrid

from g1_slam import grid_utils as gu


def main():
    rospy.init_node("grid_composer")
    yaml_path = rospy.get_param("~map_yaml")
    frame = rospy.get_param("~frame_id", "map")
    danger = float(rospy.get_param("~danger_radius", 0.25))
    risk = float(rospy.get_param("~risk_radius", 0.50))
    rate_hz = float(rospy.get_param("~publish_rate", 1.0))
    holes = int(rospy.get_param("~hole_fill_iterations", 3))

    grid, res, ox, oy = gu.load_map_yaml(yaml_path)
    tiers = gu.compose_tiers(grid, res, danger_m=danger, risk_m=risk, hole_fill_iterations=holes)
    h, w = tiers.shape

    msg = OccupancyGrid()
    msg.header.frame_id = frame
    msg.info = MapMetaData()
    msg.info.resolution = res
    msg.info.width = w
    msg.info.height = h
    msg.info.origin.position.x = ox
    msg.info.origin.position.y = oy
    msg.info.origin.orientation.w = 1.0
    msg.data = tiers.flatten().tolist()

    pub = rospy.Publisher("/map", OccupancyGrid, queue_size=1, latch=True)
    rospy.loginfo("[grid_composer] %s -> /map %dx%d @ %.2f m, danger<=%.2f risk<=%.2f (frame %s)",
                  yaml_path, w, h, res, danger, risk, frame)
    rate = rospy.Rate(rate_hz)
    while not rospy.is_shutdown():
        msg.header.stamp = rospy.Time.now()
        msg.info.map_load_time = msg.header.stamp
        pub.publish(msg)
        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
