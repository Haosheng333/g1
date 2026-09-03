#!/usr/bin/env python3
import rospy
import os
import yaml
import time
from std_srvs.srv import Empty, EmptyResponse

class MapManagerNode:
    def __init__(self):
        rospy.init_node('map_manager_node')
        
        self.maps_dir = rospy.get_param('~maps_dir', os.path.expanduser('~/catkin_ws/src/g1_navigation/maps'))
        os.makedirs(self.maps_dir, exist_ok=True)
        
        self.save_service = rospy.Service('/slam/manager/save_current_map', Empty, self.handle_save_map)
        rospy.loginfo("Map Manager Node active. Listening on /slam/manager/save_current_map")

    def handle_save_map(self, req):
        rospy.loginfo("Triggering map save sequence...")
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        
        # 1. Attempt to call RTAB-Map save/backup service if available
        rtabmap_saved = False
        for srv_name in ['/rtabmap/save_backup', '/rtabmap/backup']:
            try:
                rospy.wait_for_service(srv_name, timeout=1.0)
                save_srv = rospy.ServiceProxy(srv_name, Empty)
                save_srv()
                rospy.loginfo(f"Successfully called {srv_name}")
                rtabmap_saved = True
                break
            except (rospy.ROSException, rospy.ServiceException):
                continue

        if not rtabmap_saved:
            rospy.logwarn("RTAB-Map service not found directly; relying on standard database continuous commit.")

        # 2. Write manifest.yaml
        manifest_path = os.path.join(self.maps_dir, "manifest.yaml")
        manifest_data = {
            'map_name': f"g1_map_{timestamp}",
            'created_at': timestamp,
            'db_file': 'rtabmap.db',
            'sensor_config': 'Livox Mid-360 + RealSense RGB-D',
            'frame_id': 'map',
            'resolution': 0.05
        }
        with open(manifest_path, 'w') as f:
            yaml.dump(manifest_data, f, default_flow_style=False)

        # 3. Create semantic.yaml placeholder
        semantic_path = os.path.join(self.maps_dir, "semantic.yaml")
        if not os.path.exists(semantic_path):
            semantic_data = {
                'waypoints': {
                    'charging_station': {'x': 0.0, 'y': 0.0, 'yaw': 0.0},
                    'table': {'x': 2.5, 'y': 1.2, 'yaw': 1.57}
                }
            }
            with open(semantic_path, 'w') as f:
                yaml.dump(semantic_data, f, default_flow_style=False)

        rospy.loginfo(f"Map artifacts successfully saved to {self.maps_dir}")
        return EmptyResponse()

if __name__ == '__main__':
    try:
        node = MapManagerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
