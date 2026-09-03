#!/usr/bin/env python3
import rospy
import numpy as np
import tf2_ros
import math
from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField, LaserScan
from std_msgs.msg import Header
import sensor_msgs.point_cloud2 as pc2

class FakeG1Sensors:
    def __init__(self):
        rospy.init_node('fake_g1_sensors')

        # Publishers
        self.scan_pub = rospy.Publisher('/scan', LaserScan, queue_size=5)
        self.cloud_pub = rospy.Publisher('/livox/lidar', PointCloud2, queue_size=5)
        self.rgb_pub = rospy.Publisher('/camera/color/image_raw', Image, queue_size=5)
        self.depth_pub = rospy.Publisher('/camera/aligned_depth_to_color/image_raw', Image, queue_size=5)
        self.info_pub = rospy.Publisher('/camera/color/camera_info', CameraInfo, queue_size=5)
        
        # Subscriber for teleop movement
        self.cmd_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_cb)

        # Pose & Motion state
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.vx = 0.0
        self.vth = 0.0
        self.last_time = rospy.Time.now()

        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.static_tf = tf2_ros.StaticTransformBroadcaster()

        self.publish_static_transforms()
        rospy.loginfo("Fake G1 Sensor Simulator Running...")

    def cmd_cb(self, msg):
        self.vx = msg.linear.x
        self.vth = msg.angular.z

    def publish_static_transforms(self):
        # base_link -> laser_frame
        t_laser = TransformStamped()
        t_laser.header.stamp = rospy.Time.now()
        t_laser.header.frame_id = "base_link"
        t_laser.child_frame_id = "laser_frame"
        t_laser.transform.translation.z = 0.4
        t_laser.transform.rotation.w = 1.0

        # base_link -> camera_link
        t_cam = TransformStamped()
        t_cam.header.stamp = rospy.Time.now()
        t_cam.header.frame_id = "base_link"
        t_cam.child_frame_id = "camera_link"
        t_cam.transform.translation.x = 0.1
        t_cam.transform.translation.z = 0.5
        t_cam.transform.rotation.w = 1.0

        self.static_tf.sendTransform([t_laser, t_cam])

    def update_pose(self):
        now = rospy.Time.now()
        dt = (now - self.last_time).to_sec()
        self.last_time = now

        delta_x = (self.vx * math.cos(self.th)) * dt
        delta_y = (self.vx * math.sin(self.th)) * dt
        delta_th = self.vth * dt

        self.x += delta_x
        self.y += delta_y
        self.th += delta_th

        # Broadcast odom -> base_link
        t_odom = TransformStamped()
        t_odom.header.stamp = now
        t_odom.header.frame_id = "odom"
        t_odom.child_frame_id = "base_link"
        t_odom.transform.translation.x = self.x
        t_odom.transform.translation.y = self.y
        t_odom.transform.rotation.z = math.sin(self.th / 2.0)
        t_odom.transform.rotation.w = math.cos(self.th / 2.0)
        self.tf_broadcaster.sendTransform(t_odom)

    def publish_sensors(self):
        now = rospy.Time.now()

        # 1. 2D LaserScan (360-degree scan for AMCL)
        scan = LaserScan()
        scan.header.stamp = now
        scan.header.frame_id = "laser_frame"
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = math.pi / 180.0
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = 0.1
        scan.range_max = 30.0
        scan.ranges = [3.0] * 360
        self.scan_pub.publish(scan)

        # 2. PointCloud2
        header = Header(stamp=now, frame_id="laser_frame")
        points = []
        for a in np.linspace(-np.pi, np.pi, 180):
            r = 3.0
            points.append([r * math.cos(a), r * math.sin(a), 0.0])
            points.append([r * math.cos(a), r * math.sin(a), 0.2])
        
        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1)
        ]
        pc = pc2.create_cloud(header, fields, points)
        self.cloud_pub.publish(pc)

        # 3. Camera Info & Images
        img = Image()
        img.header = Header(stamp=now, frame_id="camera_link")
        img.height, img.width = 240, 320
        img.encoding = "rgb8"
        img.step = 320 * 3
        img.data = b'\x80' * (240 * 320 * 3)
        self.rgb_pub.publish(img)

        depth_img = Image()
        depth_img.header = Header(stamp=now, frame_id="camera_link")
        depth_img.height, depth_img.width = 240, 320
        depth_img.encoding = "16UC1"
        depth_img.step = 320 * 2
        depth_img.data = b'\x05\x00' * (240 * 320)
        self.depth_pub.publish(depth_img)

        info = CameraInfo()
        info.header = Header(stamp=now, frame_id="camera_link")
        info.height, info.width = 240, 320
        info.K = [200.0, 0.0, 160.0, 0.0, 200.0, 120.0, 0.0, 0.0, 1.0]
        self.info_pub.publish(info)

if __name__ == '__main__':
    node = FakeG1Sensors()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        node.update_pose()
        node.publish_sensors()
        rate.sleep()
