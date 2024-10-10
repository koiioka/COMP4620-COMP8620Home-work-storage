#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, TransformStamped
import tf2_ros

from collections import deque

class DangerousAreasNode(Node):
    def __init__(self):
        super().__init__('dangerous_areas_node')
        # Subscribe to the position data callback
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.broadcast_frequency = 0.2
        self.last_time = self.get_clock().now()
        self.safe_poses = deque(maxlen=10)

        # Hard code the map being used and link to statically defined dangerous areas
        self.map = "1a"
        self.map_to_coords = {
            "1a": {"x": (-0.25, 1.25), "y": (-1, 0)},
            "1b": {"x": (-0.25, 1.25), "y": (-1, 0)},
            "1c": {"x": (-0.25, 1.25), "y": (-1, 0)},
            "2a": {"x": (0, 2), "y": (-3, -1)},
            "2b": {"x": (0, 2), "y": (-3, -1)},
            "2c": {"x": (1, 3), "y": (-1, 1)},
        }

    def odom_callback(self, msg):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9  # Convert to seconds
        # To prevent message spam from completely disabling the robot, limit message frequency
        if dt < self.broadcast_frequency:
            return
        self.last_time = current_time

        # Grab location data
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Check whether the robot is within the dangerous area
        if self.map_to_coords[self.map]["x"][0] <= x <= self.map_to_coords[self.map]["x"][1] and \
           self.map_to_coords[self.map]["y"][0] <= y <= self.map_to_coords[self.map]["y"][1]:
            # Revert the position (creates model disconnect, emulates sensor error)
            msg.pose.pose = self.safe_poses[0]
            
            # Set all velocities to zero
            msg.twist.twist = Twist()
        else:
            # If not in the dangerous area, we can add a safe pose
            self.safe_poses.append(msg.pose.pose)
        
        self.publish_tf(msg)

    # Publish a new robot transform including any dangerous area interference
    def publish_tf(self, msg):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    dangerous_areas_node = DangerousAreasNode()
    rclpy.spin(dangerous_areas_node)
    dangerous_areas_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()