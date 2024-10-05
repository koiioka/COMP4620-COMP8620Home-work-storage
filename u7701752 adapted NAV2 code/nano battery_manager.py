import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
import random
import time

class BatteryManager(Node):
    def __init__(self):
        super().__init__('battery_manager')
        self.battery_level = 90.0  # Adjusted initial battery level to 90 seconds
        self.low_battery_threshold = 10.0  # Adjusted threshold for low battery to 10 seconds
        self.timer = self.create_timer(1.0, self.timer_callback)  # Timer callback every 1 second
        self.publisher = self.create_publisher(Twist, 'cmd_vel_modified', 10)  # Publisher to control robot movement
        self.cmd_vel_subscriber = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)  # Subscribe to velocity commands
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)  # Subscribe to odometry data
        self.nav_goal_publisher = self.create_publisher(PoseStamped, 'goal_pose', 10)  # Publisher to set navigation goals
        self.current_pose = None  # Store the current pose of the robot
        self.current_twist = Twist()  # Store the current velocity command
        self.original_goal = None  # Store the original navigation goal to return to after charging
        self.charging_station_position = (1.1, 1.1)  # Position of the charging station
        self.charging_radius = 0.5  # Radius within which the robot is considered to be at the charging station
        self.moving = False  # Flag to indicate if the robot is currently moving
        self.recharging = False  # Flag to indicate if the robot is currently recharging

    def timer_callback(self):
        # This function is called every second to update the battery status and decide the robot's actions
        if self.recharging:
            # Check if the robot is within the charging station to complete charging
            self.check_charging_station(self.current_pose.position)
        elif self.moving:  # Only decrease battery if the robot is moving and not currently recharging
            if self.battery_level > 0:
                self.battery_level -= 1.0  # Decrease battery level by 1 second
                self.get_logger().info(f'Battery level: {self.battery_level} seconds left')
            if self.battery_level <= self.low_battery_threshold:  # Check if battery is below the low threshold
                self.navigate_to_charging_station()

    def cmd_vel_callback(self, msg):
        # This function handles incoming velocity commands and tracks the robot's movement
        self.current_twist = msg
        self.moving = msg.linear.x != 0 or msg.angular.z != 0  # Check if the robot is moving
        if self.battery_level > self.low_battery_threshold and not self.recharging:  # Allow movement only if the battery is not depleted and not recharging
            self.publisher.publish(msg)

    def odom_callback(self, msg):
        # This function updates the robot's current position using odometry data
        self.current_pose = msg.pose.pose  # Save the robot's current pose

    def navigate_to_charging_station(self):
        """Navigate to the charging station when battery is low."""
        if not self.recharging:  # Prevent the robot from attempting to navigate to the charging station multiple times
            # Save the current goal so that the robot can resume it after charging
            if self.original_goal is None:
                self.original_goal = self.current_goal()
                
            self.recharging = True  # Enter charging mode
            charging_goal = PoseStamped()
            charging_goal.header.frame_id = "map"  # Set the frame of reference to "map"
            charging_goal.pose.position.x = self.charging_station_position[0]
            charging_goal.pose.position.y = self.charging_station_position[1]
            charging_goal.pose.orientation.w = 1.0  # Default orientation
            self.get_logger().info('Navigating to charging station...')
            self.nav_goal_publisher.publish(charging_goal)  # Publish the goal to navigate to the charging station

    def current_goal(self):
        """Return the robot's current position as a navigation goal."""
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.pose = self.current_pose  # Use the current pose as the goal
        return goal

    def check_charging_station(self, current_position):
        # Check if the robot is within the charging radius of the charging station
        distance = self.calculate_distance(current_position, self.charging_station_position)
        if distance < self.charging_radius and self.recharging:
            # Start the charging process at the charging station without delay
            self.get_logger().info('Reached charging station. Battery fully restored.')
            self.battery_level = 90.0  # Restore battery to full
            self.recharging = False  # Exit recharging mode
            self.get_logger().info('Returning to original goal.')
            # After charging, resume the original goal
            if self.original_goal:
                self.nav_goal_publisher.publish(self.original_goal)
                self.original_goal = None  # Clear the saved goal
        elif self.battery_level <= 0 and not distance < self.charging_radius:
            # When out of battery and not at the charging station, simulate stopping for 60 seconds
            self.get_logger().info('Out of battery! Stopping for 60 seconds...')
            time.sleep(60)  # Simulate waiting for 60 seconds to "recharge"
            self.battery_level = 90.0  # Restore battery to full
            self.get_logger().info('Battery restored! Resuming navigation.')
            if self.original_goal:
                self.nav_goal_publisher.publish(self.original_goal)
                self.original_goal = None  # Clear the saved goal

    def calculate_distance(self, current_position, target_position):
        # Calculate the Euclidean distance between the current position and the target position
        return ((current_position.x - target_position[0]) ** 2 + 
                (current_position.y - target_position[1]) ** 2) ** 0.5

def main(args=None):
    # Main entry point for the node
    rclpy.init(args=args)
    node = BatteryManager()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
