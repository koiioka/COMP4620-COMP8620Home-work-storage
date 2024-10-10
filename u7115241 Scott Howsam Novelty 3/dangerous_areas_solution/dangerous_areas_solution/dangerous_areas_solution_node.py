import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
import math
from collections import deque

# TF2
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class DangerousAreasSolutionNode(Node):
    def __init__(self):
        super().__init__('dangerous_areas_solution_node')
        # Subscribe the movement changes
        self.vel_subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)
        # Publish movement commands
        self.vel_publisher = self.create_publisher(
            Twist,
            'cmd_vel_modified',
            10)
        # Publish navigation goal changes
        self.nav_goal_publisher = self.create_publisher(
            PoseStamped,
            'goal_pose',
            10)

        # Position variables
        self.last_position = None
        self.last_linear_vel = 0.0
        self.sec_last_linear_vel = 0.0
        self.max_position_error_per_second = 0.4
        self.safe_poses = deque(maxlen=20)

        # Orientation variables
        self.last_orientation = None
        self.last_rotation = None
        self.last_angular_vel = 0.0

        # Callback time variables
        self.callback_frequency = 0.2
        self.last_odom_time = self.get_clock().now()
        self.last_cmd_vel_time = self.get_clock().now()

        # Dangerous area variables
        self.danger_x = None
        self.danger_y = None
        self.backtracking = False
        self.backtrack_distance = 0.5

        # TF2 setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(0.1, self.check_robot_transform)

        # Static map corner values
        self.static_corners = {
            'maps/world_1.yaml': [
                (-0.5, 2.0),
                (-0.5, -1.0),
                (-3.5, -1.0),
                (-3.5, 2.0)
            ],
            'maps/world_2.yaml': [
                (4.2, 2.6),
                (4.2, -1.75),
                (1.8, -0.3),
                (0.0, -1.75),
                (0.0, 2.6)
            ]
        }
        # Get the correct corners for the map
        self.declare_parameter('map_file', '')
        self.map_file = self.get_parameter('map_file').value
        self.corners = self.static_corners[self.map_file]

        # Set goal position to start with
        # 1a -4.0, 0.98
        # 1b -3.6, 1.5
        # 1c 0.1,  1.25
        # 2a 0.7,  -1.5
        # 2b 1.6,  -0.5
        # 2c 2.35, -1.4
        self.goal_x = -4.0
        self.goal_y = 0.98
        self.init_counter = 0

        # Recovery goal setting variables
        self.recovery_goals = []
        self.initial_x = None
        self.initial_y = None

    # Get the current local transform of the robot detected in RViz
    def check_robot_transform(self):
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            self.process_robot_transform(transform)
        except TransformException as ex:
            self.get_logger().warn(f'Could not transform map to base_link: {ex}')

    # Use the robot transform to detect and avoid dangerous areas
    def process_robot_transform(self, transform):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_odom_time).nanoseconds / 1e9  # Convert to seconds
        # To prevent message spam from completely disabling the robot, limit message frequency
        if dt < self.callback_frequency:
            return
        self.last_odom_time = current_time
    
        # Calculate expected position, and calculate error compared to actual position
        position_error_per_second = 0.0
        if self.last_position is not None:
            expected_x = self.last_position.x + self.sec_last_linear_vel * dt * math.cos(self.last_orientation)
            expected_y = self.last_position.y + self.sec_last_linear_vel * dt * math.sin(self.last_orientation)
            
            actual_x = transform.transform.translation.x
            actual_y = transform.transform.translation.y
            
            position_error = self.euclidean_distance(expected_x, expected_y, actual_x, actual_y)
            position_error_per_second = position_error / dt

            self.init_counter += 1
        
        # After waiting 10 messages for initial pose to load in RViz, set the initial location and goal
        if self.init_counter == 10:
            self.initial_x = transform.transform.translation.x
            self.initial_y = transform.transform.translation.y
            # Set the initial goal location
            initial_goal = PoseStamped()
            initial_goal.header.stamp = self.get_clock().now().to_msg()
            initial_goal.pose.position.x = self.goal_x
            initial_goal.pose.position.y = self.goal_y
            initial_goal.pose.orientation.x = transform.transform.rotation.x
            initial_goal.pose.orientation.y = transform.transform.rotation.y
            initial_goal.pose.orientation.z = transform.transform.rotation.z
            initial_goal.pose.orientation.w = transform.transform.rotation.w
            initial_goal.header.frame_id = "map"
            self.nav_goal_publisher.publish(initial_goal)

        # If position error is above its maximum allowable level, we likely have a sensor error, initiate backtracking
        if position_error_per_second > self.max_position_error_per_second:
            # Difference between current and last x and y to account for direction of movement
            danger_diff_x = transform.transform.translation.x - self.last_position.x
            danger_diff_y = transform.transform.translation.y - self.last_position.y
            # Create an estimated center of the dangerous region for avoidance
            self.danger_x = transform.transform.translation.x + 5 * danger_diff_x
            self.danger_y = transform.transform.translation.y + 5 * danger_diff_y
            
            # Initiate backtracking
            self.backtracking = True
            self.initiate_backtracking()
        
        # If backtracking, maintain backward acceleration
        if self.backtracking:
            backwards_twist = Twist()
            backwards_twist.linear.x = self.last_linear_vel - 0.05
            self.vel_publisher.publish(backwards_twist)
        # When not backtracking we are safe, append position as a safe position
        else:
            self.safe_poses.append(transform.transform.translation)

        # Update last known position and orientation
        self.last_position = transform.transform.translation
        self.last_orientation = self.quaternion_to_yaw(transform.transform.rotation)
        self.last_rotation = transform.transform.rotation

    # Callback function for changes in robot velocity
    def cmd_vel_callback(self, msg):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_cmd_vel_time).nanoseconds / 1e9  # Convert to seconds
        # To prevent message spam from completely disabling the robot, limit message frequency
        if dt < self.callback_frequency:
            return
        self.last_cmd_vel_time = current_time

        # Second last linear velocity is stored and used to account for the possibility of cmd_vel_callbacks before an odom_callback
        # If this happened, the last velocity could be a sensor error velocity, and this would be used to calculate what the correct odom position should be, which would be bad for detection
        self.sec_last_linear_vel = self.last_linear_vel
        # Note that the robot only moves forward relative to its current orientation, and so msg.linear.x is the entire linear velocity
        self.last_linear_vel = msg.linear.x

        # Only rotating on xy plane, which is z axis rotation
        self.last_angular_vel = msg.angular.z

        # Check whether backtracking from dangerous area has been completed, and if so navigate to the goal around the dangerous area
        if self.backtracking and self.sec_last_linear_vel > -0.005 and self.sec_last_linear_vel < 0.005 \
                             and self.last_angular_vel > -0.005 and  self.last_angular_vel < 0.005:
            self.backtracking = False

            # Set and begin the list of recovery goals
            self.set_recovery_goals()
            self.publish_next_recovery_goal()

        # If an intermediary recovery goal has been achieved, start the next one
        if len(self.recovery_goals) > 0 and self.sec_last_linear_vel > -0.005 and self.sec_last_linear_vel < 0.005 \
                                        and self.last_angular_vel > -0.005 and  self.last_angular_vel < 0.005:
            self.publish_next_recovery_goal()  

        # If the robot is stuck with no recovery goals, head to main goal
        if  len(self.recovery_goals) == 0 and not self.backtracking and self.sec_last_linear_vel > -0.01 and self.sec_last_linear_vel < 0.01 \
                                        and self.last_angular_vel > -0.01 and  self.last_angular_vel < 0.01:
            self.recovery_goals.append((self.goal_x, self.goal_y))
            self.publish_next_recovery_goal()

    # Publish the next recovery goal to NavFn
    def publish_next_recovery_goal(self):
        restored_goal = PoseStamped()
        restored_goal.header.stamp = self.get_clock().now().to_msg()
        restored_goal.pose.position.x = self.recovery_goals[0][0]
        restored_goal.pose.position.y = self.recovery_goals[0][1]
        restored_goal.pose.orientation.x = self.last_rotation.x
        restored_goal.pose.orientation.y = self.last_rotation.y
        restored_goal.pose.orientation.z = self.last_rotation.z
        restored_goal.pose.orientation.w = self.last_rotation.w
        restored_goal.header.frame_id = "map"
        self.nav_goal_publisher.publish(restored_goal)
        self.recovery_goals.pop(0)

    # Find a route around the map which is likely to avoid the dangerous area
    def set_recovery_goals(self):
        # Find the edge closest to the danger point
        closest_edge = None
        min_distance = float('inf')
        for i in range(len(self.corners)):
            x1, y1 = self.corners[i]
            x2, y2 = self.corners[(i + 1) % len(self.corners)]
            mid_x = (x1 + x2) / 2
            mid_y = (y1 + y2) / 2
            distance = self.euclidean_distance(mid_x, mid_y, self.danger_x, self.danger_y)
            if distance < min_distance:
                min_distance = distance
                closest_edge = (i, (i + 1) % len(self.corners))
    
        # Get all valid edges excluding the dangerous edge
        safe_edges = [(i, (i + 1) % len(self.corners)) for i in range(len(self.corners)) if (i, (i + 1) % len(self.corners)) != closest_edge]
    
        # Find the corner closest to the initial position
        init_corner = min(range(len(self.corners)), key=lambda i: self.euclidean_distance(self.corners[i][0], self.corners[i][1], self.initial_x, self.initial_y))
    
        # Find the corner closest to the goal position
        goal_corner = min(range(len(self.corners)), key=lambda i: self.euclidean_distance(self.corners[i][0], self.corners[i][1], self.goal_x, self.goal_y))
    
        # Generate recovery goals by moving through safe edges
        self.recovery_goals = []
        current_corner = init_corner
        clockwise = None # used to prevent infinite looping between points
        while current_corner != goal_corner:
            self.recovery_goals.append(self.corners[current_corner])
            for edge in safe_edges:
                if current_corner == edge[0] and (clockwise is None or clockwise):
                    current_corner = edge[1]
                    clockwise = True
                    break
                if current_corner == edge[1] and (clockwise is None or not clockwise):
                    current_corner = edge[0]
                    clockwise = False
                    break
        self.recovery_goals.append(self.corners[goal_corner])
        self.recovery_goals.append((self.goal_x, self.goal_y))

    # Publish a new goal behind the robot to return to a safe position
    def initiate_backtracking(self):
        backtrack_goal = PoseStamped()
        backtrack_goal.header.stamp = self.get_clock().now().to_msg()
        safe_pose = self.safe_poses[0]
        backtrack_goal.pose.position.x = safe_pose.x
        backtrack_goal.pose.position.y = safe_pose.y
        backtrack_goal.pose.orientation.x = self.last_rotation.x
        backtrack_goal.pose.orientation.y = self.last_rotation.y
        backtrack_goal.pose.orientation.z = self.last_rotation.z
        backtrack_goal.pose.orientation.w = self.last_rotation.w
        backtrack_goal.header.frame_id = "map"
        self.nav_goal_publisher.publish(backtrack_goal)

    # Calculate euclidean distance between two 2D points
    def euclidean_distance(self, x1, y1, x2, y2):
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

    # Convert the default quaternion rotation format into a single radian rotation value around the flat axis
    def quaternion_to_yaw(self, orientation):
        # Extract quaternion components
        x = orientation.x
        y = orientation.y
        z = orientation.z
        w = orientation.w
        
        # Compute yaw (rotation around the Z axis)
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y**2 + z**2))
        return yaw

def main(args=None):
    rclpy.init(args=args)
    node = DangerousAreasSolutionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()