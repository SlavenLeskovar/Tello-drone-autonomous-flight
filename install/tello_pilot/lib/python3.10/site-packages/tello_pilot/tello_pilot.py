import rclpy
import numpy as np
import math
import time
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from geometry_msgs.msg import PoseArray
from scipy.interpolate import CubicSpline
from nav_msgs.msg import Path
from std_msgs.msg import Header


class TelloPurePursuitNode(Node):
    def __init__(self):
        super().__init__('tello_pure_pursuit_node')

        # Subscribe to the /aruco_poses topic to get the positions of the ArUco markers
        self.aruco_subscriber = self.create_subscription(PoseArray, '/aruco_poses', self.aruco_callback, 10)

        # Publisher to control the drone using /cmd_vel topic
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Publisher to visualize the path in RViz
        self.path_publisher = self.create_publisher(Path, '/generated_path', 10)

        self.lookahead_distance = 0.15  # 10 cm lookahead distance
        self.path = None
        self.current_index = 0
        self.trajectory_generated = False  # Flag to check if the trajectory has been generated
        self.initial_pose = None  # Will hold the first marker position
        self.new_marker_position = None  # Holds the subsequent marker positions
        self.is_moving_forward = False
        self.forward_move_timer = None

        self.Kp = 0.18

        self.last_marker_time = self.get_clock().now()  # Timestamp of the last marker detection
        self.timeout_duration = 3.0  # Stop the drone if no markers are detected for 2 seconds
        self.grace_period = 3.0  # Allow a 3-second grace period after startup

        # Timer for continuously publishing the path in RViz
        self.path_timer = self.create_timer(0.1, self.publish_path_to_rviz)  # Publishes every 0.1 seconds

        # Timer for checking if the marker is still visible
        self.marker_timer = self.create_timer(1.0, self.check_marker_timeout)  # Check every 1 second

        # Set the start time to measure the grace period
        self.start_time = self.get_clock().now()

    def aruco_callback(self, msg: PoseArray):
        """Callback triggered when ArUco markers are detected."""

        # Reset the last marker detection time
        self.last_marker_time = self.get_clock().now()

        # Extract the new marker position
        self.new_marker_position = np.array([
            msg.poses[0].position.z,
            -msg.poses[0].position.x,
            -msg.poses[0].position.y
        ])
        quaternion_angles = [msg.poses[0].orientation.x, msg.poses[0].orientation.y, msg.poses[0].orientation.z, msg.poses[0].orientation.w]
        self.get_logger().info(f"New marker position: x={self.new_marker_position[0]:.2f}, y={self.new_marker_position[1]:.2f}, z={self.new_marker_position[2]:.2f}")
            
        # Convert Quaternion to Euler
        euler_angles = self.quaternion_to_euler(quaternion_angles)
        yaw_angle = euler_angles[2]
        self.get_logger().info(f"Yaw angle: {yaw_angle:.2f} radians")
        

        # If this is the first time, set the initial pose to the current marker position
        if self.initial_pose is None:
            self.initial_pose = self.new_marker_position
            self.get_logger().info(f"Initial Pose set: x={self.initial_pose[0]:.2f}, y={self.initial_pose[1]:.2f}, z={self.initial_pose[2]:.2f}")

        # Calculate the current position: current_position = initial_pose - new_marker_position
        self.current_position = self.initial_pose - self.new_marker_position
        #self.get_logger().info(f"Current Position: x={self.current_position[0]:.2f}, y={self.current_position[1]:.2f}, z={self.current_position[2]:.2f}")

        if not self.trajectory_generated:  # Only generate trajectory if it hasn't been generated yet
            # Define the key points for the spline path
            start_position = np.array([0, 0, 0])
            end_position = self.new_marker_position + np.array([0, 0, 0.15])
            middle_position_from_left = ((start_position + end_position) / 2) - np.array([0, 0.15, 0])
            middle_position_from_right = ((start_position + end_position) / 2) + np.array([0, 0.15, 0])

            """if self.new_marker_position[1] > 0 and abs(yaw_angle) <= 0.07:
                middle_position = middle_position_from_right
            if self.new_marker_position[1] < 0 and abs(yaw_angle) > 0.07:
                middle_position = middle_position_from_right
            if self.new_marker_position[1] < 0 and abs(yaw_angle) <= 0.07:
                middle_position = middle_position_from_left
            if self.new_marker_position[1] > 0 and abs(yaw_angle) > 0.07:
                middle_position = middle_position_from_left"""
            if self.new_marker_position[1] > 0:
                middle_position = middle_position_from_right
            else:
                middle_position = middle_position_from_left

            # Create a path using a cubic spline
            self.path = self.create_spline_path(start_position, middle_position, end_position)
            self.current_index = 0

            # Mark trajectory as generated
            self.trajectory_generated = True
        # Start the Pure Pursuit control loop
        self.pure_pursuit_control()

    def check_marker_timeout(self):
        """Check if the marker detection has timed out."""
        current_time = self.get_clock().now()
        time_since_last_marker = (current_time - self.last_marker_time).nanoseconds * 1e-9  # Convert nanoseconds to seconds
        time_since_start = (current_time - self.start_time).nanoseconds * 1e-9  # Time since node startup

        # Only stop the drone if the grace period has passed
        if time_since_start > self.grace_period:
            if time_since_last_marker > self.timeout_duration:
                #self.get_logger().warn(f"No marker detected for {self.timeout_duration} seconds on topic {self.current_subscription_topic}. Switching topics.")

                # Destroy the current subscription
                '''if self.aruco_subscriber is not None:
                    self.destroy_subscription(self.aruco_subscriber)
                    self.get_logger().info(f"Unsubscribed from /aruco_poses")

                # Subscribe to the new topic /aruco_poses_2
                #self.current_subscription_topic = '/aruco_poses2'
                self.trajectory_generated = False
                self.initial_pose = None 
                self.aruco_subscriber = self.create_subscription(PoseArray, '/aruco_poses2', self.aruco_callback, 10)
                self.get_logger().info(f"Subscribed to /aruco_poses2")'''

                # Stop the drone since no marker was detected
                #self.stop_drone()

    def quaternion_to_euler(self, quaternion_angles):
        roll, pitch, yaw = 0, 0, 0
        x, y, z, w = quaternion_angles
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)
        return roll, pitch, yaw

    def pure_pursuit_control(self):
        if self.is_moving_forward:
            # Drone is moving forward for 2 seconds
            twist = Twist()
            twist.linear.x = 0.3  # Set speed to 0.8
            twist.angular.z = 0.0  # No turning
            self.cmd_vel_publisher.publish(twist)
            return  # Exit to avoid doing anything else while moving forward
        # If the path is not set or completed, stop the drone
        if self.path is None or self.current_index >= len(self.path):
            self.get_logger().info("Path completed or not set.")
            self.stop_drone()
            return

        while self.current_index < len(self.path):
            target_point = self.path[self.current_index]
            distance_to_waypoint = np.linalg.norm(target_point - self.current_position)

            if distance_to_waypoint >= self.lookahead_distance:
                direction = (target_point - self.current_position) / distance_to_waypoint
                twist = Twist()
                twist.linear.x = 0.08  # Move forward at a fixed speed
                yaw_angle_control = np.arctan2(direction[1], direction[0])
                twist.angular.z = yaw_angle_control * self.Kp
                #if self.new_marker_position[0] < 1.3:
                    #self.Kp = 0.25
                    #self.lookahead_distance = 0.15
                if self.new_marker_position[0] < 0.85:
                    self.get_logger().info(f"Marker within 1 meter, moving forward at speed 0.8 for 2 seconds.")
                    self.is_moving_forward = True

                    # Start a timer for 2 seconds to move forward and then stop
                    self.forward_move_timer = self.create_timer(3, self.finish_forward_movement)

                    return
                self.cmd_vel_publisher.publish(twist)
                return
            else:
                self.current_index += 1

        self.get_logger().info("Reached final waypoint.")
        self.stop_drone()
        self.trajectory_generated = False  # Allow for new trajectory generation

    def create_spline_path(self, start, mid, end):
        # Define the time points for the key positions: start, mid, and end
        t = np.array([0, 0.5, 1])

        # Interpolating 3D points (x, y, z) with cubic spline
        x_spline = CubicSpline(t, [start[0], mid[0], end[0]])
        y_spline = CubicSpline(t, [start[1], mid[1], end[1]])
        z_spline = CubicSpline(t, [start[2], mid[2], end[2]])

        # Generate 50 points along the spline for each axis
        t_fine = np.linspace(0, 1, num=50)
        path_x = x_spline(t_fine)
        path_y = y_spline(t_fine)
        path_z = z_spline(t_fine)

        # Stack the generated x, y, z points into a single path array
        path = np.vstack((path_x, path_y, path_z)).T
        return path

    def publish_path_to_rviz(self):
        if self.path is None:
            return  # If the path is not generated yet, do nothing

        path_msg = Path()
        path_msg.header = Header()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'  # Use 'map' or the relevant frame

        for point in self.path:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.position.z = point[2]
            path_msg.poses.append(pose)

        self.path_publisher.publish(path_msg)

    def finish_forward_movement(self):
        """Callback after 2 seconds to switch subscriptions and resume control."""
        # Stop the drone after 2 seconds of moving forward
        #self.stop_drone()

        # Destroy the current subscriber
        self.destroy_subscription(self.aruco_subscriber)
        self.get_logger().info(f"Unsubscribed from /aruco_poses")

        # Reset for the new marker detection and trajectory generation
        self.trajectory_generated = False
        self.initial_pose = None

        # Create a new subscriber to /aruco_poses2
        self.aruco_subscriber = self.create_subscription(PoseArray, '/aruco_poses2', self.aruco_callback, 10)
        self.get_logger().info(f"Subscribed to /aruco_poses2")

        # Reset the state
        self.is_moving_forward = False

        # Cancel the timer since it's no longer needed
        self.forward_move_timer.cancel()

    def stop_drone(self):
        """Stop the drone by setting all velocities to zero."""
        twist = Twist()
        twist.angular.z = 0.0
        twist.linear.x = 0.0
        self.cmd_vel_publisher.publish(twist)
        self.get_logger().info("Drone stopped.")


def main(args=None):
    rclpy.init(args=args)
    tello_pure_pursuit_node = TelloPurePursuitNode()
    rclpy.spin(tello_pure_pursuit_node)
    tello_pure_pursuit_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
