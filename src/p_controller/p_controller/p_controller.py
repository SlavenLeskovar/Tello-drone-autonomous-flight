#!/usr/bin/env python3

#import tf2_ros
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Twist
from sensor_msgs.msg import Joy


class PController(Node):
    def __init__(self):
        super().__init__('p_controller')
        
        # Set the target position
        self.target_position = [0.0, 0.1, 1.3] 
        self.target_yaw = 0.0

        # P controller gains
        self.Kpx = 0.3 
        self.Kpy = 0.4 
        self.Kpz = 0.55 
        self.Kpyaw = 0.9 

        # I controller gains
        self.Kix = 0.0005 
        self.Kiy = 0.0005  
        self.Kiz = 0.0010 
        self.Kiyaw = 0.001
        self.integral_error_x = 0
        self.integral_error_y = 0
        self.integral_error_z = 0
        self.integral_error_yaw = 0

        # D controller gains
        self.Kdx = 0.8 
        self.Kdy = 0.2 
        self.Kdz = 0.7
        self.prev_error_x = 0
        self.prev_error_y = 0
        self.prev_error_z = 0
        self.prev_error_yaw = 0

        self.prev_time = self.get_clock().now().nanoseconds / 1e9

        self.velocity_z = 0
        self.velocity_x = 0
        self.velocity_y = 0
        self.velocity_yaw = 0

        # PS4 buttons are off
        self.state = True
            
        # Subscriber for getting PS4 joy commands
        self.joystick_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 100)

        # Subscriber for getting current position
        self.pose_sub = self.create_subscription(PoseArray, '/aruco_poses', self.pose_callback, 100)
        

        # Publisher for sending control commands
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 100)


    def joy_callback(self, msg:Joy):
        if msg.buttons[0] == 1:  # Activate with "X"
            self.state = True
        if msg.buttons[1] == 1:  # Deactivate with "O"
            self.state = False
        
    def pose_callback(self, msg: PoseArray):          
        if self.state == True:

            # Getting current position frome aruco node
            current_position = [msg.poses[0].position.x, msg.poses[0].position.y, msg.poses[0].position.z]
            quaternion_angles = [msg.poses[0].orientation.x, msg.poses[0].orientation.y, msg.poses[0].orientation.z, msg.poses[0].orientation.w]
            
            # Convert Quaternion to Euler
            euler_angles = self.quaternion_to_euler(quaternion_angles)
            yaw_angle = euler_angles[2]

            # Calculate time
            current_time = self.get_clock().now().nanoseconds / 1e9  # Get current time
            dt = current_time - self.prev_time  # Calculate time difference
            self.prev_time = current_time  # Update prev_time for next iteration
                
            # Calculate errors
            error_x = self.target_position[0] - current_position[0]
            error_y = self.target_position[1] - current_position[1]
            error_z = -self.target_position[2] + current_position[2]
            error_yaw = self.target_yaw - yaw_angle

            self.integral_error_x += error_x*dt
            self.integral_error_y += error_y*dt
            self.integral_error_z += error_z*dt
            self.integral_error_yaw += error_yaw*dt

            self.derivative_x = (error_x - self.prev_error_x)/dt
            self.prev_error_x = error_x
            self.derivative_y = (error_y - self.prev_error_y)/dt
            self.prev_error_y = error_y
            self.derivative_z = (error_z - self.prev_error_z)/dt
            self.prev_error_z = error_z
            
            # P controller
            #cmd_vel_x = self.Kpx * error_z *0
            #cmd_vel_y = self.Kpy * error_x*0
            #cmd_vel_z = self.Kpz * error_y
            #cmd_vel_yaw = self.Kpyaw * error_yaw*0 

            
            cmd_vel_x = self.Kpx * error_z #+ self.Kdx * self.derivative_z + self.Kix * self.integral_error_z 
            cmd_vel_y = self.Kpy * error_x #+ self.Kiy * self.integral_error_x + self.Kdy * self.derivative_x
            cmd_vel_z = self.Kpz * error_y #+ self.Kiz * self.integral_error_y + self.Kdz*self.derivative_z
            cmd_vel_yaw = self.Kpyaw * error_yaw #+ self.Kiyaw * self.integral_error_yaw
            
            #cmd_vel_x = self.Kpx * 0
            #cmd_vel_y = self.Kpy * 0
            #cmd_vel_z = self.Kpz * 0
            #cmd_vel_yaw = self.Kpyaw * 0

            if cmd_vel_x > 0.6:
                cmd_vel_x = 0.6
            if cmd_vel_x < -0.6:
                cmd_vel_x = -0.6
            if cmd_vel_y > 0.6:
                cmd_vel_y = 0.6
            if cmd_vel_y < -0.6:
                cmd_vel_y = -0.6            
            if cmd_vel_z > 0.6:
                cmd_vel_z = 0.6    
            if cmd_vel_z < -0.6:
                cmd_vel_z = -0.6


            # Publish control commands
            self.publish_command(cmd_vel_x, cmd_vel_y, cmd_vel_z, cmd_vel_yaw)
                    
        else:
            cmd_vel_x = 0.0
            cmd_vel_y = 0.0
            cmd_vel_z = 0.0
            cmd_vel_yaw = 0.0
            self.publish_command(cmd_vel_x, cmd_vel_y, cmd_vel_z, cmd_vel_yaw)
        

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
               

    def publish_command(self, cmd_vel_x, cmd_vel_y, cmd_vel_z, cmd_vel_yaw):
        cmd_msg = Twist()
        cmd_msg.linear.x = cmd_vel_x
        cmd_msg.linear.y = cmd_vel_y
        cmd_msg.linear.z = cmd_vel_z
        cmd_msg.angular.x = 0.0
        cmd_msg.angular.y = 0.0
        cmd_msg.angular.z = cmd_vel_yaw
        self.cmd_pub.publish(cmd_msg)

def main(args=None):
    rclpy.init(args=args)
    controller = PController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()