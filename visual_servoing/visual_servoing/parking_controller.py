#!/usr/bin/env python

import rclpy
from rclpy.node import Node
import numpy as np

from vs_msgs.msg import ConeLocation, ParkingError
from ackermann_msgs.msg import AckermannDriveStamped

class ParkingController(Node):
    """
    A controller for parking in front of a cone.
    Listens for a relative cone location and publishes control commands.
    Can be used in the simulator and on the real robot.
    """
    def __init__(self):
        super().__init__("parking_controller")

        self.declare_parameter("drive_topic")
        DRIVE_TOPIC = self.get_parameter("drive_topic").value # set in launch file; different for simulator vs racecar

        self.drive_pub = self.create_publisher(AckermannDriveStamped, DRIVE_TOPIC, 10)
        self.error_pub = self.create_publisher(ParkingError, "/parking_error", 10)

        self.create_subscription(ConeLocation, "/relative_cone", 
            self.relative_cone_callback, 1)
        
        self.TURN_RADIUS = 1  # meters; this is actually measured/calculated

        self.PARKING_DISTANCE = 0.75 # meters; try playing with this number!
        self.relative_x = 0
        self.relative_y = 0

        self.previous_distance_error = 0
        self.previous_angle = 0
        self.distance_error = 0
        self.current_angle = None

        self.drive_mode = "forward"

        self.get_logger().info("Parking Controller Initialized")

    def bound_variable(self, value, min_value, max_value):
        if value < min_value:
            return min_value
        elif value > max_value:
            return max_value
        else:
            return value

    def relative_cone_callback(self, msg):
        self.relative_x = msg.x_pos
        self.relative_y = msg.y_pos
        drive_cmd = AckermannDriveStamped()

        #################################
        steering_angle = 0.0
        velocity = 0.0
        angle_buffer = 0.15
        distance_buffer = 0.01

        current_angle = np.arctan2(self.relative_y, self.relative_x)
        self.current_angle = current_angle
        # self.get_logger().info('Angle: ' + str(current_angle))
        current_distance_from_cone = np.sqrt((self.relative_x)**2+(self.relative_y)**2)
        self.get_logger().info('Distance: ' + str(current_distance_from_cone))
        self.current_distance_from_cone = current_distance_from_cone
        current_distance_error = current_distance_from_cone - self.PARKING_DISTANCE


        x_error = (current_distance_from_cone - self.PARKING_DISTANCE) * np.cos(self.current_angle)
        y_error = (current_distance_from_cone - self.PARKING_DISTANCE) * np.sin(self.current_angle)
        distance_error = np.sqrt(x_error** 2 + y_error ** 2)
        self.distance_error = distance_error

        K_p_steer = 1
        K_i_steer = 0
        K_d_steer = 0

        # K_p_drive = 1.5
        # K_i_drive = 0
        # K_d_drive = 0.3

        K_p_drive = 1.5
        K_i_drive = 0
        K_d_drive = 0.05


        # LINE FOLLOWING
        # K_p_drive = 2
        # K_p_steer = 0.5
        # K_d_steer = 0.1

        # YOUR CODE HERE
        # Use relative position and your control law to set drive_cmd
        # also probably need to check self.relative_y
        
        # if the cone is anywhere behind the car, or if the car is too close to the cone, switch to "backing" mode
        if (self.relative_x < 0 and current_distance_error > 0) or (current_distance_error < 0 and current_angle > -1.57 and current_angle < 1.57):
            self.drive_mode = "backing"
            flip_controls = -1
        else:
            self.drive_mode = "forward"
            flip_controls = 1

        self.get_logger().info('Current Mode: ' + str(self.drive_mode))

        # stop condition: car is facing the cone and is at the desired distance away, plus or minus a buffer
        if abs(current_angle) < angle_buffer and abs(current_distance_error) < distance_buffer: # buffer 
                velocity = 0.0

        # car drives backwards (velocity < 0) in "backing" mode
        elif self.drive_mode == "backing":
            #                                           car is close to the cone, but is not facing the cone at the right angle
            # if (self.relative_x < self.TURN_RADIUS) or (abs(current_angle) > (angle_buffer+0.05) and current_distance_error < (distance_buffer+0.05)):
            steering_angle = flip_controls * K_p_steer * (current_angle)
            steering_angle = self.bound_variable(steering_angle, -0.34, 0.34)
            velocity = 1.5 * (-K_p_drive*abs(current_distance_error) - K_d_drive*abs(current_distance_error - self.previous_distance_error)) - 0.4
            
        elif self.drive_mode == "forward":
            # steering according to PID control based on the angle, velocity based on the distance error and steering angle
            steering_angle = flip_controls * K_p_steer * (current_angle)
            steering_angle = self.bound_variable(steering_angle, -0.34, 0.34)
            velocity = K_p_drive*abs(current_distance_error**1.5) + K_d_drive*abs(current_distance_error - self.previous_distance_error) + 0.4

        ################################# 
        drive_cmd.drive.steering_angle = min(0.34, steering_angle) if steering_angle > 0 else max(-0.34, steering_angle)
        max_speed = 1.0
        drive_cmd.drive.speed = min(max_speed, velocity) if velocity > 0 else max(-max_speed, velocity)
        self.get_logger().info('Angle: ' + str(drive_cmd.drive.steering_angle))
        self.get_logger().info('Speed: ' + str(drive_cmd.drive.speed)) 
        self.drive_pub.publish(drive_cmd)
        self.error_publisher()

        self.previous_distance_error = current_distance_error
        self.previous_angle = current_angle


    def error_publisher(self):
        """
        Publish the error between the car and the cone. We will view this
        with rqt_plot to plot the success of the controller
        """
        error_msg = ParkingError()

        #################################

        # YOUR CODE HERE
        # Populate error_msg with relative_x, relative_y, sqrt(x^2+y^2)
        error_msg.x_error = (self.current_distance_from_cone - self.PARKING_DISTANCE) * np.cos(self.current_angle)
        error_msg.y_error = (self.current_distance_from_cone - self.PARKING_DISTANCE) * np.sin(self.current_angle)
        error_msg.distance_error = np.sqrt(error_msg.x_error** 2 + error_msg.y_error ** 2)
        #################################
        
        self.error_pub.publish(error_msg)

def main(args=None):
    rclpy.init(args=args)
    pc = ParkingController()
    rclpy.spin(pc)
    rclpy.shutdown()

if __name__ == '__main__':
    main()