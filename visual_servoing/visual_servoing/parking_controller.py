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

        self.parking_distance = 1.5 # meters; try playing with this number!
        self.relative_x = 0
        self.relative_y = 0

        self.previous_distance = None
        self.previous_angle = None
        self.current_distance_from_cone = None
        self.current_angle = None

        self.drive_mode = "forward"

        self.get_logger().info("Parking Controller Initialized")

    def relative_cone_callback(self, msg):
        self.relative_x = msg.x_pos
        self.relative_y = msg.y_pos
        drive_cmd = AckermannDriveStamped()

        #################################
        steering_angle = 0.0
        velocity = 0.0

        current_angle = np.arctan(self.relative_y/self.relative_x)
        self.current_angle = current_angle
        # self.get_logger().info('Angle: ' + str(current_angle))
        current_distance_from_cone = np.sqrt((self.relative_x)**2+(self.relative_x)**2)
        self.current_distance_from_cone = current_distance_from_cone
        current_distance_error = current_distance_from_cone - self.parking_distance
        flip_controls = -1 if self.relative_x < 0 else 1

        K_p = 1
        K_i = 0
        K_d = 0

        # YOUR CODE HERE
        # Use relative position and your control law to set drive_cmd
        # also probably need to check self.relative_y
        
        if self.relative_x < 0 or current_distance_error < 0:
            self.get_logger().info('A')
            self.drive_mode = "backing"

        if self.drive_mode == "backing":
            if self.relative_x < self.parking_distance:
                self.get_logger().info('B')
                velocity = min(-0.5, -abs(current_distance_from_cone))
            else:
                self.get_logger().info('C')
                self.drive_mode = "forward"
        
        if self.drive_mode == "forward":
            self.get_logger().info('D')
            steering_angle = flip_controls * K_p * (current_angle)
            steering_angle = min(0.34, steering_angle) if steering_angle > 0 else max(-0.34, steering_angle)
            velocity = flip_controls * (0.5 * abs(steering_angle) + current_distance_error * 0.25)


        # if current_distance_from_cone < 1.0:
        #     distance_error = self.relative_x - self.parking_distance
        #     if abs(current_angle) > 0.0005:
        #         if distance_error > 0 and distance_error < 0.0005:
        #             velocity = 0
        #         else:
        #             velocity = 0.25 * distance_error
        #     else:
        #         velocity = -0.5 * distance_error
        # elif abs(current_angle) < 0.0005: # car is lined up
        #     distance_error = self.relative_x - self.parking_distance
        #     if abs(distance_error) > 0.005:
        #         velocity = 0.5 * distance_error
        #     else:
        #         velocity = 0.0
        # else:
        #     steering_angle = flip_controls * K_p * (current_angle)
        #     steering_angle = min(0.34, steering_angle) if steering_angle > 0 else max(-0.34, steering_angle)

        #     velocity = flip_controls * (1.0 * abs(steering_angle) + current_distance_from_cone * 0.25)
        





        #################################
        drive_cmd.drive.steering_angle = min(0.34, steering_angle) if steering_angle > 0 else max(-0.34, steering_angle)
        drive_cmd.drive.speed = min(1.0, velocity) if velocity < 0 else max(-1.0, velocity)
        self.drive_pub.publish(drive_cmd)
        self.error_publisher()

    def error_publisher(self):
        """
        Publish the error between the car and the cone. We will view this
        with rqt_plot to plot the success of the controller
        """
        error_msg = ParkingError()

        #################################

        # YOUR CODE HERE
        # Populate error_msg with relative_x, relative_y, sqrt(x^2+y^2)
        error_msg.x_error = (self.current_distance_from_cone - self.parking_distance) * np.cos(self.current_angle)
        error_msg.y_error = (self.current_distance_from_cone - self.parking_distance) * np.sin(self.current_angle)
        # error_msg.x_error = self.relative_x
        # error_msg.y_error = self.relative_y
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