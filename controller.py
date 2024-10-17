from pybricks.parameters import Port, Stop, Direction, Button, Color

import math

class Controller:
    def __init__(self, wheel_base, wheel_radius, left_motor, right_motor):
        self.wheel_base = wheel_base        # Distance between wheels
        self.wheel_radius = wheel_radius    # Radius of each wheel
        self.left_motor = left_motor        # Motor obj from Lego
        self.right_motor = right_motor      # Motor obj from Lego
        self.Kp_linear = 0.5                # Proportional gain for linear velocity
        self.Kp_angular = 2.0               # Proportional gain for angular velocity
        self.threshold = 0.1                # Precision of distance/orientation calculations
        self.segment_time = 1000            # time(ms) to run/block before error checks

    def follow_path(self, path):
        # loop through each waypoint
        for target_x, target_y, target_orientation in path:
            current_x, current_y, current_orientation = self.get_current_pose()
            
            while not self.reached_target(current_x, current_y, current_orientation, 
                                          target_x, target_y, target_orientation):
                # Calculate required linear and angular velocities
                linear_velocity, angular_velocity = self.calculate_velocities(
                    current_x, current_y, current_orientation, 
                    target_x, target_y, target_orientation
                )
                
                # Convert to wheel speeds
                left_speed, right_speed = self.convert_to_wheel_speeds(linear_velocity, angular_velocity)
                
                # Set motor speeds
                self.left_motor.run_time(left_speed, self.segment_time, then=Stop.HOLD, wait=False)
                self.right_motor.run_time(right_speed, self.segment_time, then=Stop.HOLD, wait=True)
                
                # Update current position
                current_x, current_y, current_orientation = self.get_current_pose()

    def get_current_pose(self):
        # position based on motor encoders
        left_distance = self.left_motor.angle() * self.wheel_radius
        right_distance = self.right_motor.angle() * self.wheel_radius
        
        distance = (left_distance + right_distance) / 2
        orientation = (right_distance - left_distance) / self.wheel_base
        
        x = distance * math.cos(orientation)
        y = distance * math.sin(orientation)
        return x, y, orientation

    def reached_target(self, current_x, current_y, current_orientation, 
                       target_x, target_y, target_orientation):
        # goal reached to precision of target - current
        distance = math.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)
        angle_diff = abs(target_orientation - current_orientation)
        return distance < self.threshold and angle_diff < self.threshold

    def calculate_velocities(self, current_x, current_y, current_orientation, 
                             target_x, target_y, target_orientation):
        # Simple proportional control
        distance = math.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)
        angle = math.atan2(target_y - current_y, target_x - current_x)
        
        linear_velocity = distance * self.Kp_linear
        angular_velocity = (angle - current_orientation) * self.Kp_angular
        return linear_velocity, angular_velocity

    def convert_to_wheel_speeds(self, linear_velocity, angular_velocity):
        left_speed = (linear_velocity - angular_velocity * self.wheel_base / 2) / self.wheel_radius
        right_speed = (linear_velocity + angular_velocity * self.wheel_base / 2) / self.wheel_radius
        return left_speed, right_speed