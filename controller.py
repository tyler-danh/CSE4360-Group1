import math

class Controller:
    def __init__(self, wheel_base, wheel_radius, watch, left_motor, right_motor):
        self.wheel_base = wheel_base        # Distance between wheels
        self.wheel_radius = wheel_radius    # Radius of each wheel
        self.left_motor = left_motor        # Motor obj from Lego
        self.right_motor = right_motor      # Motor obj from Lego
        self.stopwatch = watch              # Stopwatch obj from Lego

        self.Kp = 200.0                      # Proportional gain for angular velocity
        self.Kd = 28.28427                   # Derivative gain for angular velocity

        self.THRESHOLD_DIST = 50.0          # Distance(mm) Goal reached within Precision
        self.THRESHOLD_ORIENT = 1.0         # Angle(deg) Goal reached within Precision
        self.MAX_SPEED = 900                # Motor MAX speed approx 900 deg/s (150 RPM)
        self.SET_SPEED = self.MAX_SPEED/2   # Default speed with room for increase/decrease

        self.x_dot = 0.0                    # world x_dot
        self.y_dot = 0.0                    # world y_dot
        self.omega_dot = 0.0                # world omega_dot = local omega_dot
        self.position = [0.0, 0.0]          # world position [x, y]
        self.orientation = 0.0              # world orientation

    """
    We get the path in world frame as waypoints (target x, target y, target orientation)

    """
    def follow_path(self, path):
        # loop through each waypoint in the path
        for target_x, target_y, target_orientation in path:
            self.resets()
            print("Moving to: (x: {0}mm, y: {1}mm, ang: {2}°)".format(target_x, target_y, target_orientation))

            # position control loop with goal
            print("Starting PD for position.")
            while not self.reached_target_pos(target_x, target_y):
                self.update_odometry()                          # Get bot position, orientation
                self.PDcontrol_position(target_x, target_y)     # Adjust bot position

            # orientation control loop with goal
            print("Starting PD for orientation.")
            while not self.reached_target_orient(target_orientation):
                self.update_odometry()                          # Get bot position, orientation
                self.PDcontrol_orientation(target_orientation)  # Adjust bot orientation

            print("Waypoint complete.")
        print("** Goal Reached **")

    def resets(self):
        # Reset encoder
        self.left_motor.reset_angle(0)
        self.right_motor.reset_angle(0)
        # Reset globals
        self.x_dot = 0.0 
        self.y_dot = 0.0 
        self.omega_dot = 0.0  
        self.position = [0.0, 0.0]
        self.orientation = 0.0
        # Reset time interval
        self.stopwatch.reset()
        # Reset initial motor speed
        self.left_motor.run(self.SET_SPEED)
        self.right_motor.run(self.SET_SPEED)
    
    
    """
    GOAL (each waypoint) reached if at target within THRESHOLD
    Parameters in World Frame
    """
    def reached_target_pos(self, target_x, target_y):
        distance = math.sqrt((target_x - self.position[0])**2 + (target_y - self.position[1])**2)
        return distance < self.THRESHOLD_DIST

    def reached_target_orient(self, target_orientation):
        angle_diff = abs(self.normalize_angle(target_orientation - self.orientation))
        return angle_diff < self.THRESHOLD_ORIENT

    # Normalized Angles: angles are in range [-180, 180]. This prevents angle wrapping.
    def normalize_angle(self, angle):
        return (angle + 180) % 360 - 180

    
    """
    https://www.youtube.com/watch?v=83r-Z9vMIiA
    """
    def PDcontrol_position(self, target_x, target_y):
        
        # angular error
        bot_to_target_orient = math.degrees(math.atan2(target_y - self.position[1], target_x - self.position[0]))
        error = self.normalize_angle(bot_to_target_orient - self.orientation)

        # angular velocity error
        top1 = (self.position[0] - target_x) * self.y_dot
        top2 = (self.position[1] - target_y) * self.x_dot
        bot1 = (self.position[0] - target_x)**2
        bot2 = (self.position[1] - target_y)**2
        bot_to_target_orient_dot = (top1 - top2) / (bot1 + bot2)
        error_dot = bot_to_target_orient_dot - self.omega_dot

        # PD Control law
        angular_v = self.Kp * error + self.Kd * error_dot

        # left: (v-w)/r
        # right: (v+w)/r
        left_speed = (self.SET_SPEED - angular_v) / self.wheel_radius
        right_speed = (self.SET_SPEED + angular_v) / self.wheel_radius
        # Apply speed limits
        left_speed = max(min(left_speed, self.MAX_SPEED), -self.MAX_SPEED)
        right_speed = max(min(right_speed, self.MAX_SPEED), -self.MAX_SPEED)
        
        self.left_motor.run(left_speed)
        self.right_motor.run(right_speed)

    def PDcontrol_orientation(self, target_orientation):
        error = self.normalize_angle(target_orientation - self.orientation)
        error_dot = -1 * self.omega_dot

        # PD Control law
        angular_v = self.Kp * error + self.Kd * error_dot

        # left: -w/r
        # right: w/r
        left_speed = (-1 * angular_v) / self.wheel_radius
        right_speed = angular_v / self.wheel_radius
        # Apply speed limits
        left_speed = max(min(left_speed, self.MAX_SPEED), -self.MAX_SPEED)
        right_speed = max(min(right_speed, self.MAX_SPEED), -self.MAX_SPEED)

        self.left_motor.run(left_speed)
        self.right_motor.run(right_speed)
    

    """
    Odometry is where the bot thinks it is in the world from its sensors (encoder).
    returns the bots estimated omega_dot, x_dot, y_dot, position and orientation in the world
    ref: lecture on Mobile Robot Kinematics powerpoint
    """
    def update_odometry(self):
        dt = self.stopwatch.time() / 1000.0 # get deltaTime in sec
        self.stopwatch.reset()              # Reset for next interval

        if dt == 0:
            return

        # Get wheel angular velocities (deg/s)
        omega_dot_L = self.left_motor.speed()
        omega_dot_R = self.right_motor.speed()

        # Calculate bot linear and angular velocities
        v_x = self.wheel_radius * (omega_dot_R + omega_dot_L) / 2
        self.omega_dot = self.wheel_radius * (omega_dot_R - omega_dot_L) / self.wheel_base

        # Update world orientation
        self.orientation += self.omega_dot * dt
        self.orientation = self.normalize_angle(self.orientation)

        # Calculate velocity in world frame
        self.x_dot = v_x * math.cos(math.radians(self.orientation))
        self.y_dot = v_x * math.sin(math.radians(self.orientation))

        # Update world position
        self.position[0] += self.x_dot * dt
        self.position[1] += self.y_dot * dt
