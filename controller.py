import math

"""
Controls Differential Drive Robot (2 Tank Tracks with axel) from Lego EV3
"""
class Controller:
    def __init__(self, wheel_base, wheel_radius, watch, left_motor, right_motor):
        self.wheel_base = wheel_base        # Distance between wheels (mm)
        self.wheel_radius = wheel_radius    # Radius of each wheel (mm)
        self.left_motor = left_motor        # Motor obj from Lego
        self.right_motor = right_motor      # Motor obj from Lego
        self.stopwatch = watch              # Stopwatch obj from Lego

        self.Kp = 25.0                      # Proportional gain for tighter control
        self.Kd = 10.0                      # Derivative gain for damping oscillations

        self.THRESHOLD_DIST = 1.0           # Distance(mm) Goal reached within Precision
        self.THRESHOLD_ORIENT = 2.0         # Angle(deg) Goal reached within Precision

        self.MAX_SPEED = 730                # Motor MAX speed approx 730 deg/s (121.67 RPM)
        self.SET_SPEED = self.MAX_SPEED / 2 # Default speed with room for increase/decrease
        self.MAX_DC = 100                   # MAX value for DC power input (%)
        self.SET_DC = self.MAX_DC / 2       # Default power (%)
        #self.SAMPLE_TIME = 100             # 1000ms per Sample (1 sec)

        self.prev_left = 0                  # previous left motor angle
        self.prev_right = 0                 # previous right motor angle

        self.dX = 0.0                       # delta world x_dot
        self.dY = 0.0                       # delta world y_dot
        self.dO = 0.0                       # delta world orient = local phi_dot
        self.position = [0.0, 0.0]          # world position [x, y]
        self.orientation = 0.0              # world orientation

    """
    We get the path in world frame as waypoints (target x, target y, target orient)
    """
    def follow_path(self, path):
        # loop through each waypoint in the path
        for target_x, target_y, target_orient in path:
            target_orient = self.normalize_radians(math.radians(target_orient))
            self.resets()
            print("Moving to: (x: {}mm, y: {}mm, ang: {}°)".format(target_x, target_y, math.degrees(target_orient)))

            # position control loop with goal
            print("Starting PD for position.")
            while not self.reached_target_pos(target_x, target_y):
                self.update_odometry()                          # Get bot position, orientation
                self.PDcontrol_position(target_x, target_y)     # Adjust bot position

            # orientation control loop with goal
            print("Starting PD for orientation.")
            while not self.reached_target_orient(target_orient):
                self.update_odometry()                          # Get bot position, orientation
                self.PDcontrol_orientation(target_orient)       # Adjust bot orientation

            print("Finished: (x: {}mm, y: {}mm, ang: {}°)".format(
                self.position[0], self.position[1], math.degrees(self.orientation)
                ))
        print("** Goal Reached **")

    def resets(self):
        self.dX = 0.0                                           # Reset globals
        self.dY = 0.0 
        self.dO = 0.0  
        #self.position = [0.0, 0.0]
        #self.orientation = 0.0

        self.left_motor.reset_angle(0)                          # Reset encoders
        self.right_motor.reset_angle(0)
        self.prev_left = 0
        self.prev_right = 0

        self.stopwatch.reset()                                  # Reset time interval
        self.left_motor.dc(self.SET_DC)                         # Reset initial motor speed
        self.right_motor.dc(self.SET_DC)
    
    """
    GOAL (each waypoint) reached if at target within THRESHOLD
    Parameters in World Frame
    """
    def reached_target_pos(self, target_x, target_y):
        distance = math.sqrt((target_x - self.position[0])**2 + (target_y - self.position[1])**2)
        return distance < self.THRESHOLD_DIST

    def reached_target_orient(self, target_orient):
        angle_diff = abs(self.normalize_radians(target_orient - self.orientation))
        return angle_diff < self.THRESHOLD_ORIENT

    """
    Normalized Angles: angles are in range [-180, 180] or [-PI, PI].
    This prevents angle wrapping.
    """
    def normalize_degrees(self, angle):
        # shift [-180,180] to [0,360] then modulo then shift back
        return (angle + 180) % 360 - 180

    def normalize_radians(self, angle):
        # shift by PI, modulo 2PI, shift back
        return (angle + math.pi) % (2 * math.pi) - math.pi

    """
    Scales motor speed range [-730, 730] to dc range [-100, 100]
    Also enforces pos/neg speed limits
    """
    def getDC(self, speed):
        # Ensure motor speed is within the allowable range
        speed = max(min(speed, self.MAX_SPEED), -self.MAX_SPEED)
        # Scale to dc
        return (speed / self.MAX_SPEED) * self.MAX_DC

    """
    https://www.youtube.com/watch?v=83r-Z9vMIiA
    """
    def PDcontrol_position(self, target_x, target_y):
        # angular error
        bot_to_target_orient = math.atan2(target_y - self.position[1], target_x - self.position[0])
        error = self.normalize_radians(bot_to_target_orient - self.orientation)

        # angular velocity error
        top1 = (self.position[0] - target_x) * self.dY
        top2 = (self.position[1] - target_y) * self.dX
        bot1 = (self.position[0] - target_x)**2
        bot2 = (self.position[1] - target_y)**2
        bot_to_target_orient_dot = (top1 - top2) / (bot1 + bot2)
        error_dot = bot_to_target_orient_dot - self.dO

        # PD Control law
        angular_v = self.Kp * error + self.Kd * error_dot

        # left: (v-w)/r
        # right: (v+w)/r
        left_speed = (self.SET_SPEED - angular_v * self.wheel_base / 2) / self.wheel_radius
        right_speed = (self.SET_SPEED + angular_v * self.wheel_base / 2) / self.wheel_radius

        # Scale to DC power (%)
        left_dc = self.getDC(left_speed)
        right_dc = self.getDC(right_speed)
        self.left_motor.dc(left_dc)
        self.right_motor.dc(right_dc)

    def PDcontrol_orientation(self, target_orient):
        # simplified because only adjusting orientation
        error = self.normalize_radians(target_orient - self.orientation)
        error_dot = -self.dO

        # PD Control law
        angular_v = self.Kp * error + self.Kd * error_dot

        # left: -w/r
        # right: w/r
        left_speed = -angular_v / self.wheel_radius
        right_speed = angular_v / self.wheel_radius

        # Scale to DC power (%)
        left_dc = self.getDC(left_speed)
        right_dc = self.getDC(right_speed)
        self.left_motor.dc(left_dc)
        self.right_motor.dc(right_dc)
    
    """
    Odometry is where the bot thinks it is in the world from its sensors (encoder).
    returns the bots estimated dX, dY, position and orientation in the world
    ref: lecture on Mobile Robot Kinematics powerpoint
    """
    def update_odometry(self):
        dt = self.stopwatch.time() / 1000.0                                 # get deltaTime in sec
        self.stopwatch.reset()                                              # Reset for next interval
        if dt == 0: return                                                  # Guard

        # OPTION1 = Get wheel ang v from angle over time interval
        angle_left = math.radians(self.left_motor.angle())
        angle_right = math.radians(self.right_motor.angle())
        wL = (angle_left - self.prev_left) / dt
        wR = (angle_right - self.prev_right) / dt
        self.prev_left = angle_left
        self.prev_right = angle_right
        
        # OPTION2 = Get wheel angular velocities (deg/s) to rad/s
        wL2 = math.radians(self.left_motor.speed())
        wR2 = math.radians(self.right_motor.speed())

        print("speed(L/R): {:.2f}/{:.2f} \t\tspeedAngle: {:.2f}/{:.2f} \t\t\tdt: {:.4f} \t\t\tang(L/R): {:.2f}/{:.2f}".format(
            wL2,wR2,wL,wR,dt,angle_left,angle_right
            ))

        v = (self.wheel_radius / 2) * (wR + wL)                             # Linear velocity (mm/s)
        w = (self.wheel_radius / self.wheel_base) * (wR - wL)               # Angular velocity (rad/s)
        
        self.dO = w * dt                                                    # delta orientation (rad)

        if w == 0:                                                          # delta position-straight line
            self.dX = v * dt * math.cos(self.orientation)
            self.dY = v * dt * math.sin(self.orientation)
        else:                                                               # delta position-arc (rotation)
            self.dX = v * dt * math.cos(self.orientation + (self.dO / 2))
            self.dY = v * dt * math.sin(self.orientation + (self.dO / 2))
        
        self.position[0] += self.dX
        self.position[1] += self.dY
        self.orientation += self.dO

        self.orientation = self.normalize_radians(self.orientation)         # normalize to [-PI, PI]

        print("v(x,y): {:.2f},{:.2f} \t\t\torient(O): {:.2f} \t\t\t\tpos(x,y): {:.2f},{:.2f}\n".format(
            self.dX,self.dY,self.orientation,self.position[0],self.position[1]
            ))
