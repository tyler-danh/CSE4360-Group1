import math

"""
Controls Differential Drive Robot (2 Tank Tracks with axel) from Lego EV3
"""
class Controller:
    def __init__(self, wheel_base, wheel_radius, watch, watch2, wait, left_motor, right_motor):
        self.wheel_base = wheel_base        # Distance between wheels (mm)
        self.wheel_radius = wheel_radius    # Radius of each wheel (mm)
        self.left_motor = left_motor        # Motor obj from Lego
        self.right_motor = right_motor      # Motor obj from Lego
        self.stopwatch = watch              # Stopwatch obj from Lego
        self.stopwatch2 = watch2            # Stopwatch obj from Lego
        self.wait = wait                    # wait function from Lego

        self.Kp = 400.0                     # Proportional gain for tighter control
        self.Kd = 40.0                      # Derivative gain for damping oscillations
        #self.Kp = 250.0                    # Proportional gain for tighter control
        #self.Kd = 31.62277                 # Derivative gain for damping oscillations
        #self.Kp = 100.0                    # Proportional gain for tighter control
        #self.Kd = 20.0                     # Derivative gain for damping oscillations
        #self.Kp = 25.0                     # Proportional gain for tighter control
        #self.Kd = 10.0                     # Derivative gain for damping oscillations

        self.THRESHOLD_DIST = 10.0          # Distance(mm) Goal reached within Precision
        self.THRESHOLD_ORIENT = 1.0         # Angle(deg) Goal reached within Precision

        self.MAX_SPEED = 730                # Motor MAX speed approx 730 deg/s (121.67 RPM)
        self.SET_SPEED = self.MAX_SPEED / 2 # Default speed with room for increase/decrease
        self.MAX_DC = 100                   # MAX value for DC power input (%) [-100,100]
        self.SET_DC = self.MAX_DC / 4       # Default power (%). start slow
        self.DEADZONE = 17                  # Apply MIN DC to range [-100,-20], [20,100]
        self.SLOWDOWN_DIST = 200            # Distance to Goal (mm) to begin slowing
        self.RAMP_DURATION = 1000           # at start of waypoint, ramp up speed over 1000ms
        self.SPEED_FACTOR = 0.2             # initial speed factor (20% of full speed)
        # scale min dc power to min speed
        self.MIN_SPEED = (self.DEADZONE / self.MAX_DC) * self.MAX_SPEED

        self.prev_left = 0                  # previous left motor angle
        self.prev_right = 0                 # previous right motor angle
        self.prev_error = 0                 # previous angle error

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
            print("Moving to: (x: {:.2f}mm, y: {:.2f}mm, ang: {:.2f}°)".format(
                target_x, target_y, math.degrees(target_orient)
                ))

            # position control loop with goal
            print("Starting PD for position.")
            while not self.reached_target_pos(target_x, target_y):
                self.update_odometry()                          # Get bot position, orientation
                self.PDcontrol_position(target_x, target_y)     # Adjust bot position
            
            #print("Coasting...")
            #self.wait(1000)                                    # wait 1 sec to allow settling

            # orientation control loop with goal
            print("Starting PD for orientation.")
            while not self.reached_target_orient(target_orient):
                self.update_odometry()                          # Get bot position, orientation
                self.PDcontrol_orientation(target_orient)       # Adjust bot orientation

            print("Braking...")
            self.left_motor.hold()
            self.right_motor.hold()
            #self.wait(500)                                     # wait 1.5 sec to allow settling

            print("Finished: (x: {:.2f}mm, y: {:.2f}mm, ang: {:.2f}°)".format(
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
        self.prev_error = 0

        self.stopwatch.reset()                                  # Reset time interval
        self.stopwatch2.reset()
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
        return angle_diff < math.radians(self.THRESHOLD_ORIENT)

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
    Also allows optional deadzone around zero [-100,-20], [20,100]
    """
    def getDC(self, speed, deadzone=0):
        # Ensure motor speed is within the allowable range
        speed = max(min(speed, self.MAX_SPEED), -self.MAX_SPEED)
        # Scale to DC %
        scaled = (speed / self.MAX_SPEED) * self.MAX_DC
        # optional deadzone
        if abs(scaled) < deadzone:
            # Scale deadzone up to min effective power
            return 0 if scaled == 0 else deadzone * (1 if scaled > 0 else -1)
        elif scaled >= deadzone:
            return max(scaled, deadzone)
        else:
            return min(scaled, -deadzone)

    """
    https://www.youtube.com/watch?v=83r-Z9vMIiA
    """
    def PDcontrol_position(self, target_x, target_y):
        # Calculate position error
        dy = target_y - self.position[1]
        dx = target_x - self.position[0]
        dist_goal = math.sqrt(dx**2 + dy**2)
        
        # angular error - atan2
        err_orient = math.atan2(dy, dx)
        error = self.normalize_radians(err_orient - self.orientation)

        # angular velocity error - derivative of atan2
        top1 = (self.position[0] - target_x) * self.dY
        top2 = (self.position[1] - target_y) * self.dX
        bot1 = (self.position[0] - target_x)**2
        bot2 = (self.position[1] - target_y)**2
        err_orient_dot = (top1 - top2) / (bot1 + bot2)
        error_dot = err_orient_dot - self.dO

        #error_dot = error - self.prev_error
        #self.prev_error = error

        # PD Control law
        angular_v = self.Kp * error + self.Kd * error_dot

        # left: (v-w) / r
        # right: (v+w) / r
        left_speed = self.SET_SPEED - angular_v
        right_speed = self.SET_SPEED + angular_v

        # Apply initial speed ramp at start of waypoint
        dt2 = self.stopwatch2.time()
        if dt2 < self.RAMP_DURATION:
            ramp_factor = self.SPEED_FACTOR + (1 - self.SPEED_FACTOR) * (dt2 / self.RAMP_DURATION)
            left_speed *= ramp_factor
            right_speed *= ramp_factor

        # Apply slowdown factor if close to goal
        if dist_goal < self.SLOWDOWN_DIST:
            slowdown_factor = dist_goal / self.SLOWDOWN_DIST
            slowdown_factor = max(slowdown_factor, self.MIN_SPEED / self.MAX_SPEED)
            left_speed *= slowdown_factor
            right_speed *= slowdown_factor

        # Scale to DC power (%)
        left_dc = self.getDC(left_speed, self.DEADZONE)
        right_dc = self.getDC(right_speed, self.DEADZONE)

        self.left_motor.dc(left_dc)
        self.right_motor.dc(right_dc)

        #print("ang_v: {:.2f}, Spd(l/r): {:.2f}/{:.2f}, DC(l/r): {:.2f}/{:.2f}".format(
        #    angular_v, left_speed, right_speed, left_dc, right_dc
        #    ))

    def PDcontrol_orientation(self, target_orient):
        # simplified because only adjusting orientation
        error = self.normalize_radians(target_orient - self.orientation)
        error_dot = -self.dO

        # PD Control law
        angular_v = self.Kp * error + self.Kd * error_dot

        # left: -w / r
        # right: w / r
        left_speed = -angular_v / self.wheel_radius
        right_speed = angular_v / self.wheel_radius

        # Scale to DC power (%)
        left_dc = self.getDC(left_speed, self.DEADZONE)
        right_dc = self.getDC(right_speed, self.DEADZONE)
        self.left_motor.dc(left_dc)
        self.right_motor.dc(right_dc)
        #print("ang_v: {:.2f}, Spd(l/r): {:.2f}/{:.2f}, DC(l/r): {:.2f}/{:.2f}".format(
        #       angular_v, left_speed,right_speed,left_dc,right_dc
        #      ))
    
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
        # likely less accurate than getting angle from encoder
        # wL2 = math.radians(self.left_motor.speed())
        # wR2 = math.radians(self.right_motor.speed())

        #print("speed(L/R): {:.2f}/{:.2f} \t\tspeedAngle: {:.2f}/{:.2f} \t\t\tdt: {:.4f} \t\t\tang(L/R): {:.2f}/{:.2f}".format(
        #    wL2,wR2,wL,wR,dt,angle_left,angle_right
        #    ))

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

        #print("v(x,y): {:.2f},{:.2f} \t\t\torient(O): {:.2f}° \t\t\t\tpos(x,y): {:.2f},{:.2f}\n".format(
        #    self.dX,self.dY,math.degrees(self.orientation),self.position[0],self.position[1]
        #    ))
