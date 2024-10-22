import math
from pybricks.parameters import Stop

"""
Controls Differential Drive Robot (tri-cycle) from Lego EV3
"""
class Controller:
    def __init__(self, wheel_base, wheel_radius, watch, left_motor, right_motor):
        self.wheel_base = wheel_base        # Distance between wheels (mm)
        self.wheel_radius = wheel_radius    # Radius of each wheel (mm)
        self.left_motor = left_motor        # Motor obj from Lego
        self.right_motor = right_motor      # Motor obj from Lego
        self.stopwatch = watch              # Stopwatch obj from Lego

        self.THRESHOLD_DIST = 10.0          # Distance(mm) Goal reached within Precision
        self.THRESHOLD_ORIENT = 1.0         # Angle(deg) Goal reached within Precision
        self.OFFSET = 10.0                  # Calibration offset for over/under shooting turns

        self.MAX_SPEED = 730                # Motor MAX speed approx 730 deg/s (121.67 RPM)
        self.SET_SPEED = self.MAX_SPEED / 4 # Default speed
        self.MIN_SPEED = 124.1

        self.prev_left = 0.0                # previous left motor angle (rad)
        self.prev_right = 0.0               # previous right motor angle (rad)

        self.dX = 0.0                       # delta x_dot (mm)
        self.dO = 0.0                       # delta world orient = local phi_dot (rad)
        self.current_dist = 0.0             # distance travelled in line (mm)
        self.orientation = 0.0              # world orientation (rad)

    """
    We get the path in world frame as waypoints (target distance, target orientation)
    """
    def follow_path(self, path):
        # loop through each waypoint in the path
        print("\nStarting Controller...")
        for target_distance, target_orient in path:
            target_orient = self.normalize_radians(math.radians(target_orient))
            if(target_orient > 0):
                target_orient -= math.radians(self.OFFSET)
            else:
                target_orient += math.radians(self.OFFSET)
                
            self.resets()
            print("Move: (distance: {:.2f}mm, ang: {:.2f}°)".format(target_distance, math.degrees(target_orient)))

            # position control loop with goal
            print("Starting PD for position.")
            while not self.reached_target_pos(target_distance):
                self.update_odometry()                          # Get bot changes
                distance_degrees = self.calculate_distance_angle(target_distance - self.current_dist)
                self.left_motor.run_angle(self.SET_SPEED, distance_degrees, then=Stop.HOLD, wait=False)
                self.right_motor.run_angle(self.SET_SPEED, distance_degrees, then=Stop.HOLD, wait=True)
                
            self.resets()

            # orientation control loop with goal
            print("Starting PD for orientation.")
            while not self.reached_target_orient(target_orient):
                self.update_odometry()                          # Get bot changes
                #print("target: {:.2f}, orient: {:.2f}".format(
                #    math.degrees(target_orient), math.degrees(self.orientation)
                #))

                # feedback
                relative_angle = self.normalize_radians(target_orient - self.orientation)
                rotation_degrees = self.calculate_rotation_angle(relative_angle)
                # because of motor setup. switch -rotate to left motor
                # if bot is rotating 270 in both directions then switch -rotate to right motor
                self.left_motor.run_angle(self.MIN_SPEED, -rotation_degrees, then=Stop.HOLD, wait=False)
                self.right_motor.run_angle(self.MIN_SPEED, rotation_degrees, then=Stop.HOLD, wait=True)

            print("Finished: (distance: {:.2f}mm, ang: {:.2f}°)".format(
                self.current_dist, math.degrees(self.orientation)
                ))
        print("** Goal Reached **\n")

    def resets(self):
        self.dX = 0.0                                           # Reset globals
        self.dO = 0.0
        self.current_dist = 0.0
        self.orientation = 0.0

        self.left_motor.reset_angle(0)                          # Reset encoders
        self.right_motor.reset_angle(0)
        self.prev_left = 0
        self.prev_right = 0

        self.stopwatch.reset()                                  # Reset time interval


    """
    GOAL (each waypoint) reached if at target within THRESHOLD
    Parameters in World Frame
    """
    def reached_target_pos(self, target_distance):
        distance = abs(target_distance - self.current_dist)
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
    Find the degrees the motor needs to turn for distance and turns
    """
    def calculate_distance_angle(self, target_distance):
        # Arc length formula: distance = angle * radius
        wheel_circumference = 2 * math.pi * self.wheel_radius
        wheel_rotations = target_distance / wheel_circumference
        degrees = wheel_rotations * 360
        return degrees

    def calculate_rotation_angle(self, target_angle):
        # Calculate arc length each wheel needs to travel
        arc_length = target_angle * self.wheel_base / 2
        
        # Convert arc length to degrees of wheel rotation
        wheel_circumference = 2 * math.pi * self.wheel_radius
        wheel_rotations = arc_length / wheel_circumference
        degrees = wheel_rotations * 360
        #print("arc: {:.2f}, circum: {:.2f}, rotation: {:.2f}, deg: {:.2f}".format(
        #    arc_length, wheel_circumference, wheel_rotations, degrees
        #))
        return degrees

    
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

        #print("speedAngle: {:.2f}/{:.2f} \t\t\tdt: {:.4f} \t\t\tang(L/R): {:.2f}/{:.2f}".format(
        #    wL,wR,dt,angle_left,angle_right
        #    ))

        v = (self.wheel_radius / 2) * (wR + wL)                             # Linear velocity (mm/s)
        w = (self.wheel_radius / self.wheel_base) * (wR - wL)               # Angular velocity (rad/s)

        self.dX = v * dt                                                    # delta x - straight line (mm)
        self.dO = w * dt                                                    # delta orientation (rad)

        self.current_dist += self.dX
        self.orientation += self.dO
        self.orientation = self.normalize_radians(self.orientation)         # normalize to [-PI, PI]

        #print("dist(x): {:.2f} \t\t\torient(O): {:.2f}°\n".format(self.dX,math.degrees(self.orientation)))
