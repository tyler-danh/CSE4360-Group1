#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor)
from pybricks.tools import StopWatch
from pybricks.parameters import Port, Direction
from pybricks.media.ev3dev import SoundFile, ImageFile

from controller import Controller

# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.

# Init and beep when ready.
ev3 = EV3Brick()
ev3.speaker.beep()

"""
# Display a question mark to indicate robot waiting
# Wait until any Brick Button is pressed.
ev3.screen.load_image(ImageFile.QUESTION_MARK)
while not any(ev3.buttons.pressed()):
    wait(10)
ev3.screen.clear()
"""

## INIT CONTROLS ##
wheel_base = 126.1364       # AXEL(mm) from center of wheel to center of wheel
wheel_radius = 25.2984      # RADIUS(mm) from (floor to top of wheel) / 2

watch = StopWatch()

motorRight = Motor(Port.A, positive_direction=Direction.CLOCKWISE, gears=None)
motorLeft= Motor(Port.B, positive_direction=Direction.CLOCKWISE, gears=None)

control = Controller(wheel_base, wheel_radius, watch, motorLeft, motorRight)

## CREATE PATH ##
# list of waypoints [(x,y,angle), (x,y,angle), ...]
# x,y in mm, angle in deg
# path = findPath()

# example path (a straight line)
# 1000mm = 1m
path = [(450,0,0)]

# example path (a square)(reset everything after waypoint)
# path = [(500,0,90), (500,0,90), (500,0,90), (500,0,90)]

# example path (a square)(remember previous position)
# path = [(500,0,90), (500,500,90), (0,500,90), (0,0,90)]

# example path (a square)(remember previous position and orientation)
# path = [(500,0,90), (500,500,180), (0,500,270), (0,0,360)]

## EXECUTE PATH ##
control.follow_path(path)
ev3.speaker.beep()
    
"""
time = time / 1000                              # Convert ms to s
time = time * total_count                       # time x num of loops
wheel_speed = math.radians(wheel_speed)         # Convert degrees to radians

distL = wheel_radius * wheel_speed * time          # Distance traveled = radius * speed * dTime
distR = wheel_radius * wheel_speed * time          # Distance traveled = radius * speed * dTime

v = (wheel_radius / 2) * (wheel_speed + wheel_speed)
w = (wheel_radius / wheel_base) * (wheel_speed + wheel_speed)
dTheta = w * time
dX = v * time * math.cos(0)
dY = v * time * math.sin(0)
print("({:.2f}, {:.2f}, {:.2f})".format(dX, dY, dTheta))
"""

"""
def odom1():
    # Convert angular velocity (deg/s) to linear velocity (mm/s)
    left_v = (L / 360) * (2 * math.pi * wheel_radius)
    right_v = (R / 360) * (2 * math.pi * wheel_radius)
    
    # Calculate the distance traveled by each wheel
    left_dist = left_v * dt
    right_dist = right_v * dt
    
    # Calculate the average distance and change in orientation
    distance = (left_dist + right_dist) / 2
    phi_dot = (right_dist - left_dist) / wheel_base
    
    # Calculate change in x and y
    x_dot = distance * math.cos(orientation + phi_dot / 2)
    y_dot = distance * math.sin(orientation + phi_dot / 2)
    
    # Update position and orientation
    position[0] += x_dot
    position[1] += y_dot
    orientation += phi_dot

    print("v(x,y): {:.2f},{:.2f} \t\t\torient(O): {:.2f} \t\t\t\tpos(x,y): {:.2f},{:.2f}\n".format(
        x_dot,y_dot,phi_dot,position[0],position[1]
        ))
"""
