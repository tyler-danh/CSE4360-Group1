#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor)
from pybricks.tools import StopWatch
from pybricks.parameters import Port, Stop, Direction
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
wheel_base = 129.87     # AXEL(mm) from center of wheel to center of wheel
wheel_radius = 25.8445  # RADIUS(mm) from (floor to top of wheel) / 2

watch = StopWatch()

# Motor(port, positive_direction=Direction.CLOCKWISE, gears=None)
# CounterClockwise?
motorRight = Motor(Port.A, positive_direction=Direction.CLOCKWISE, gears=None)
motorLeft= Motor(Port.B, positive_direction=Direction.CLOCKWISE, gears=None)

control = Controller(wheel_base, wheel_radius, watch, motorLeft, motorRight)


## CREATE PATH ##
# list of waypoints [(x,y,angle), (x,y,angle), ...]
# x,y in mm, angle in deg
# path = findPath()

# example path (a straight line)
# 1000mm = 1m
path = [(500,0,90)]

# example path (a square)
# path = [(500,0,0), (500,0,90), (500,500,90), (500,500,180),
#        (0,500,180), (0,500,270), (0,0,270)]


## EXECUTE PATH ##
control.follow_path(path)
ev3.speaker.beep()
