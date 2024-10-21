#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor)
from pybricks.tools import StopWatch, wait
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
wheel_base = 110.7186       # AXEL(mm) from center of wheel to center of wheel
wheel_radius = 34.7218      # RADIUS(mm) from (floor to top of wheel) / 2

watch = StopWatch()
watch2 = StopWatch()

motorRight = Motor(Port.A, positive_direction=Direction.CLOCKWISE, gears=None)
motorLeft= Motor(Port.D, positive_direction=Direction.CLOCKWISE, gears=None)

control = Controller(wheel_base, wheel_radius, watch, watch2, wait, motorLeft, motorRight)

## CREATE PATH ##
# list of waypoints [(x,y,angle), (x,y,angle), ...]
# x,y in mm, angle in deg
# robot starts on x axis facing 0 deg
# add 90 for left turn, subtract 90 for right turn

# example path (a straight line)
#path = [(450,0,0)]

# example path (a square)
#path = [(450,0,90), (450,450,180), (0,450,270), (0,0,360)]

# example path
# 450 with left turn, 450 straight, 450 with right turn
#path = [(450,0,90), (450,450,90), (450,450,0)]
path = [(450,0,90), (450,450,180), (0,450,270), (0,0,0)]

## EXECUTE ##
control.follow_path(path)
ev3.speaker.beep()
