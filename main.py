#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor)
from pybricks.tools import StopWatch
from pybricks.parameters import Port, Direction
import pathfinder3

#from controller import Controller
from simple_controller import Controller

# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.

# Init and beep when ready.
ev3 = EV3Brick()
ev3.speaker.beep()

## INIT CONTROLS ##
wheel_base = 110.7186       # AXEL(mm) from center of wheel to center of wheel
wheel_radius = 34.7218      # RADIUS(mm) from (floor to top of wheel) / 2

watch = StopWatch()

motorRight = Motor(Port.A, positive_direction=Direction.CLOCKWISE, gears=None)
motorLeft= Motor(Port.D, positive_direction=Direction.CLOCKWISE, gears=None)

#control = Controller(wheel_base, wheel_radius, watch, watch2, wait, motorLeft, motorRight)
simple_control = Controller(wheel_base, wheel_radius, watch, motorLeft, motorRight)

## CREATE PATH ##
# list of waypoints [(dist,angle), (dist,angle), ...]
# dist in mm, angle in deg
# robot starts on x axis facing 0 deg
# add 90 for left turn, subtract 90 for right turn

# example path (a square with left turns)
#path = [(450, 90), (450,90), (450,90), (450,90)]

# example path (a square with left turns)
#path = [(450, -90), (450,-90), (450,-90), (450,-90)]

## EXECUTE ##
simple_control.follow_path(pathfinder3.search())
ev3.speaker.beep()
