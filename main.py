#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, UltrasonicSensor, GyroSensor, ColorSensor, TouchSensor
from pybricks.tools import StopWatch
from pybricks.parameters import Port, Direction, Color
from pybricks.media.ev3dev import SoundFile

from little_explorer import Explorer

# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.

# Init and beep when ready.
ev3 = EV3Brick()
ev3.speaker.play_file(SoundFile.MOTOR_START)

watch = StopWatch()

gyro = GyroSensor(Port.S2, Direction.COUNTERCLOCKWISE)
print("gyro in")
ultrasonic = UltrasonicSensor(Port.S1)
print("ultrasonic in")
colors = ColorSensor(Port.S3)
print("color sensor in")
#ultrasonic2 = UltrasonicSensor(Port.S4)
touch = TouchSensor(Port.S4)

left_motor = Motor(Port.D, positive_direction=Direction.CLOCKWISE, gears=None)
right_motor = Motor(Port.C, positive_direction=Direction.CLOCKWISE, gears=None)

#gyro.reset_angle(0)
# while ultrasonic.distance() > 150:
#     # angle = gyro.angle()
#     # if angle == 90 or angle == -90:
#     #     gyro.reset_angle(0)
#     #     print("reset angle")
#     # print(angle)

#     # color = colors.color()
#     # if color != Color.BLACK:
#     #     print(color)
#     print(ultrasonic.distance())
#     left_motor.dc(50)
#     right_motor.dc(50)

explorer = Explorer(ev3, left_motor, right_motor, gyro, ultrasonic, colors, touch, watch)
explorer.explore()
ev3.speaker.play_file(SoundFile.HORN_1)
