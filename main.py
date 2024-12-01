#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, UltrasonicSensor, GyroSensor, ColorSensor, TouchSensor
from pybricks.tools import StopWatch, wait
from pybricks.parameters import Port, Direction, Color, Stop
from pybricks.media.ev3dev import SoundFile

from little_explorer import Explorer

# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.

DISTANCETHRESHOLD = 1200 #approximate ring diameter

# Init and beep when ready.
ev3 = EV3Brick()
ev3.speaker.play_file(SoundFile.HORN_2) #each horn sound is 1 sec
ev3.speaker.play_file(SoundFile.HORN_2)
ev3.speaker.play_file(SoundFile.HORN_2)


watch = StopWatch()

gyro = GyroSensor(Port.S1, Direction.COUNTERCLOCKWISE)

color_sensor = ColorSensor(Port.S2) #PLEASE PUT IN PORT FOR COLOR SENSOR

left_motor = Motor(Port.A, positive_direction=Direction.COUNTERCLOCKWISE, gears=None)
right_motor = Motor(Port.B, positive_direction=Direction.COUNTERCLOCKWISE, gears=None)

left_sonic = UltrasonicSensor(Port.S3)
right_sonic = UltrasonicSensor(Port.S4) #REMEMBER TO PUT PORTS FOR ULTRASONIC

#----------------------SUMO FUNCTIONS-----------------------------#

def lockon():
    while True:
        left_dist = left_sonic.distance()
        right_dist = right_sonic.distance()
        
        if abs(left_dist - right_dist) < 60: #this means target is centered... hopefully
            ev3.speaker.play_file(SoundFile.SHOUTING)
            turn(15)
            return False
        elif left_dist < right_dist:
            turn(30)
            edge_detection()
        elif left_dist > right_dist:
            turn(-30)
            edge_detection

def turn(angle):
    gyro.reset_angle(0)
    current_angle = 0
    if angle > 0:
        while current_angle <= angle:
            left_motor.run(-200)
            right_motor.run(200)
            current_angle = gyro.angle()
    else:
        while current_angle >= angle:
            left_motor.run(200)
            right_motor.run(-200)
            current_angle = gyro.angle()
    stop()
    wait(100)

def stop():
    left_motor.stop(Stop.BRAKE)
    right_motor.stop(Stop.BRAKE)

def attack():
    searching = True
    while searching != False:
        searching = lockon()
    ev3.speaker.play_file(SoundFile.T_REX_ROAR)
    left_motor.dc(100) #SEND IT FORWARD!
    right_motor.dc(100)

def edge_detection():
    if color_sensor.color() == Color.BLUE:
        stop()
        turn(90)

#----------------------SUMO FUNCTIONS----------------------------------#


ev3.speaker.play_file(SoundFile.HORN_1)
