#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, UltrasonicSensor, GyroSensor, ColorSensor, TouchSensor
from pybricks.tools import StopWatch, wait
from pybricks.parameters import Port, Direction, Color, Stop
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
shovel_motor = Motor(Port.B, positive_direction=Direction.CLOCKWISE, gears=None)

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



# while True:
#     print(shovel_motor.angle())
#     intial_angle = shovel_motor.angle()
#     shovel_motor.track_target(150)
#     wait(500)
#     shovel_motor.track_target(-100)
#     wait(500)

# while True:
#     print(gyro.angle())

""" 
def stop():
    left_motor.stop(Stop.BRAKE)
    right_motor.stop(Stop.BRAKE) 

def turn(angle):
    print("turning: {:.2f}".format(angle))
    gyro.reset_angle(0)
    current_angle = 0
    if angle > 0:
        #print("left")
        while current_angle <= angle:
            current_angle = gyro.angle()
            print(current_angle)
            left_motor.run(-150 )
            right_motor.run(150 )
    else:
        #print("right")
        while current_angle >= angle:
            print(current_angle)
            left_motor.run(150)
            right_motor.run(-150)
            current_angle = gyro.angle()

# sawtooth wander pattern __/|__/|__
print("we are wandering")
stop()
turn(45)

time = 0
watch.reset()
print("straight")
while(time < 2000):
    left_motor.run(150 )
    right_motor.run(150 )
    time = watch.time()

stop()
turn(-135)

time = 0
watch.reset()
print("straight")
while(time < 1500):
    left_motor.run(150)
    right_motor.run(150)
    time =watch.time()   
 """


explorer = Explorer(ev3, left_motor, right_motor,shovel_motor, gyro, ultrasonic, colors, touch, watch)
explorer.explore()
ev3.speaker.play_file(SoundFile.HORN_1)
