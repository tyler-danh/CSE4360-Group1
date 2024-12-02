#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, UltrasonicSensor, GyroSensor, ColorSensor, TouchSensor
from pybricks.tools import StopWatch, wait
from pybricks.parameters import Port, Direction, Color, Stop
from pybricks.media.ev3dev import SoundFile

# from little_explorer import Explorer

# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.

DISTANCETHRESHOLD = 1200 #approximate ring diameter

# Init and beep when ready.
ev3 = EV3Brick()

# will probably remove this on fight day so we hunt as soon as possible. 
ev3.speaker.play_file(SoundFile.MOTOR_START)

watch = StopWatch()

gyro = GyroSensor(Port.S1, Direction.COUNTERCLOCKWISE)

color_sensor = ColorSensor(Port.S2) #PLEASE PUT IN PORT FOR COLOR SENSOR

left_motor = Motor(Port.B, positive_direction=Direction.CLOCKWISE, gears=None)
right_motor = Motor(Port.A, positive_direction=Direction.CLOCKWISE, gears=None)

left_sonic = UltrasonicSensor(Port.S3)
right_sonic = UltrasonicSensor(Port.S4) #REMEMBER TO PUT PORTS FOR ULTRASONIC

# left_dist = left_sonic.distance()
# right_dist = right_sonic.distance()

#----------------------SUMO FUNCTIONS-----------------------------#

# primary hunting function 
def lockon():

    print("runnning lockon")

    prev_left = 3000
    prev_right = 3000
    while True:

        edge_detection()
        anti_tip()
        forward()

        sonic_dist = avg_sonic()
        left_dist = sonic_dist[0]
        right_dist = sonic_dist[1]
        print("left sonic reads: ", left_dist)
        print("right sonic reads: ", right_dist)

        # this will attack the object as soon as its in front of the bot
        if abs(right_dist) < 90:
            print("made contact")
            # ev3.speaker.play_file(SoundFile.SHOUTING)
            return False
        # turns left when left sensor sees something closer 
        elif left_dist < right_dist and right_dist > 500.0:
            prev_right = right_dist
            moving_turn(5)
            edge_detection()
            forward()
            wait(200)
        # goes right if we saw something closer on the right than lost it 
        elif prev_right < right_dist:
            moving_turn(-5)
            edge_detection()
            forward()
            wait(200)
        # moves forward whenever right sensor sees somethign closer 
        elif left_dist > right_dist:
            prev_right = right_dist
            forward()
            edge_detection()
            wait(200)
            
# Joseph :
    # floor at home 90 deg turn req input of 80 deg
    # pos is right 
    # neg is left 

# sharp stationary turning
def turn(angle):

    # print("starting turn")
    gyro.reset_angle(0)
    current_angle = 0

    if angle > 0:
        while current_angle <= angle:
            left_motor.dc(-90)
            right_motor.dc(90)
            current_angle = gyro.angle()
            # print("turn left")
            # print("desired angl: ", angle)
            # print("current angle: ", current_angle)
    else:
        while current_angle >= angle:
            left_motor.dc(90)
            right_motor.dc(-90)
            current_angle = gyro.angle()
            # print("turn right")
            # print("desired angl: ", angle)
            # print("current angle: ", current_angle)
    stop()
    # wait allows sensors too update 
    wait(200)
    # print("turn complete")

# allows for forward movement while hunting
# thinking about adjsuting this too a love function too have sensors drive oposite motors with power increaseing too the desired 
#   motor as the distanance decreases 
def moving_turn(angle):

    # print("starting turn")
    gyro.reset_angle(0)
    current_angle = 0

    if angle > 0:
        while current_angle <= angle:
            if edge_detection() or anti_tip():
                return
            left_motor.dc(30)
            right_motor.dc(90)
            current_angle = gyro.angle()
            # print("turn left")
            # print("desired angl: ", angle)
            # print("current angle: ", current_angle)
    else:
        while current_angle >= angle:
            if edge_detection() or anti_tip():
                return
            left_motor.dc(90)
            right_motor.dc(30)
            current_angle = gyro.angle()
            # print("turn right")
            # print("desired angl: ", angle)
            # print("current angle: ", current_angle)
    # wait allows sensors too update 
    wait(200)

# provides the sharp turn for anti_tip, could probably swap out for normal turn function 
def roll_turn(angle):
    print("roll turn")
    gyro.reset_angle(0)
    current_angle = 0

    if angle > 0:
        while current_angle <= angle:
            if edge_detection():
                return
            left_motor.dc(-90)
            right_motor.dc(90)
            current_angle = gyro.angle()
            # print("turn left")
            # print("desired angl: ", angle)
            # print("current angle: ", current_angle)
    else:
        while current_angle >= angle:
            if edge_detection():
                return
            left_motor.dc(90)
            right_motor.dc(-90)
            current_angle = gyro.angle()
            # print("turn right")
            # print("desired angl: ", angle)
            # print("current angle: ", current_angle)
    # wait allows sensors too update 
    wait(200)


def stop():
    left_motor.stop(Stop.BRAKE)
    right_motor.stop(Stop.BRAKE)
# foward and backwards are at 80 anything higher would not give reliable sonic readings 
def forward():
    left_motor.dc(80)
    right_motor.dc(80)
    return

def reverse():
    left_motor.dc(-80)
    right_motor.dc(-80)
    return

# attack function with edgedetextion and anti tipping resets 
# took out the speaker sounds because it slows us down too much. 
def attack():
    while True:    
        lockon()
    #     # ev3.speaker.play_file(SoundFile.T_REX_ROAR)
        while not edge_detection() and not anti_tip():            
            left_motor.dc(100) #SEND IT FORWARD!
            right_motor.dc(100)
        # print(color_sensor.color())
    
    

# May need too adjsut the wait too allow furthur movement backwards 
def edge_detection():
    if color_sensor.color() == Color.GREEN: #maybe change color detection for testing?
        print("at edge")
        stop()
        reverse()
        wait(400)
        turn(90)
        return True
    return False

# get the average of the distances to adjust for any spikes 
def avg_sonic():
    left = left_sonic.distance()
    right = right_sonic.distance()
    for i in range(10):

        left = left + left_sonic.distance()
        right = right + right_sonic.distance()

    return left/10, right/10

# whenever we get lifted the color sensor will sense black after a certain point 
def anti_tip():
    if color_sensor.color() == Color.BLACK:
        roll_turn(-180)
        return True
    return False  

#----------------------SUMO FUNCTIONS----------------------------------#

attack()

ev3.speaker.play_file(SoundFile.HORN_1)
