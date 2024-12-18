from pybricks.parameters import Color, Stop
from pybricks.media.ev3dev import SoundFile
from pybricks.ev3devices import Motor, UltrasonicSensor, GyroSensor, ColorSensor, TouchSensor
from pybricks.tools import wait
class Explorer:
    def __init__(self, ev3, left_motor, right_motor, gyroscope, ultrasonic, colorSensor, touch):
        self.ev3 = ev3
        self.left_motor = left_motor
        self.right_motor = right_motor
        self.gyroscope = gyroscope
        self.supersonic = ultrasonic
        self.colorsensor = colorSensor
        #self.ultrasonic2 = ultrasonic2
        self.touch = touch

        self.DISTANCE_THRESHOLD = 145 #distance threshold for obstacles in mm
        self.MOVE_POWER = 300/2 #70% power
        self.TURN_POWER = 50 #50% power


    def stop(self):
        self.left_motor.stop(Stop.BRAKE)
        self.right_motor.stop(Stop.BRAKE)

    def turn(self, angle):
        print("turning here")
        self.gyroscope.reset_angle(0)
        current_angle = 0
        if angle > 0:
            print("big")
            while current_angle <= angle:
                current_angle = self.gyroscope.angle()
                print(current_angle)
                self.left_motor.run(-self.MOVE_POWER)
                self.right_motor.run(self.MOVE_POWER)
        else:
            print("small")
            while current_angle >= angle:
                current_angle = self.gyroscope.angle()
                self.left_motor.run(self.MOVE_POWER)
                self.right_motor.run(-self.MOVE_POWER)

    def explore(self):
        print("hello")
        while True:
            self.left_motor.run(self.MOVE_POWER)
            self.right_motor.run(self.MOVE_POWER)
            #if self.ultrasonic2.distance() < 50:
            if self.touch.pressed():
                print("obstacle detected")
                self.ev3.speaker.play_file(SoundFile.OKEY_DOKEY)
                self.stop()
                wait(500)
                self.left_motor.run(-self.MOVE_POWER)
                self.right_motor.run(-self.MOVE_POWER)
                wait(500)
                self.stop()
                self.turn(90)
                print("turning")
            #  if self.supersonic.distance() > self.DISTANCE_THRESHOLD:
            #     self.turn(-90)
            if self.colorsensor.color() == Color.RED:
                self.stop()
                self.ev3.speaker.play_file(SoundFile.OKEY_DOKEY)
                return False    