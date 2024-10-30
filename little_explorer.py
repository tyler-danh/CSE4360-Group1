from pybricks.parameters import Color
from pybricks.media.ev3dev import SoundFile
class Explorer:
    def __init__(self, ev3, left_motor, right_motor, gyroscrope, ultrasonic, colorsensor, axel_length, wheel_radiius, wait):
        self.ev3 = ev3
        self.left_motor = left_motor
        self.right_motor = right_motor
        self.gyroscrope = gyroscope
        self.supersonic = supersonic
        self.colorsensor = colorsensor
        self.axel_lengh = axel_length
        self.wheel_radius = wheel_radius

        self.DISTANCE_THRESHOLD = 100 #distance threshold for obstacles in mm
        self.MOVE_POWER = 70 #70% power
        self.TURN_POWER = 50 #50% power


    def stop(self):
        self.left_motor.stop(Stop.BRAKE)
        self.right_motor.stop(Stop.BRAKE)

    def turn(self, angle):
        self.gyroscope.reset_angle(0)
        current_angle = 0
        if angle > 0:
            while current_angle != angle:
                current_angle = self.gyroscope.angle()
                self.left_motor.dc(MOVE_POWER)
                self.right_motor.dc(MOVE_POWER)
        else:
            while current_angle != angle:
                current_angle = self.gyroscope.angle()
                self.left_motor.dc(MOVE_POWER)
                self.right_motor.dc(-MOVE_POWER)

    def explore(self):
        while True:
            self.left_motor(MOVE_POWER)
            self.right_motor(MOVE_POWER)

            if self.ultrasonic.distance() < DISTANCE_THRESHOLD:
                stop()
                self.ev3.speaker.beep() #obstacle detected

                turn(45)
                if self.ultrasonic.distance() > DISTANCE_THRESHOLD:
                    continue
                turn(-180)
                if self.ultrasonic.distance() > DISTANCE_THRESHOLD:
                    conitnue
                #both directions blocked?
                turn(45)
                self.left_motor.run(-MOVE_POWER)
                self.right_motor.run(-MOVE_POWER)
                self.wait(1000) #move backwards for 10 secs
                stop()
            if self.colorsensor.color() == Color.RED:
                stop()
                self.ev3.speaker.play_file(SoundFile.OKEY_DOKEY)
                return False


    