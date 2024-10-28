class explorer:
    def __init__(self, ev3, left_motor, right_motor, gyroscrope, ultrasonic, axel_length, wheel_radiius, wait):
        self.ev3 = ev3
        self.left_motor = left_motor
        self.right_motor = right motor
        self.gyroscrope = gyroscope
        self.supersonic = supersonic
        self.axel_lengh = axel_length
        self.wheel_radius = wheel_radius
        self.wait = wait

        self.DISTANCE_THRESHOLD = 300 #distance threshold for obstacles in mm
        self.MOVE_SPEED = 300 #mm per sec
        self.TURN_SPEED = 100 #degress per sec


def stop(self):
    self.left_motor.stop(Stop.BRAKE)
    self.right_motor.stop(Stop.BRAKE)

def turn(self, angle):
    self.gyroscope.reset_angle(0)
    if angle > 0:
        self.left_motor.run(-TURN_SPEED)
        self.right_motor.run(TURN_SPEED)
    else:
        self.left_motor.run(TURN_SPEED)
        self.right_motor.run(-TURN_SPEED)

def explore(self):
    while True:
        self.left_motor(MOVE_SPEED)
        self.right_motor(MOVE_SPEED)

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
            self.left_motor.run(-MOVE_SPEED)
            self.right_motor.run(-MOVE_SPEED)
            wait(1000) #move backwards for 10 secs
            stop()


    