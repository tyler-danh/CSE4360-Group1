from pybricks.parameters import Color, Stop
from pybricks.media.ev3dev import SoundFile
from pybricks.ev3devices import Motor, UltrasonicSensor, GyroSensor, ColorSensor, TouchSensor
from pybricks.tools import wait
from enum import Enum, auto

# states of robot
# auto() handles int assignment automatically
class State(Enum):
    WANDERING = auto()
    WALL_FOLLOWING = auto()

class Explorer:
    def __init__(self, ev3, left_motor, right_motor, gyroscope, ultrasonic, colorSensor, touch, watch):
        self.ev3 = ev3
        self.left_motor = left_motor
        self.right_motor = right_motor
        self.gyroscope = gyroscope
        self.supersonic = ultrasonic
        self.colorsensor = colorSensor
        self.touch = touch
        self.stopwatch = watch

        self.FOLLOW_TIME = 10000        # wall following timer (ms)
        self.DISTANCE_THRESHOLD = 200   # wander / follow threshold (mm)
        self.DISTANCE_FOLLOW = 145      # wall following distance (mm)
        self.MAX_POWER = 300
        self.MOVE_POWER = 150           # 70% power?
        self.TURN_POWER = 50            # 50% power?

        self.current_state = State.WANDERING


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

    # reset after hitting obstacle
    def obstacle(self):
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

    # sawtooth wander pattern __/|__/|__
    def wander():
        self.stop()
        self.turn(45)
        wait(500)

        time = 0
        self.stopwatch.reset()
        while(time < 2000):
            self.left_motor.run(self.MOVE_POWER)
            self.right_motor.run(self.MOVE_POWER)
            if check_goal() == False:
                return
            time = self.stopwatch.time()

        self.stop()
        self.turn(-135)
        wait(1000)

        time = 0
        self.stopwatch.reset()
        while(time < 2000):
            self.left_motor.run(self.MOVE_POWER)
            self.right_motor.run(self.MOVE_POWER)
            if check_goal() == False:
                return
            time = self.stopwatch.time()

    def wall_follow(self, wall_distance):
        # simple error correction
        error = wall_distance - self.DISTANCE_FOLLOW
        correction = error * 0.5

        # Ensure speed is (MIN < speed < MAX)
        left_speed = max(min(self.MOVE_POWER + correction, self.MAX_SPEED), -self.MAX_SPEED)
        right_speed = max(min(self.MOVE_POWER - correction, self.MAX_SPEED), -self.MAX_SPEED)
        
        # Adjust motors to maintain consistent wall distance
        self.left_motor.run(left_speed)
        self.right_motor.run(right_speed)

    def check_goal(self):
        if self.colorsensor.color() == Color.RED:
            return False
        else:
            return True

    def explore(self):
        print("hello")
        self.stopwatch.reset()
        while True:
            # goal reached?
            if check_goal() == False:
                self.stop()
                self.ev3.speaker.play_file(SoundFile.OKEY_DOKEY)
                return False

            # move
            self.left_motor.run(self.MOVE_POWER)
            self.right_motor.run(self.MOVE_POWER)
            
            # State machine
            # WALL_FOLLOW if touch or close to wall
            if self.current_state == State.WANDERING:
                if self.supersonic.distance() <= self.DISTANCE_THRESHOLD:
                    self.current_state = State.WALL_FOLLOWING
                    self.stopwatch.reset()
                elif self.touch.pressed():
                    self.current_state = State.WALL_FOLLOWING
                    self.obstacle()
                    self.stopwatch.reset()
                else:
                    self.wander()
            # WANDER if NOT close to wall or follow timer expired
            elif self.current_state == State.WALL_FOLLOWING:
                wall_distance = self.supersonic.distance()
                if wall_distance > self.DISTANCE_THRESHOLD:
                    self.current_state = State.WANDERING
                elif self.stopwatch.time() > self.FOLLOW_TIME:
                    self.current_state = State.WANDERING
                    self.wander()
                else:
                    self.wall_follow(wall_distance)
