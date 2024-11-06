from pybricks.parameters import Color, Stop
from pybricks.media.ev3dev import SoundFile
from pybricks.ev3devices import Motor, UltrasonicSensor, GyroSensor, ColorSensor, TouchSensor
from pybricks.tools import wait
import sys

# states of robot
class State:
    def _init_(self):
        self.WANDERING = 1
        self.WALL_FOLLOWING = 2

class Explorer:
    def __init__(self, ev3, left_motor, right_motor,shovel_motor, gyroscope, ultrasonic, colorSensor, touch, watch):
        self.ev3 = ev3
        self.left_motor = left_motor
        self.right_motor = right_motor
        self.gyroscope = gyroscope
        self.supersonic = ultrasonic
        self.colorsensor = colorSensor
        self.touch = touch 
        self.stopwatch = watch

        self.FOLLOW_TIME = 8000         # wall following timer (ms)
        self.DISTANCE_THRESHOLD = 100   # wander / follow threshold (mm)
        self.DISTANCE_FOLLOW = 60       # wall following distance (mm)
        self.MAX_POWER = 300 
        self.MOVE_POWER = 150

        self.current_state = State.WANDERING

    def stop(self):
        self.left_motor.stop(Stop.BRAKE)
        self.right_motor.stop(Stop.BRAKE)

    def smash_it(self):
        for _ in range(3):
            self.shovel_motor.track_target(150)
            wait(500)
            self.shovel_motor.track_target(-130)
            wait(500)

    def firefight(self):    
        # initial smash
        self.smash_it()
        while True:
            # if we get off goal then correct
            # drive square pattern
            if self.colorsensor.color() != Color.RED:
                self.stop()
                self.turn(90)
                self.drive_forward(1000)

            # fan smash pattern \|/
            self.stop()
            self.left_motor.run(self.MOVE_POWER)
            self.right_motor.run(self.MOVE_POWER)
            wait(1000)
            self.smash_it()

            self.turn(45)
            self.smash_it()

            self.turn(-90)
            self.smash_it()

            self.turn(45)

    def turn(self, angle):
        print("turning: {:.2f}".format(angle))
        self.gyroscope.reset_angle(0)
        current_angle = 0
        if angle > 0:
            #print("left")
            while current_angle <= angle:
                self.left_motor.run(-self.MOVE_POWER )
                self.right_motor.run(self.MOVE_POWER )
                current_angle = self.gyroscope.angle()
                #print(current_angle)
        else:
            #print("right")
            while current_angle >= angle:
                self.left_motor.run(self.MOVE_POWER)
                self.right_motor.run(-self.MOVE_POWER)
                current_angle = self.gyroscope.angle()
                #print(current_angle)
        self.stop()

    # move forward with goal and touch checks
    def drive_forward(self, drive_time):
        time = 0
        self.stopwatch.reset()
        while(time < drive_time):
            self.check_goal()
            self.left_motor.run(self.MOVE_POWER)
            self.right_motor.run(self.MOVE_POWER)
            if self.touch.pressed():
                self.current_state = State.WALL_FOLLOWING
                self.obstacle()
                self.stopwatch.reset()
                return
            time = self.stopwatch.time()
        
    # reset after hitting obstacle
    def obstacle(self):
        print("obstacle detected")
        self.ev3.speaker.play_file(SoundFile.BACKING_ALERT)
        self.stop()
        self.left_motor.run(-self.MOVE_POWER)
        self.right_motor.run(-self.MOVE_POWER)
        wait(1000)
        self.stop()
        self.turn(80)

    # sawtooth wander pattern __/|__/|__
    def wander(self):
        print("we are wandering")
        self.ev3.speaker.play_file(SoundFile.HORN_2)
        self.stop()
        self.turn(45)
        self.drive_forward(3500)
        self.stop()
        self.turn(-90)
        self.drive_forward(2500)

    # wall follow at specific distance from wall
    def wall_follow(self, wall_distance):
        self.ev3.speaker.play_file(SoundFile.HORN_1)
        
        # simple error correction
        error = wall_distance - self.DISTANCE_FOLLOW
        correction = error * 10
        # Ensure speed is (MIN < speed < MAX)
        left_speed = max(min(self.MOVE_POWER + correction, self.MAX_POWER), -self.MAX_POWER)  #+ 25
        right_speed = max(min(self.MOVE_POWER - correction, self.MAX_POWER), -self.MAX_POWER)
        # Adjust motors to maintain consistent wall distance
        self.left_motor.run(left_speed)
        self.right_motor.run(right_speed)

    # entry point - loop (goal, wander, wall follow)
    def explore(self):
        self.stopwatch.reset()
        while True:
            # goal reached?
            self.check_goal()

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
                print("we are wall following")
                wall_distance = self.supersonic.distance()
                if wall_distance > self.DISTANCE_THRESHOLD:
                    print("we passed the wall")
                    # this turns us too the right when the wall ends
                    # and drives us forward
                    self.turn(-90)
                    self.drive_forward(3500) #3500 ~ 1 square
                    self.turn(-90)                 
                    self.current_state = State.WANDERING
                elif self.touch.pressed():
                    self.current_state = State.WALL_FOLLOWING
                    self.obstacle()
                    self.stopwatch.reset()
                elif self.stopwatch.time() > self.FOLLOW_TIME:
                    self.current_state = State.WANDERING
                    self.wander()
                else:
                    self.wall_follow(wall_distance)

    # goal reached so fight fire then exit
    def check_goal(self):
        if self.colorsensor.color() == Color.RED:
            print("goal")
            self.stop()
            self.ev3.speaker.play_file(SoundFile.CONFIRM)
            self.firefight()
            sys.exit()
