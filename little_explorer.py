from pybricks.parameters import Color, Stop
from pybricks.media.ev3dev import SoundFile
from pybricks.ev3devices import Motor, UltrasonicSensor, GyroSensor, ColorSensor, TouchSensor
from pybricks.tools import wait
# from enum import Enum, auto

# states of robot
# auto() handles int assignment automatically
class State:
    def _init_(self):
        self.WANDERING = 1 #auto()
        self.WALL_FOLLOWING = 2 #auto()

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

        self.FOLLOW_TIME = 10000        # wall following timer (ms)
        self.DISTANCE_THRESHOLD = 100  # wander / follow threshold (mm)
        self.DISTANCE_FOLLOW = 60      # wall following distance (mm)
        self.MAX_POWER = 300
        self.MOVE_POWER = 300 
        self.MOVE_POWER_1 = 150         # 70% power?
        self.TURN_POWER = 50            # 50% power?

        self.current_state = 1


    def stop(self):
        self.left_motor.stop(Stop.BRAKE)
        self.right_motor.stop(Stop.BRAKE)

    def firefight(self):    
        while True:
        # print(shovel_motor.angle())
            self.shovel_motor.track_target(150)
            wait(500)
            self.shovel_motor.track_target(-130)
            wait(500)
        return 
   
    def turn(self, angle):
        print("turning: {:.2f}".format(angle))
        self.gyroscope.reset_angle(0)
        current_angle = 0
        if angle > 0:
            #print("left")
            while current_angle <= angle:
                self.left_motor.run(-self.MOVE_POWER_1 )
                self.right_motor.run(self.MOVE_POWER_1 )
                current_angle = self.gyroscope.angle()
                #print(current_angle)
        else:
            #print("right")
            while current_angle >= angle:
                self.left_motor.run(self.MOVE_POWER_1)
                self.right_motor.run(-self.MOVE_POWER_1)
                current_angle = self.gyroscope.angle()
                #print(current_angle)
        self.stop()

    def drive_forward(self, drive_time):
        time = 0
        self.stopwatch.reset()
        while(time < drive_time):
            if self.check_goal == False:
                self.stop()
                self.ev3.speaker.play_file(SoundFile.OKEY_DOKEY)
                self.firefight()
                return
            self.left_motor.run(self.MOVE_POWER_1)
            self.right_motor.run(self.MOVE_POWER_1)
            if self.touch.pressed():
                self.current_state = 2
                self.obstacle()
                self.stopwatch.reset()
                return
            time = self.stopwatch.time()
        return
        
    # reset after hitting obstacle
    def obstacle(self):
        print("obstacle detected")
        self.ev3.speaker.play_file(SoundFile.OKEY_DOKEY)
        self.stop()
        self.left_motor.run(-self.MOVE_POWER_1)
        self.right_motor.run(-self.MOVE_POWER_1)
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

    def wall_follow(self, wall_distance):
        # print("we are wall following")
        # simple error correction
        self.ev3.speaker.play_file(SoundFile.HORN_1)
        error = wall_distance - self.DISTANCE_FOLLOW
        correction = error * 10

        # Ensure speed is (MIN < speed < MAX)
        left_speed = max(min(self.MOVE_POWER_1 + correction, self.MAX_POWER), -self.MAX_POWER)  #+ 25
        right_speed = max(min(self.MOVE_POWER_1 - correction, self.MAX_POWER), -self.MAX_POWER)
        
        # Adjust motors to maintain consistent wall distance
        self.left_motor.run(left_speed)
        self.right_motor.run(right_speed)

    def explore(self):
        print("hello")
        self.stopwatch.reset()
        while True:
            # goal reached?
            if self.check_goal() == False:
                self.stop()
                self.ev3.speaker.play_file(SoundFile.OKEY_DOKEY)
                self.firefight()
                return False

            # move
            self.left_motor.run(self.MOVE_POWER_1)
            self.right_motor.run(self.MOVE_POWER_1)
            
            # State machine
            # WALL_FOLLOW if touch or close to wall
            if self.current_state == 1:
                # print("we are wandering")
                if self.supersonic.distance() <= self.DISTANCE_THRESHOLD:
                    self.current_state = 2
                    self.stopwatch.reset()
                elif self.touch.pressed():
                    self.current_state = 2
                    self.obstacle()
                    self.stopwatch.reset()
                else:
                    self.wander()
            # WANDER if NOT close to wall or follow timer expired
            elif self.current_state == 2:
                print("we are wall following")
                wall_distance = self.supersonic.distance()
                if wall_distance > self.DISTANCE_THRESHOLD:
                    print("we passed the wall")
                # this turns us too the right when the wall ends and drives us forward
                    self.turn(-90)
                    self.drive_forward(3500)    #3500= 1 square                   
                    self.current_state = 1
                elif self.touch.pressed():
                    self.current_state = 2
                    self.obstacle()
                    self.stopwatch.reset()
                elif self.stopwatch.time() > self.FOLLOW_TIME:
                    self.current_state = 1
                    self.wander()
                else:
                    self.wall_follow(wall_distance)

    def check_goal(self):
        if self.colorsensor.color() == Color.RED:
            return False
        else:
            return True

