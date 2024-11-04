from pybricks.parameters import Color, Stop
from pybricks.media.ev3dev import SoundFile
from pybricks.ev3devices import Motor, UltrasonicSensor, GyroSensor, ColorSensor, TouchSensor
from pybricks.tools import wait

# states of robot
class State:
    def _init_(self):
        self.WANDERING = 1
        self.WALL_FOLLOWING = 2

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

        self.FOLLOW_TIME = 15000        # wall following timer (ms)
        self.DISTANCE_THRESHOLD = 150  # wander / follow threshold (mm)
        self.DISTANCE_FOLLOW = 145      # wall following distance (mm)
        self.MAX_POWER = 300
        self.MOVE_POWER = 300 
        self.MOVE_POWER_1 = 150         # 70% power?
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
                # print(current_angle)
                self.left_motor.run(-self.MOVE_POWER_1 )
                self.right_motor.run(self.MOVE_POWER_1 )
        else:
            print("small")
            while current_angle >= angle:
                # print(current_angle)
                current_angle = self.gyroscope.angle()
                self.left_motor.run(self.MOVE_POWER_1)
                self.right_motor.run(-self.MOVE_POWER_1)

    def drive_forward(self):
        time = 0
        self.stopwatch.reset()
        while(time < 3500):
            if self.check_goal == False:
                self.stop()
                # input firefirgher mode
                return 
            # print("move forward")
            self.left_motor.run(self.MOVE_POWER_1)
            self.right_motor.run(self.MOVE_POWER_1)
            if self.touch.pressed():
                self.current_state = State.WALL_FOLLOWING
                self.obstacle()
                self.stopwatch.reset()
                continue
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
        wait(500)
        self.turn(60)
        self.current_state = State.WALL_FOLLOWING

        # print(self.supersonic.distance())
        # if self.supersonic.distance() >= self.DISTANCE_THRESHOLD:
        #     self.turn(-70)
        #     self.drive_forward()
            
        #     self.current_state = 2
        # else:
        #     # adjusted too 80 due too 90 overshooting 
            # self.turn(70)
            # self.current_state = 2


    # sawtooth wander pattern __/|__/|__
    def wander(self):
        # print("we are wandering")
        self.stop()
        self.turn(-20)
        wait(1000)

        time = 0
        self.stopwatch.reset()
        while(time < 2000):
            self.left_motor.run(self.MOVE_POWER )
            self.right_motor.run(self.MOVE_POWER )
            if self.check_goal() == False:
                return
            if self.touch.pressed():
                self.current_state = State.WALL_FOLLOWING
                self.obstacle()
                self.stopwatch.reset()
                return
            time = self.stopwatch.time()

        self.stop()
        self.turn(20)
        wait(1000)

        time = 0
        self.stopwatch.reset()
        while(time < 2000):
            self.left_motor.run(self.MOVE_POWER)
            self.right_motor.run(self.MOVE_POWER)
            if self.check_goal() == False:
                return
            if self.touch.pressed():
                self.current_state = State.WALL_FOLLOWING
                self.obstacle()
                self.stopwatch.reset()
                return
            time = self.stopwatch.time()

    def wall_follow(self, wall_distance):
        # print("we are wall following")
        # simple error correction
        error = wall_distance - self.DISTANCE_FOLLOW
        correction = error * 0.5

        # Ensure speed is (MIN < speed < MAX)
        left_speed = max(min(self.MOVE_POWER + correction, self.MAX_POWER), -self.MAX_POWER) + 50
        right_speed = max(min(self.MOVE_POWER - correction, self.MAX_POWER), -self.MAX_POWER)
        
        # Adjust motors to maintain consistent wall distance
        self.left_motor.run(left_speed)
        self.right_motor.run(right_speed)

    def explore(self):
        self.stopwatch.reset()
        while True:
            # goal reached?
            if self.check_goal() == False:
                self.stop()
                self.ev3.speaker.play_file(SoundFile.OKEY_DOKEY)
                return False

            # move
            self.left_motor.run(self.MOVE_POWER)
            self.right_motor.run(self.MOVE_POWER)
            
            # State machine
            # WALL_FOLLOW if touch or close to wall
            if self.current_state == State.WANDERING:
                # print("state 1")
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
                # print("state 2")
                wall_distance = self.supersonic.distance()
                if wall_distance > self.DISTANCE_THRESHOLD:
                    print("we passed the wall")
                # this turns us too the right when the wall ends and drives us forward
                    self.turn(-60)
                    self.drive_forward()
                    if wall_distance > self.DISTANCE_THRESHOLD:
                        self.turn(-60)
                        self.drive_forward()                              
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

    def check_goal(self):
        if self.colorsensor.color() == Color.RED:
            return False
        else:
            return True

