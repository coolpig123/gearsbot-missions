#!/usr/bin/env python3

# Import the necessary libraries
import time
import math
from ev3dev2.motor import *
from ev3dev2.sound import Sound
from ev3dev2.button import Button
from ev3dev2.sensor import *
from ev3dev2.sensor.lego import *
from ev3dev2.sensor.virtual import *

# Create the sensors and motors objects
motorA = LargeMotor(OUTPUT_A)
motorB = LargeMotor(OUTPUT_B)
left_motor = motorA
right_motor = motorB
tank_drive = MoveTank(OUTPUT_A, OUTPUT_B)
steering_drive = MoveSteering(OUTPUT_A, OUTPUT_B)

spkr = Sound()
btn = Button()
radio = Radio()

color_sensor_in1 = ColorSensor(INPUT_1)
ultrasonic_sensor_in2 = UltrasonicSensor(INPUT_2)
gyro_sensor_in3 = GyroSensor(INPUT_3)
gps_sensor_in4 = GPSSensor(INPUT_4)
pen_in5 = Pen(INPUT_5)

motor_A = LargeMotor(OUTPUT_A)
motor_B = LargeMotor(OUTPUT_B)

#calculate the wheel circumference
wheel_circumference = 65*3.14

# function to turn the tank 90 degrees
def turn_90_degrees():
    # set the velocities for right and left wheel
    tank_drive.on(25,-25)
    # do not stop the engines until the vehicle turned 90 degrees
    gyro_sensor_in3.wait_until_angle_changed_by(-90)
    # stop the tank
    tank_drive.stop()
    
    
    
# function to move the tank 200 mm
def move_line(distance):
    # set values for acceleration and initial velocity
    acceleration = 10
    velocity = 30
    
    # acceleration part
    for i in range(4):
        tank_drive.on_for_rotations(SpeedRPM(int(velocity)),SpeedRPM(int(velocity)), (distance / wheel_circumference) / 8,brake = False)
        # add acceleration to velocity
        velocity += acceleration
        
    # deceleration part
    acceleration = -10
    for i in range(4):
        tank_drive.on_for_rotations(SpeedRPM(velocity),SpeedRPM(velocity), (distance / wheel_circumference) / 8,brake = False)
        # add acceleration to velocity
        velocity += acceleration
    
# put pen down 
pen_in5.down()

# move 200mm then turn 90 degrees 4 times
dist = [90,325,300,300]
for i in range(4):
    move_line(dist[i])
    turn_90_degrees()
move_line(75)