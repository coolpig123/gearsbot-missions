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
    gyro_sensor_in3.reset()
    gain = 1
    while True:
        error = 90 - gyro_sensor_in3.angle
        correction = gain * error
        if error > 0:
            steering_drive.on(100,correction)
        else:
            break
    steering_drive.on(0,0)
        
    
# function to move the tank 200 mm
def move_line(delay):
    # set values for acceleration and initial velocity
    init = 10
    initial_velocity = 10
    final_velocity = 50
    while initial_velocity < final_velocity:
        tank_drive.on(initial_velocity,initial_velocity)
        initial_velocity += 0.4
        time.sleep(delay)
    while initial_velocity > init:
        tank_drive.on(initial_velocity,initial_velocity)
        initial_velocity -= 0.4
        time.sleep(delay)
    print(initial_velocity)
    
# function to move the tank 200 mm
def move_line_2(delay):
    # set values for acceleration and initial velocity
    init = -10
    initial_velocity = -10
    final_velocity = -50
    while initial_velocity > final_velocity:
        tank_drive.on(initial_velocity,initial_velocity)
        initial_velocity -= 0.4
        time.sleep(delay)
    while initial_velocity < init:
        tank_drive.on(initial_velocity,initial_velocity)
        initial_velocity += 0.4
        time.sleep(delay)
    print(initial_velocity)
# put pen down 
pen_in5.down()

# move 200mm then turn 90 degrees 4 times
move_line_2(0.005)
for i in range(2):
    move_line(0.009)
    turn_90_degrees()
    move_line(0.01)
    turn_90_degrees()