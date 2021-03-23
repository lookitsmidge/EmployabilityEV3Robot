#!/usr/bin/env python3

# Import the necessary libraries
import time
import math
from ev3dev2.motor import *
from ev3dev2.sound import Sound
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
radio = Radio()

color_sensor_in1 = ColorSensor(INPUT_1)
ultrasonic_sensor_in2 = UltrasonicSensor(INPUT_2)
gyro_sensor_in3 = GyroSensor(INPUT_3)
gps_sensor_in4 = GPSSensor(INPUT_4)
pen_in5 = Pen(INPUT_5)
touch_in6 = TouchSensor(INPUT_6)

motorC = LargeMotor(OUTPUT_C) # Magnet

# VARS
sq_distance = 30 #DEFAULT = 30
bumped = False
turn_speed = 3 #DEFAULT = 3
forwards_time = 2.2 # DEFAULT = 2.2
forwards_percent = 20 # DEFAULT = 20
backwards_time = 1.5 # DEFAULT = 1.5
backwards_percent = 20 # DEFAULT = 20


maze_end_colour = "Red"
maze_start_colour = "Green"
# Function Defining

# This function gets the robot to say something
def speak( text ):
    spkr.speak(text, play_type=Sound.PLAY_NO_WAIT_FOR_COMPLETE)

# This function gets the robot to beep   
def beep(tone, time):
    spkr.play_tone(tone, time, play_type=Sound.PLAY_NO_WAIT_FOR_COMPLETE)

# This function gets the colour from the colour sensor
def getColour():
    return color_sensor_in1.color_name
    
# This function gets whether the touch sensor has been pressed or not
def touchPressed():
    return touch_in6.is_pressed
    
# This function gets the distance from the ultrasonic sensor
def get_distance():
    return ultrasonic_sensor_in2.distance_centimeters

# This function stops all motors on the robot
def stop_all_motors_brake():
    motorA.off(brake=True)
    motorB.off(brake=True)

# This function reverses both motors on the robot
def full_backwards(percent, stime):
    motorA.on(-percent)
    motorB.on(-percent)
    
    time.sleep(stime)
    stop_all_motors_brake()
    
    #END full_backwards()

# This is a function that helps define if the touch sensor has been pressed and stops the motors accordingly
def full_forwardsR():
    if(touchPressed() == 1 ):
        stop_all_motors_brake()
        speak("Ooof")
        
# This function will make the robot go forwards with both motors in a straight line
def full_forwards(percent, stime):
    stime = stime*10
    motorA.on(percent)
    motorB.on(percent)

    i = 0
    while ( i <= stime and touchPressed() == 0):
    #for x in range(0, int(stime)):
        print(( i <= stime and touchPressed() == 0), " Distance: ", i, " Units ")
        full_forwardsR()
        time.sleep(0.1)
        
        if(touch_in6.is_pressed == 1):
            full_backwards(backwards_percent, backwards_time)
            turn_right_deg(turn_speed, 90)
            break
        i += 1
        
        #END LOOP

    stop_all_motors_brake()
    beep(400, 0.5)

  
    # this next piece of code caused issues with stopping
    #steering_drive.on_for_rotations(0, 20, 1) # direction, speed, rotations
    #END full_forwards

# OLD TURN FUNCTIONS - FOR NON GYRO ROBOT
# This function is to turn right
def tturn_right(percent, stime):
    tturn(percent, stime)

# This function is to turn left
def tturn_left(percent, stime):
    tturn(-percent, stime)

# This function is to specify exact turn direction
def tturn(percent, stime):
    left_motor.on(percent)
    right_motor.on(-percent)
    
    time.sleep(stime)
    
    stop_all_motors_brake()

# END OF OLD TURN FUNCTIONS

# This function defines what the robot should do each "step" ( square of the maze travelled ) and how to traverse it
def step():
    if( get_distance() > ( sq_distance ) ):
        # free space to left
        turn_left_deg(turn_speed, 90)
        print("turning left")
        full_forwards(forwards_percent, forwards_time)
    else:
        full_forwards(forwards_percent, forwards_time)
            
        # END IF
    # PSEUDOCODE for this function (can be removed soon)
    #if free to left
        #rotate left
        #move 1 forwards
        #next step
        
    #else
        #move forwards
        #if bumped
            #turn right
        #next step

# This function uses the gyro sensor to turn the robot right a specific amount of degrees
def turn_right_deg(percent, deg):
    stop_all_motors_brake()
    gyro_sensor_in3.reset()
    time.sleep(0.02)
    # BUG - sometimes skips number and never stops spinning - need to rework
    while not gyro_sensor_in3.angle == deg+1:
        left_motor.on(percent)
        right_motor.on(-percent)
        print("Deg: ", gyro_sensor_in3.angle)
        # time.sleep(0.1)
        
    stop_all_motors_brake()
    # END FUNC

# This function uses the gyro sensor to turn the robot left a specific amount of degrees
def turn_left_deg(percent, deg):
    turn_right_deg(-percent, -deg-1)

# This function defines what the robot does when in maze solving mode
def solve_maze():
    
    solved = False
    while( not solved ):
        print(getColour(), "Looking for: ", maze_end_colour )
        if getColour() == maze_end_colour:
            speak("finished")
            stop_all_motors_brake()
            solved = True
            break
        else:
            step()
        
        #if color sensor detects ( finish colour ) - break && solved = True
        
# END OF FUNCTION DECLARATION
# START OF CODE

# THIS IS THE MAIN METHOD FOR THE ROBOT
def main():
    print(getColour())
    print("Starting Main")

    while( True ):
        if ( getColour() == maze_start_colour ):
            print("Maze Found")
            solve_maze()
        
    
    # END WHILE
    
    # END MAIN FUNCTION
    
# TEST CODE
main()

# full_forwards(20, 2.8)
#turn_left_deg(10, 90)

# full_forwards(20, 10)
# time.sleep(1)
# full_backwards(20, 1.5)
# turn_right_deg(10, 90)
