"""Omnidirectional pong playing robot

Authors: Filip Björklund and Christopher Strand
Email: "filipbjo@kth.se", "XX@kth.se"
Date: 04/2019

OTIS: Omnidirectional pong playing robot
Bachelor's Thesis Project in Mechatronics at KTH (ITM)
Course Code: MF133X

Description: This code uses a robot class to define a omnidirectional robot with a Kiwi drive configuration using Raspberry Pi.
It also defines a program in which the robot is able to follow a ball and simulate the video game pong.

Requirements
------------
Python
numpy
OpenCV
Rpi.GPIO
smbus
"""


import sys
import cv2 as cv
import numpy as np
import RPi.GPIO as GPIO
import time
import math
import smbus 
from PID import * 

########### FOR TESTING ON PC/MAC (DELETE/COMMENT OUT THIS ON RPI) ###########
# GPIO.py needs to be in same folder
#import GPIO

############################### END OF TEST ###################################

# 130.229.160.194



GPIO.setwarnings(False)

################## ROBOT CODE ######################

class Robot:

	def __init__(self):

		#Enviroment
		self.CrashDistance = 8		# Distance from US sensor. Values from sensor below this risk of chrashing the robot 
		self.backDistance = 20 		# Distance from back wall to robot

		#Defiene the wheels 
		self.w1 = 0.0
		self.w2 = 0.0
		self.w3 = 0.0
		

		#For Raspberry. Setting pins. 
		GPIO.setmode(GPIO.BOARD)


		self.freq = 1600


		# CAMERA
		self.src = cv.VideoCapture(0)
		self.distance = 0
		self.angle = 7.5
		self.x = 0.0005


		#MOTOR - OUTPUTS
		self.motor1PINR = 8
		self.motor2PINR = 12
		self.motor3PINR = 18
		
		GPIO.setup(self.motor1PINR,GPIO.OUT)
		GPIO.setup(self.motor2PINR,GPIO.OUT)
		GPIO.setup(self.motor3PINR,GPIO.OUT)

		self.pwm1R = GPIO.PWM(self.motor1PINR, self.freq)   # Initialize PWM on pwmPin 100Hz frequency. #Todo: Check frequency
		self.pwm2R = GPIO.PWM(self.motor2PINR, self.freq)   # Initialize PWM on pwmPin 100Hz frequency
		self.pwm3R = GPIO.PWM(self.motor3PINR, self.freq)   # Initialize PWM on pwmPin 100Hz frequency

		self.motor1PINL = 10
		self.motor2PINL = 16
		self.motor3PINL = 22
		
		GPIO.setup(self.motor1PINL,GPIO.OUT)
		GPIO.setup(self.motor2PINL,GPIO.OUT)
		GPIO.setup(self.motor3PINL,GPIO.OUT)

		self.pwm1L = GPIO.PWM(self.motor1PINL, self.freq)   # Initialize PWM on pwmPin 100Hz frequency
		self.pwm2L = GPIO.PWM(self.motor2PINL, self.freq)   # Initialize PWM on pwmPin 100Hz frequency
		self.pwm3L = GPIO.PWM(self.motor3PINL, self.freq)   # Initialize PWM on pwmPin 100Hz frequency

		#ULTRASOUND - INPUTS (ECHO)
		self.UPREcho = 26
		self.UPLEcho = 36
		self.UPBEcho = 40
		#ULTRASOUND - OUTPUTS (TRIGER)
		self.UPRTrig = 24
		self.UPLTrig = 32
		self.UPBTrig = 38


		GPIO.setup(self.UPREcho,GPIO.IN)
		GPIO.setup(self.UPLEcho,GPIO.IN)
		GPIO.setup(self.UPBEcho,GPIO.IN)

		GPIO.setup(self.UPRTrig,GPIO.OUT)
		GPIO.setup(self.UPLTrig,GPIO.OUT)
		GPIO.setup(self.UPBTrig,GPIO.OUT)


		# COMPASS
		self.bus = smbus.SMBus(0)
		self.address = 0x60




	# Camera function
	def camera(self, argv):
		distance = 0
		_,frame = self.src.read()                                        #reads the frames from the camera
		#gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
		blur_frame = cv.GaussianBlur(frame, (11, 11), 0)
		hsv = cv.cvtColor(blur_frame, cv.COLOR_BGR2HSV)                  #converts the colorspace from BGR to HSV


		lower_blue = np.array([20, 100, 130])
		upper_blue = np.array([40, 255, 255])


		# Threshold the HSV image to get only blue colors

		mask = cv.inRange(hsv, lower_blue, upper_blue)              #filters out all colors that are out of the blue color range,

		# Bitwise-AND mask and original image
		#res = cv.bitwise_and(frame, frame, mask=mask)               #puts the

		#cv.imshow('frame', frame)
		#cv.imshow('mask', mask)


		#cv.imshow('res', res)


		rows = hsv.shape[0]                                         #tar ut alla rader fran bilden(640x480)
		blur = cv.GaussianBlur(mask,(15,15),0)

		circles = cv.HoughCircles(blur, cv.HOUGH_GRADIENT, 1, rows / 8, param1=60, param2=30,minRadius=5, maxRadius=300)

		if circles is not None:
			circles = np.uint16(np.around(circles))
			for i in circles[0, :]:
				center = (i[0], i[1])

				# circle center
				cv.circle(blur, center, 1, (0, 100, 100), 3)
				# circle outline
				radius = i[2]
				cv.circle(blur, center, radius, (255, 0, 255), 3)
				distance = (center[0] - 320)
			# gopreturn distance
		# print(radius)
		cv.line(blur, (320, 0), (320, 480), (255, 0, 0), 5)
		# print (distance)
		cv.imshow("detected circles", blur)

		X = distance/3.2

			# Todo fix Y 
		Y = 0
	        ##print('hej')
		k = cv.waitKey(5) & 0xFF
		if k == 27:

			#break

			cv.destroyAllWindows()
			#p.stop()
			GPIO.cleanup()
			robot.motorStop()

		return X ##, Y



	# Functions of wheel movement 
	# Calcuates the vectors for each wheel on a robot with a Kiwi config. 

	# IN: X, Y and R. Desiered directions of the robots movement. Optional: radius from robot center to wheels. 
	# OUT: Val between -1 and 1 for wheel motor movement.
	def wheels(self,X,Y,R, radius = 1):

		if X == None:
			X = 0
		if Y == None:
			Y = 0
		if R == None:
			R = 0
		self.w1 = (-1/2*X) - (math.sqrt(3)/2*Y) + (radius*R)
		self.w2 = (-1/2*X) + (math.sqrt(3)/2*Y) + (radius*R)
		self.w3 = X + (radius*R)


		# Test of calc
		if self.w1 > 100:
			self.w1 = 100
		if self.w1 < -100:
			self.w1 = -100

		if self.w2 > 100:
			self.w2 = 100
		if self.w2 < -100:
			self.w2 = -100

		if self.w3 > 100:
			self.w3 = 100
		if self.w3 < -100:
			self.w3 = -100

		return self.w1, self.w2, self.w3

	# Y full, positive or negative direction. Direction 1 is set as default. 
	def driveY(self, direction = 1):

		self.wheels(0,direction,0)
		return self.w1, self.w2, self.w3

	# X full, positive or negative direction. Direction 1 is set as default. 
	def driveX(self, direction = 1):

		self.wheels(direction,0,0)
		return self.w1, self.w2, self.w3

	# R full, positive or negative direction. Direction 1 is set as default. 
	def rotate(self, direction = 1):

		self.wheels(0,0,direction)
		return self.w1, self.w2, self.w3


	#Movement function. Stop the robot is set as default.
	def general(self, directionX = 0, directionY = 0, rotateMov = 0):

		self.wheels(directionX,directionY,rotateMov)	
		return self.w1, self.w2, self.w3


	def motorDrive(self, motor1, motor2, motor3, ramp = True):
		#motorX is the desired velocites of each motor
		motor1abs = abs(motor1)
		motor2abs = abs(motor2)
		motor3abs = abs(motor2)

		print('motor1 absolutbellop: ',motor1abs)
		##print(motor2)
		##print(motor3)

		#if motor1abs or motor2abs or motor3abs > 1.001:
			#raise ValueError('Error with motor value. Check motor inputs.')

		if motor3abs > 80:
			self.motorStop()
			return False
		
		#time.sleep(0.2)

		if motor1 <= 0:
			#Negative value
			m1 = self.pwm1L	# Setting high to left direction
		elif motor1 > 0:
			m1 = self.pwm1R # Setting high to right direction

		if motor2 <= 0:
			#Negative value
			m2 = self.pwm2L	# Setting high to left direction
		elif motor2 > 0:
			m2 = self.pwm2R # Setting high to right direction

		if motor3 <= 0:
			#Negative value
			m3 = self.pwm3L	# Setting high to left direction
		elif motor3 > 0:
			m3 = self.pwm3R # Setting high to right direction

		#dutycycleFunction
		#Motor 1
		dc=0                               # set dc variable to 0 for 0%
		#self.pwm1R.start(dc)               # Start PWM with 0% duty cycle
		m1.start(dc)
		# Motor 2
		m2.start(dc)

		# Motor 3
		m3.start(dc)


		step = 5 # 5% ramping


		# Ramping up # 5% by each loop 
		for dc in range(60, 100, step):

			procent = dc/100 # Todo: Fix the procent calculation
			##print("before m1")
			##print(procent*motor1abs)
			m1.ChangeDutyCycle(procent*motor1abs)		#Setting the current duty cycle
			##print("before m2")
			m2.ChangeDutyCycle(procent*motor2abs)		# 100% will give maximum velocity 
			##print("before m3")
			m3.ChangeDutyCycle(procent*motor3abs)

			if ramp == True:
				time.sleep(0.05)



		# try:
		# 	while True:                      # Loop until Ctl C is pressed to stop.
		# 		for dc in range(0, 101, 5):    # Loop 0 to 100 stepping dc by 5 each loop
		# 			pwm.ChangeDutyCycle(dc)
		# 			time.sleep(0.05)             # wait .05 seconds at current LED brightness
		# 			print(dc)
		# 		for dc in range(95, 0, -5):    # Loop 95 to 5 stepping dc down by 5 each loop
		# 			pwm.ChangeDutyCycle(dc)
		# 			time.sleep(0.05)             # wait .05 seconds at current LED brightness
		# 			print(dc)
		# except KeyboardInterrupt:
		#   print("Ctl C pressed - ending program")

		#pwm.stop()                         # stop PWM
		#GPIO.cleanup()                     # resets GPIO ports used back to input mode


	def motorStop(self, option = 1):

		if option == 1:
			self.pwm1L.stop()
			self.pwm1R.stop()
			self.pwm2L.stop()
			self.pwm2R.stop()
			self.pwm3L.stop()
			self.pwm3R.stop()
			#GPIO.cleanup() #Do this is main() instead

		if option == 2: #Todo: fix this ramping down so it wont start all pwms and then ramp down

			for dc in range(95, 0, -5):    # Loop 95 to 5 stepping dc down by 5 each loop
					pwm.ChangeDutyCycle(dc)
					pwm.ChangeDutyCycle(dc)
					time.sleep(0.05)
					

	def USDistance(self, sensor):

		if sensor == "back":
			Trig = self.UPBTrig
			Echo = self.UPBEcho

		elif sensor == "right":
			Trig = self.UPRTrig
			Echo = self.UPREcho

		elif sensor == "left":
			Trig = self.UPLTrig
			Echo = self.UPLEcho

		else:
			
			return False


		#Trigger to High
		GPIO.output(Trig, True)

		#Shut down trigger
		time.sleep(0.00001)
		GPIO.output(Trig, False)

		StartTime = time.time()
		StopTime = time.time()

	    # save StartTime
		while GPIO.input(Echo) == 0:
			StartTime = time.time()
	 
	    # save time of arrival
		while GPIO.input(Echo) == 1:
			StopTime = time.time()
	 
	    # time difference between start and arrival
		TimeElapsed = StopTime - StartTime
	    # multiply with the sonic speed (34300 cm/s)
	    # and divide by 2, because there and back
		distance = (TimeElapsed * 34300) / 2
	 
	    #Crash check
		if distance <= self.CrashDistance:
			self.motorStop()


		return distance

		#From online https://tutorials-raspberrypi.com/raspberry-pi-ultrasonic-sensor-hc-sr04/

	def getDegree(self):
        degree1 = self.bus.read_byte_data(self.address, 2)
        degree2 = self.bus.read_byte_data(self.address, 3)
        bearing = (degree1 << 8) + degree2
        bearing = bearing/10.0
        return bearing


class PID:
    """PID Controller
    """

    def __init__(self, P=1, I=0.5, D=0.005):

        self.Kp = P
        self.Ki = I
        self.Kd = D

        self.sample_time = 0.00
        self.current_time = time.time()
        self.last_time = self.current_time

        self.clear()

    def clear(self):
        """Clears PID computations and coefficients"""
        self.SetPoint = 0.0

        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0

        # Windup Guard
        self.int_error = 0.0
        self.windup_guard = 20.0

        self.output = 0.0

    def update(self, feedback_value):
        """Calculates PID value for given reference feedback

        .. math::
            u(t) = K_p e(t) + K_i \int_{0}^{t} e(t)dt + K_d {de}/{dt}

        .. figure:: images/pid_1.png
           :align:   center

           Test PID with Kp=1.2, Ki=1, Kd=0.001 (test_pid.py)

        """
        error = self.SetPoint - feedback_value

        self.current_time = time.time()
        delta_time = self.current_time - self.last_time
        delta_error = error - self.last_error

        if (delta_time >= self.sample_time):
            self.PTerm = self.Kp * error
            self.ITerm += error * delta_time

            if (self.ITerm < -self.windup_guard):
                self.ITerm = -self.windup_guard
            elif (self.ITerm > self.windup_guard):
                self.ITerm = self.windup_guard

            self.DTerm = 0.0
            if delta_time > 0:
                self.DTerm = delta_error / delta_time

            # Remember last time and last error for next calculation
            self.last_time = self.current_time
            self.last_error = error

            self.output = self.PTerm + (self.Ki * self.ITerm) + (self.Kd * self.DTerm)
            
        return self.output;
        

    def setKp(self, proportional_gain):
        """Determines how aggressively the PID reacts to the current error with setting Proportional Gain"""
        self.Kp = proportional_gain

    def setKi(self, integral_gain):
        """Determines how aggressively the PID reacts to the current error with setting Integral Gain"""
        self.Ki = integral_gain

    def setKd(self, derivative_gain):
        """Determines how aggressively the PID reacts to the current error with setting Derivative Gain"""
        self.Kd = derivative_gain

    def setWindup(self, windup):
        """Integral windup, also known as integrator windup or reset windup,
        refers to the situation in a PID feedback controller where
        a large change in setpoint occurs (say a positive change)
        and the integral terms accumulates a significant error
        during the rise (windup), thus overshooting and continuing
        to increase as this accumulated error is unwound
        (offset by errors in the other direction).
        The specific problem is the excess overshooting.
        """
        self.windup_guard = windup

    def setSampleTime(self, sample_time):
        """PID that should be updated at a regular interval.
        Based on a pre-determined sampe time, the PID decides if it should compute or return immediately.
        """
        self.sample_time = sample_time

    def setSetPoint(self, set_point):
        self.SetPoint = set_point



################## PONG CODE ######################



def driveToBall(robot, distanceToCenterpoint, distanceToBall):
	#Maximazing value
	maxVal = 1
	# Close enough value to back wall
	closeEnough = 20

	# Initvalues for position, in order to get back to this position after driving
	InitBack = robot.USDistance("back")
	InitRight = robot.USDistance("right")
	InitLeft = robot.USDistance("left")


	#Calculate vector to ball
	x = distanceToCenterpoint #todo
	y = distanceToBall #todo


	# Get distance to backwall
	backDist = robot.USDistance("back") # Function 


	while backDist < closeEnough:
		#Use vector to get wheel drives
		x, y = robot.camera()
		#x = distanceToCenterpoint
		#y = distanceToBall
		wh1, wh2, wh3 = robot.general(x,y,0)
		wheels = [wh1, wh2, wh3]


		# Maximize vector speed
		wheels[0] = wheels[0]*maxVal
		wheels[1] = wheels[1]*maxVal
		wheels[2] = wheels[2]*maxVal

		#Drive the robot to the ball
		motor1 = wheels[0]
		motor2 = wheels[1]
		motor3 = wheels[2]

		robot.motorDrive(motor1,motor2,motor3)

		backDist = robot.USDistance("back") # Update backDistance

	
	#Drive back 
	if backDistance > normalDistance:

		CurrentBack = robot.USDistance("back")
		CurrentRight = robot.USDistance("right")
		CurrentLeft = robot.USDistance("left")

		# Calculate distance vectors

		diff = CurrentBack-robot.backDistance

		while diff > 5:

			y_d = InitBack-CurrentBack
			x_d = InitRight-CurrentRight #Todo: Make this work for both directions
			
			wh1, wh2, wh3 = robot.general(x_d,y_d,0)
			robot.motorDrive(wh1, wh2, wh3)

			CurrentBack = robot.USDistance("back")
			CurrentRight = robot.USDistance("right")
			CurrentLeft = robot.USDistance("left")

			diff = CurrentBack-robot.backDistance

			# After this loop the robot should be back at the init position




#Regulate steady direction 
#def regulate(robot, X, Y): 
	# initDeg = robot.getDegree()

	# Kp = P
	# Ki = I
	# Kd = D
	# ITerm = 0
	# windup_guard = 1

 #    current_time = time.time()
	# last_time = current_time

	# #radiusOfBoard = 30
	# #widthOfBoard = 60

	# Ts = 1
	# sample_time = Ts

	# # Calculate e - error:

	# # r - Reference
	# # s = v*Ts
	# wh1, wh2, wh3 = robot.wheels(X, Y, 0)

	# r = wh3*Ts

	# setpoint = r-backDistance

	# time.sleep(Ts)

	# #feedback
	# NewbackDistance = robot.USDistance("back")
	# NewrightDistance = robot.USDistance("right")
	# NewleftDistance = robot.USDistance("left")


	# #feedback = r-NewbackDistance
	# feedbackR = r-NewrightDistance
	# feedbackL = r-NewleftDistance

	# if feedbackR > feedbackL:
	# 	#Case: Robot has gone up
	# 	feedback = feedbackR
	# else:
	# 	feedback = feedbackL

	# # Error
	# error = setpoint-feedback


	# current_time = time.time()
	# delta_time = current_time-last_time
	# delta_error = error - last_error

	# if delta_time >= sample_time:
	# 	PTerm = Kp * error
	# 	ITerm += error * delta_time

	# 	if (ITerm <- windup_guard):
	# 		ITerm = -windup_guard
	# 	elif (ITerm > windup_guard):
	# 		ITerm =  windup_guard

    #        self.DTerm = 0.0
    #        if delta_time > 0:
    #            self.DTerm = delta_error / delta_time

            # Remember last time and last error for next calculation
    #        self.last_time = self.current_time
    #        self.last_error = error

    #        self.output = self.PTerm + (self.Ki * self.ITerm) + (self.Kd * self.DTerm)


## FROM GITHUB ###
    #    self.current_time = time.time()
    #    delta_time = self.current_time - self.last_time
    #    delta_error = error - self.last_error

    #    if (delta_time >= self.sample_time):
    #        self.PTerm = self.Kp * error
    #        self.ITerm += error * delta_time

    #        if (self.ITerm < -self.windup_guard):
    #            self.ITerm = -self.windup_guard
    #        elif (self.ITerm > self.windup_guard):
    #            self.ITerm = self.windup_guard

    #        self.DTerm = 0.0
    #        if delta_time > 0:
    #            self.DTerm = delta_error / delta_time

            # Remember last time and last error for next calculation
    #        self.last_time = self.current_time
    #        self.last_error = error

    #        self.output = self.PTerm + (self.Ki * self.ITerm) + (self.Kd * self.DTerm)


 ## END OF GITHUB ## 




	#{\displaystyle e_{k}=r_{k}-y_{k}}
	#{\displaystyle I_{k}=I_{k-1}+T_{s}/T_{i}\cdot e_{k}} {\displaystyle I_{k}=I_{k-1}+T_{s}/T_{i}\cdot e_{k}}
	#{\displaystyle u_{k}=K(e_{k}+I_{k}+T_{d}/T_{s}\cdot (e_{k}-e_{k-1}))} {\displaystyle u_{k}=K(e_{k}+I_{k}+T_{d}/T_{s}\cdot (e_{k}-e_{k-1}))}





def main(argv):

	#Usage
	robot = Robot()		# Starting robot

	time.sleep(2)

	#robot.driveX()
	#robot.driveY()
	#wh1, wh2, wh3 = robot.general(0,0,1)
	try:
		X = robot.camera(argv) # Getting robot vectors X and Y
		wh1, wh2, wh3 = robot.wheels(X,0,0)
		robot.motorDrive(wh1, wh2, wh3) # Ramping up motor
		X_Latest = 0

		# Initial degree 


		while True:	
			X = robot.camera(argv) # Getting robot vectors X and Y
			if X == 0:
				X = X_Latest
			else:
				X_Latest = X
			print (X_Latest)
			#distanceToBall = Y
			distanceToBall = 30 #Hardkodat varde i vantan pa fungerande Y-varde

			# Crash checking from Ultrasound sensors
			backDistance = robot.USDistance("back")
			rightDistance = robot.USDistance("right")
			leftDistance = robot.USDistance("left")
			
			print("bak är ", backDistance)
			print("höger är ", rightDistance)
			print("vänster är ", leftDistance)

			#backDistance = 50
			#rightDistance = 50
			#leftDistance = 50 #Hardkodat test

			if backDistance and rightDistance and leftDistance > robot.CrashDistance:

				#if distanceToBall < 20:
					#driveToBall(robot, X, Y)
					#print("Drive to ball")

				wh1, wh2, wh3 = robot.wheels(X,0,0)
				robot.motorDrive(wh1, wh2, wh3, False) # No ramping
				
			else:
				robot.motorStop()




	except KeyboardInterrupt:
		print('program terminated')		
		robot.motorStop()
		cv.destroyAllWindows()
		GPIO.cleanup()


if __name__ == '__main__':
	main(sys.argv[1:])
	#main()



