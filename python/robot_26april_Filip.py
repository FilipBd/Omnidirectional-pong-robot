"""Omnidirectional pong playing robot

Authors: Filip Björklund and Christopher Strand
Email: "filipbjo@kth.se", "chstrand@kth.se"
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
"""

import sys
import cv2 as cv
import numpy as np
import RPi.GPIO as GPIO
import time
import math
import smbus

########### FOR TESTING ON PC/MAC (DELETE/COMMENT OUT THIS ON RPI) ###########
# GPIO.py needs to be in same folder
#import GPIO

############################### END OF TEST ###################################




GPIO.setwarnings(False)

################## ROBOT CODE ######################

class Robot:

	def __init__(self):

		#Enviroment
		self.CrashDistance = 5		# Distance from US sensor. Values from sensor below this risk of chrashing the robot 
		self.backDistance = 10 		# Distance from back wall to robot

		#Defiene the wheels 
		self.w1 = 0.0
		self.w2 = 0.0
		self.w3 = 0.0
		

		#For Raspberry. Setting pins. 
		GPIO.setmode(GPIO.BOARD)


		self.freq = 1600


		# CAMERA
		self.src = cv.VideoCapture(0)
		self.src.set(3,160)
		self.src.set(4,120)
		self.distance = 0
		#self.angle = 7.5
		self.x = 0.0005


		#MOTOR - OUTPUTS
		self.motor1PINR = 8
		self.motor2PINR = 12 #
		self.motor3PINR = 18 #
		
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
		self.bus = smbus.SMBus(1)
		self.address = 0x60



	# Camera function
	def camera(self, argv):
		distance = 0
		_,frame = self.src.read()                                        #reads the frames from the camera
		#gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
		blur_frame = cv.GaussianBlur(frame, (3, 3), 0)
		hsv = cv.cvtColor(blur_frame, cv.COLOR_BGR2HSV)                  #converts the colorspace from BGR to HSV


		lower_blue = np.array([98, 75, 121])
		upper_blue = np.array([108, 141, 255])


		# Threshold the HSV image to get only blue colors

		mask = cv.inRange(hsv, lower_blue, upper_blue)              #filters out all colors that are out of the blue color range,

		# Bitwise-AND mask and original image
		#res = cv.bitwise_and(frame, frame, mask=mask)               #puts the

		#cv.imshow('frame', frame)
		#cv.imshow('mask', mask)


		#cv.imshow('res', res)


		rows = hsv.shape[0]                                         #tar ut alla rader fran bilden(640x480)
		blur = cv.GaussianBlur(mask,(3,3),0)

		circles = cv.HoughCircles(blur, cv.HOUGH_GRADIENT, 1, rows / 1, param1=150, param2=15,minRadius=0, maxRadius=0)

		if circles is not None:
			circles = np.uint16(np.around(circles))
			for i in circles[0, :]:
				center = (i[0], i[1])

				# circle center
				cv.circle(blur, center, 1, (0, 100, 100), 3)
				# circle outline
				radius = i[2]
				cv.circle(blur, center, radius, (255, 0, 255), 3)
				distance = (center[0] - 80)
			# gopreturn distance
		# print(radius)
		cv.line(blur, (80, 0), (80, 120), (255, 0, 0), 5)
		print (distance)
		cv.imshow("detected circles", blur)

		X = distance*1.25

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
		print("X = ", X)
		if X == None:
			X = 0
		if Y == None:
			Y = 0
		if R == None:
			R = 0
		self.w1 = (-1/2*(X*1.3)) - (math.sqrt(3)/2*Y) + (radius*R)
		self.w2 = (-1/2*(X*1.3)) + (math.sqrt(3)/2*Y) + (radius*R)
		self.w3 = X*0.7 + (radius*R)
		#print(self.w1)
		#print(self.w3)

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
		motor3abs = abs(motor3)

		
		motor1abs = motor1abs
		motor2abs = motor2abs
		motor3abs = motor3abs

		#print('motor1 absolutbellop: ',motor1abs)
		##print(motor2)
		##print(motor3)

		#if motor1abs or motor2abs or motor3abs > 1.001:
			#raise ValueError('Error with motor value. Check motor inputs.')

		#if  motor3abs > 80 or motor3abs < 20:
		self.motorStop()
		#	return False
		
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


		m1.ChangeDutyCycle(motor1abs)
		m2.ChangeDutyCycle(motor2abs)
		m3.ChangeDutyCycle(motor3abs)
		
		#print("m1 = ", motor1abs)
		#print("m2 = ", motor2abs)
		#print("m3 = ", motor3abs)

		# Ramping up # 5% by each loop 
		#for dc in range(35, 100, step):

			#procent = dc/100 # Todo: Fix the procent calculation
			##print("before m1")
			##print(procent*motor1abs)
			#m1.ChangeDutyCycle(procent*motor1abs)		#Setting the current duty cycle
			#m1.ChangeDutyCycle(75)
			##print("before m2")
			#m2.ChangeDutyCycle(procent*motor2abs)		# 100% will give maximum velocity 
			#m2.ChangeDutyCycle(75)			##print("before m3")
			#m3.ChangeDutyCycle(procent*motor3abs)
			#m3.ChangeDutyCycle(75)

			#if ramp == True:
			#	time.sleep(0.05)



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


def regulate(robot):
	#constants
	KP = 2.7
	KD = 0.0
	KI = 0.0

	#robot = Robot()

	#TARGET = 7
	TARGET = robot.getDegree()	#Initialt. Ta mod 360 senare 


	#print(TARGET)

	SAMPLETIME = 0.01	# Ts (?)

	#create 2 encoders
	#e1 = MockRobotEncoder(1.13)
	#e2 = MockRobotEncoder(0.87)
	#e2.speed = 0.615
	#e1 = robot.getDegree()



	#create robot
	#r = Robot((1,2), (3,4))
	#m1_speed = 0.5
	#m2_speed = 0.5



	#X, Y = robot.camera(argv)


	## FÖR VÅR
	#m1_speed, m2_speed, m3_speed = robot.wheels(X,Y,0)
	m1_speed, m2_speed, m3_speed = robot.wheels(80,0,0)
	# mellan -100 och 100


	#r.value = (m1_speed, m2_speed)
	#robot.motorDrive(m1_speed, m2_speed, m3_speed, False) # No ramping


	prev_error = 0
	prev_error = 0

	sum_error = 0
	sum_error = 0

	e1_sum_error = 0
	e2_sum_error = 0
	e3_sum_error = 0

	e1_prev_error = 0
	e2_prev_error = 0
	e3_prev_error = 0

	#Körs så länge X och Y håller sig till samma tecken, om man ska ändra X och Y i varje steg så kommer PIDen gå för långsamt.
	# Sätt intervall på X och Y som nuvurande error_summan ska köra på?




	while True:
		# Kompasser ökar medsols i värden 
		#e1_error = TARGET - e1.value
		#e2_error = TARGET - e2.value

		#e1value = robot.getDegree()
		#error = TARGET - e1value
		degreeRobot = robot.getDegree()
		degreeError = TARGET - degreeRobot

		print("TARGET = ", TARGET)
		print("Nuvurande vinkel = ", degreeRobot)
		print("Vinkelfel = ", degreeError)

		motor1vinkel = 30
		motor2vinkel = 150
		motor3vinkel = 210
		
		
		#m1_speedin, m2_speedin, m3_speedin = robot.wheels(100,0,0)
		#X, Y = robot.camera(argv)
		#m1_speedin, m2_speedin, m3_speedin = robot.wheels(X,Y,0)
		
		#b1 = math.tan(degreeError)*m1_speedin
		#b2 = math.tan(degreeError)*m2_speedin
		#b3 = math.tan(degreeError)*m3_speedin
		

		e1_error = -1*degreeError
		e2_error = -1*degreeError
		e3_error = -1*degreeError

		#print("e1 = ", e1_error)
		#print("e2 = ", e2_error)
		#print("e3 = ", e3_error)








		e1_sum_error += e1_error
		e2_sum_error += e2_error
		e3_sum_error += e3_error

		e1_adj = (e1_error * KP) + (e1_prev_error * KD) + (e1_sum_error * KI)
		e2_adj = (e2_error * KP) + (e2_prev_error * KD) + (e2_sum_error * KI)
		e3_adj = (e3_error * KP) + (e3_prev_error * KD) + (e3_sum_error * KI)


		#if e1_adj > 20:
		#	e1_adj = 20

		#if e2_adj > 20:
		#	e2_adj = 20

		#if e3_adj > 20:
		#	e3_adj = 20

		#if e1_adj < -20:
		#	e1_adj = -20

		#if e2_adj < -20:
		#	e2_adj = -20
		
		#if e3_adj < -20:
		#	e3_adj = -20

		#e1_prev_error = e1_error
		#e2_prev_error = e2_error
		#e3_prev_error = e3_error

		print("error1 {} error2 {} adj1 {} adj2 {}".format(e1_error, e2_error, e1_adj, e2_adj))

		m1_speed += e1_adj
		m2_speed += e2_adj
		#För vår även
		m3_speed += e3_adj

		#print("e1 {} e2 {} m1 {} m2 {}".format(e1.value, e2.value, m1_speed, m2_speed))

		#e1.speed = m1_speed
		#e2.speed = m2_speed



		# För att få fram en ny hastighet 
		#X, Y = robot.camera(argv)


		print("m1 = " , m1_speed)
		print("m2 = ", m2_speed)
		print("m3 = ", m3_speed)
		
		m1_speed, m2_speed, m3_speed = robot.wheels(60, 0, e3_adj)

		robot.motorDrive(m1_speed, m2_speed, m3_speed, False) # No ramping

		time.sleep(SAMPLETIME)





def main(argv):

	#Usage
	robot = Robot()		# Starting robot

	time.sleep(0)

	#robot.driveX()
	#robot.driveY()
	#wh1, wh2, wh3 = robot.general(0,0,1)
	try:
		#X = robot.camera(argv) # Getting robot vectors X and Y
		regulate(robot)
		#print("XXXX = ", X)
		#wh1, wh2, wh3 = robot.wheels(X,0,0)
		
		#robot.motorDrive(wh1, wh2, wh3) # Ramping up motor
		#X_Latest = 0

		#while True:	
			#v = robot.getDegree()
			#print(v)
		#	X = robot.camera(argv) # Getting robot vectors X and Y
		#	if X == 0:
		#		X = X_Latest
		#	else:
		#		X_Latest = X
		#	print(X_Latest)
			#distanceToBall = Y
		#	distanceToBall = 30 #Hardkodat varde i vantan pa fungerande Y-varde

			# Crash checking from Ultrasound sensors
			#backDistance = robot.USDistance("back")
			#rightDistance = robot.USDistance("right")
			#leftDistance = robot.USDistance("left")
			
			
		#	backDistance = 50
		#	rightDistance = 50
		#	leftDistance = 50 #Hardkodat test
			
			#print("bak är ", backDistance)
			#print("höger är ", rightDistance)
			#print("vänster är ", leftDistance)


		#	if backDistance and rightDistance and leftDistance > robot.CrashDistance:

				#if distanceToBall < 20:
					#driveToBall(robot, X, Y)
					#print("Drive to ball")

		#		wh1, wh2, wh3 = robot.wheels(X,0,0)
		#		robot.motorDrive(wh1, wh2, wh3, False) # No ramping

				
		#	else:
		#		robot.motorStop()




	except KeyboardInterrupt:
		print('program terminated')		
		robot.motorStop()
		cv.destroyAllWindows()
		GPIO.cleanup()


if __name__ == '__main__':
	main(sys.argv[1:])
	#main()



