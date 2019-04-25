# From https://projects.raspberrypi.org/en/projects/robotPID/4

#from gpiozero import Robot, DigitalInputDevice
#from time import sleep
from time import sleep
import threading

class MockRobotEncoder(object):
    def __init__(self, speed_error):
        self._speed_error = speed_error
        self._value = 0
        self._total = 0
        self.speed = 0.5

        self._t = threading.Thread(
            target = self._mock_encoder,
            args = (0.1,))
        self._t.start()

    def reset(self):
        self._value = 0

    def _mock_encoder(self, interval):
        while True:
            self._increment()
            #sleep differing amounts based on the speed and error introduced
            sleep(interval * (2 - self._speed) * self._speed_error)

    def _increment(self):
        self._value += 1
        self._total += 1

    @property
    def value(self):
        return int(self._value)

    @property
    def speed(self):
        return self._speed

    @property
    def total(self):
        return int(self._total)

    @speed.setter
    def speed(self, value):
        self._speed = value 

#constants
KP = 0.1
KD = 0.025
KI = 0.01

#TARGET = 7
TARGET = robot.getDegree()	#Initialt. Ta mod 360 senare 




SAMPLETIME = 1	# Ts (?)

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
m1_speed, m2_speed, m3_speed = robot.driveX()
# mellan -100 och 100


#r.value = (m1_speed, m2_speed)
robot.motorDrive(m1_speed, m2_speed, m3_speed, False) # No ramping


prev_error = 0
prev_error = 0

sum_error = 0
sum_error = 0


#Körs så länge X och Y håller sig till samma tecken, om man ska ändra X och Y i varje steg så kommer PIDen gå för långsamt.
# Sätt intervall på X och Y som nuvurande error_summan ska köra på?




while True:

	# Kompasser ökar medsols i värden 
    #e1_error = TARGET - e1.value
    #e2_error = TARGET - e2.value

    #e1value = robot.getDegree()
    #error = TARGET - e1value
    degreeRobot = robot.getDegree()
    degreeError = TARGET - degreeError

    e1_error = degreeError*cos(motor1vinkel)	#TODO: tänk ut alla dessa
    e2_error = degreeError*cos(motor2vinkel)
    e3_error = degreeError*cos(motor3vinkel)










    e1_sum_error += e1_error
    e2_sum_error += e2_error
    e3_sum_error += e3_error

    e1_adj = (e1_error * KP) + (e1_prev_error * KD) + (e1_sum_error * KI)
    e2_adj = (e2_error * KP) + (e2_prev_error * KD) + (e2_sum_error * KI)
    e3_adj = (e3_error * KP) + (e3_prev_error * KD) + (e3_sum_error * KI)

    e1_prev_error = e1_error
    e2_prev_error = e2_error
    e3_prev_error = e3_error

    print("error1 {} error2 {} adj1 {} adj2 {}".format(e1_error, e2_error, e1_adj, e2_adj))

    m1_speed += e1_adj
    m2_speed += e2_adj
    #För vår även
    m3_speed += e3_adj
    
    print("e1 {} e2 {} m1 {} m2 {}".format(e1.value, e2.value, m1_speed, m2_speed))

    #e1.speed = m1_speed
    #e2.speed = m2_speed



    # För att få fram en ny hastighet 
    #X, Y = robot.camera(argv)




    robot.motorDrive(m1_speed, m2_speed, m3_speed, False) # No ramping

    # update the robots speed
    #r.value = (m1_speed, m2_speed)

    #e1.reset()
    #e2.reset()

    sleep(SAMPLETIME)