from __future__ import print_function # use python 3 syntax but make it compatible with python 2
from __future__ import division # ''

import math
import csv
import time # import the time library for the sleep function
import easygopigo3 as easy# import the GoPiGo3 drivers
GPG = easy.EasyGoPiGo3()

dS = GPG.init_distance_sensor()
dS.read_mm()
ccw = False
top = 5
count = 1
degree = 0
distance = 0
i = 90
trvld = 860
# Create an instance of the GoPiGo3 class. GPG will be the GoPiGo3 object.

try:
GPG.offset_motor_encoder(GPG.MOTOR_LEFT, GPG.get_motor_encoder(GPG.MOTOR_LEFT))
GPG.offset_motor_encoder(GPG.MOTOR_RIGHT, GPG.get_motor_encoder(GPG.MOTOR_RIGHT))
obsta = dS.read_mm()
row = [‘row index’, ‘degree’, ‘distance’] 
with open('problem1_pathtrace.csv','a') as csvFile:
writer = csv.writer(csvFile)
writer.writerow(row)
while True:
s = 0
GPG.set_motor_position(GPG.MOTOR_LEFT + GPG.MOTOR_RIGHT, trvld)
while (s != 25):
s = s + 1
print(s)
time.sleep(.01)
if (dS.read_mm() < 250):
GPG.stop()
trvld = GPG.get_motor_encoder(GPG.MOTOR_RIGHT)
degree = trvld
distance = ((degree/360)*(math.pi*(66.5)))
row = [count, degree, distance] 
with open('problem1_pathtrace.csv','a') as csvFile:
writer = csv.writer(csvFile)
writer.writerow(row)
count = count + 1
GPG.turn_degrees(180)
GPG.reset_motor_encoder(GPG.MOTOR_LEFT + GPG.MOTOR_RIGHT)
i = i*(-1)
GPG.set_motor_position(GPG.MOTOR_LEFT + GPG.MOTOR_RIGHT, trvld)
degree = GPG.get_motor_encoder(GPG.MOTOR_RIGHT)
distance = ((degree/360)*(math.pi*(66.5)))
row = [count, degree, distance] 
with open('problem1_pathtrace.csv','a') as csvFile:
writer = csv.writer(csvFile)
writer.writerow(row)
count = count + 1
time.sleep(1)
GPG.stop()
trvld = 860
GPG.turn_degrees(i)
GPG.reset_motor_encoder(GPG.MOTOR_LEFT + GPG.MOTOR_RIGHT)


except KeyboardInterrupt: # except the program gets interrupted by Ctrl+C on the keyboard.
GPG.reset_all() 
csvFile.close() # Unconfigure the sensors, disable the motors, and restore the LED to the control of the GoPiGo3 firmware.
