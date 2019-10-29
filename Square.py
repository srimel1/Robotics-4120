import easygopigo3 as easy
import gopigo3
from di_sensors import inertial_measurement_unit
import csv
"""from __future__ import print_function # use python 3 syntax but make it compatible with python 2
from __future__ import division       #  """    

my_gopigo = easy.EasyGoPiGo3()
my_distance_sensor = my_gopigo.init_distance_sensor()
GPG = gopigo3.GoPiGo3()
#file.write("Distance Sensor Reading: {} mm ".format(my_distance_sensor.read_mm()))
filename = "./reading.csv"
file = open(filename, 'w+')
min = 250
robot_operating = True
current_distance = my_distance_sensor.read_mm()
    
def end(signal, frame):
    global robot_operating
    print("Stop initiated")
    robot_opperating = False
    

    
    
def drive():
    for i in range(1, 50):
        current_distance = my_distance_sensor.read_mm()
        #my_gopigo.init_distance_sensor()
        if(current_distance < min):
                return False
        my_gopigo.drive_cm(1)
    my_gopigo.turn_degrees(90)
    return True

try:
    while robot_operating:
        if drive() == False:
            my_gopigo.stop()
            my_gopigo.turn_degrees(180)
        index = 1

        


        
        index += 1
except KeyboardInterrupt as E:
    file.close()
    
finally:
    file.write("{}, {}, {}, \n" .format(index,\
                                    GPG.get_motor_encoder(GPG.MOTOR_RIGHT), \
                                    my_distance_sensor.read_mm()))
    


