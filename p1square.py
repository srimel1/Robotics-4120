import time
import easygopigo3 as easy
import queue
import signal
import threading
from math import *
from statistics import mean
from time import sleep

import numpy as np
from curtsies import Input
from di_sensors import inertial_measurement_unit
from easygopigo3 import *

gpg = easy.EasyGoPiGo3()
move = False
block = False









def robotControl(my_distance_sensor):
    turnCount = 0
    direction = 1
    index = 1
    fw = open('problem1_pathtrace.csv', 'w')
    fw.write('"INDEX", "ENCODER VALUE", "DISTANCE"\n')
    ds = my_distance_sensor
    encoderReading = 0
    while turnCount < 4:
        gpg.drive_cm(50, blocking=False)
        previousReading = -1000
        #print(gpg.read_encoders_average())
        while not previousReading == encoderReading:
            distanceReading = my_distance_sensor.read_mm()
            previousReading = encoderReading
            encoderReading = gpg.read_encoders_average()
            fw.write("{}, {}, {}\n".format(index, encoderReading, distanceReading))
            index += 1
            if distanceReading < 250:
                print('Obstruction ahead....Turning...')
                #gpg.stop()
                direction *= -1
                gpg.turn_degrees(90 * direction)
                turnCount = -1
                break
            time.sleep(.05)
        gpg.turn_degrees(90 * direction)
        turnCount += 1
    fw.close()

def Main(trigger):

    simultaneous_launcher = threading.Barrier(4)  # sync 
    sensor_queue = queue.Queue(maxsize=1)  # imu sensor
    obstruction = threading.Event()
    complete = threading.Event()
    moving = threading.Event()

    print("   _____       _____ _  _____         ____  ")
    print("  / ____|     |  __ (_)/ ____|       |___ \ ")
    print(" | |  __  ___ | |__) || |  __  ___     __) |")
    print(" | | |_ |/ _ \|  ___/ | | |_ |/ _ \   |__ < ")
    print(" | |__| | (_) | |   | | |__| | (_) |  ___) |")
    print("  \_____|\___/|_|   |_|\_____|\___/  |____/ ")
    print("                                            ")

    my_distance_sensor = gpg.init_distance_sensor()
    robotControl(my_distance_sensor)
    sys.exit(0)


    try:
        simultaneous_launcher.wait()
    except threading.BrokenBarrierError:
        pass

    # exit codes depending on the issue
    if simultaneous_launcher.broken:
        sys.exit(1)
    sys.exit(0)


if __name__ == "__main__":
    trigger = threading.Event() 
    signal.signal(signal.SIGINT, lambda signum, frame: trigger.set())  
    Main(trigger)