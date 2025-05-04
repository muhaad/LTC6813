import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style
import numpy as np
import random
import serial
import serial.tools.list_ports
import time

#initialize serial port

with open("data.csv", 'a') as file:
    ser = serial.Serial()
    ports = serial.tools.list_ports.comports()
    for port_num in range(1, len(ports)  + 1):
        try:
            port_num = 6
            ser.port = f'COM{port_num}' #Arduino serial port  - try statement to find proper serial port
            #ser.baudrate = 100000
            ser.baudrate = 9600
            ser.timeout = 1000 #specify timeout when using readline() in ms
            ser.open()
            if(ser.is_open==True):
                print("ehre")
                break
        except:
            pass
    if ser.is_open==True:
        print("\nAll right, serial port now open. Configuration:\n")
        print(ser, "\n") #print serial parameters
        ser.write(b"debug\n")
        ser.write(b'begin\n')
        start_time = time.time()
        i = 0
        while True:
            line = ser.readline().decode('utf-8').strip()
            if(line == b'serial dump done'):
                print(line)
                break
            else:
                print(line)
                file.write(line)
    file.close()
