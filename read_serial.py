import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style
import numpy as np
import random
import serial

#initialize serial port
with open("data.csv", 'a') as file:
    ser = serial.Serial()
    ser.port = 'COM9' #Arduino serial port  - try statement to find proper serial port
    ser.baudrate = 9600
    ser.timeout = 1000 #specify timeout when using readline() in ms
    ser.open()
    if ser.is_open==True:
        print("\nAll right, serial port now open. Configuration:\n")
        print(ser, "\n") #print serial parameters
        ser.write(b'hello from python\n')

        while True:
            line = ser.readline().decode('utf-8').strip()
            if(line == b'serial dump done'):
                print(line)
                break
            else:
                file.write(line)
            #label, value_str = line.split(":", 1)
            #value = float(value_str)
            #print(f"Received line: {line}")