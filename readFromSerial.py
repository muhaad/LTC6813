import serial
import time
from datetime import datetime

#Replace with your actual serial port
SERIAL_PORT = '/dev/tty/USB0'
BAUD_RATE = 9600

#open the serial port
ser = serial.Serial(SERIAL_PORT, BAUD_RATE)

def read_serial_data():
    while True:
        #Wait until data is available
        if ser.in_waiting > 0:
            #Read the first line to get the number of bytes to read
            num_bytes_line = ser.readline().decode('utf-8').strip()
            num_bytes = int(num_bytes_line)

            #Read the specified number of bytes
            data = ser.read(num_bytes).decode('utf-8').strip()

            #Get the current timestamp
            timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')

            #Save the data to a file with the timestamp
            filename = 'vehicle_data_' + timestamp + '.txt' 
            with open(filename, 'a') as file:
                file.write(data)
            #end funciton once specified number of byte has been read and written to the file
            break
              

try:
    read_serial_data()
except KeyboardInterrupt:
    print("Program terminated.")
finally:
    ser.close()
