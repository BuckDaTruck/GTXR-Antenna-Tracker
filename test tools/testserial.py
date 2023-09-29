import serial
import time
import struct

ser = serial.Serial('/dev/cu.usbserial-0001', 115200)  # Replace 'COMx' with your serial port

while True:
    value1 = 3.14159
    value2 = 3.2222

    # Convert float values to bytes
    value1_bytes = struct.pack('f', value1)
    value2_bytes = struct.pack('f', value2)

    # Send the bytes over serial
    ser.write(value1_bytes)
    ser.write(value2_bytes)

    time.sleep(2)