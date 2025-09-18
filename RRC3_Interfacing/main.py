import serial

import sys, signal

signal.signal(signal.SIGPIPE, signal.SIG_DFL)
print("Check 1")

ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
print("Check 2")

while True:
    line = ser.readline().decode('ascii', errors='ignore').strip()
    print("Check 3")
    if line:
        print(line)
        print("Check 4")