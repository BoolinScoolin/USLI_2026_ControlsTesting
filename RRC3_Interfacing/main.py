import serial
ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
while True:
    line = ser.readline().decode('ascii', errors='ignore').strip()
    if line:
        print(line)