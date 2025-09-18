import socket  # used for live streaming through TCP
import time  # used for delays
import board, busio  # circuitpython
import adafruit_lsm6ds.lsm6ds3trc as LSM6DS  # adafruit

i2c = busio.I2C(board.SCL, board.SDA)
sensor = LSM6DS.LSM6DS3TRC(i2c)

HOST = "192.168.1.1"   # your laptop's IP on the Pi–laptop Ethernet link
PORT = 5000

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((HOST, PORT))

try:
    while True:
        gx, gy, gz = sensor.gyro
        line = f"{gx:.6f},{gy:.6f},{gz:.6f}\n"
        sock.sendall(line.encode("utf-8"))
except KeyboardInterrupt:
    sock.close()