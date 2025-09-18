import board, busio
import adafruit_lsm6ds.lsm6ds3trc as LSM6DS

i2c = busio.I2C(board.SCL, board.SDA)
sensor = LSM6DS.LSM6DS3TRC(i2c)

print(sensor.gyro)  # test